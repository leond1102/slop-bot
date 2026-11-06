"""Andocken ohne ROS (damit testbar): Marker-Pose, Regelgesetz, Dock-Datei.

Koordinaten: Marker-Pose in base_footprint (x vorn, y links). `normal` ist der Winkel der Markernormalen (zeigt von der
Wand zum Roboter) in base_footprint. Steht der Roboter gerade vor dem Marker und schaut ihn an, ist normal = 180°.

Marker-Rahmen M: Ursprung im Marker, x_M = Normale (von der Wand weg), y_M links davon. Der Roboter soll bei
(stop, 0) stehen und in -x_M schauen (θ_M = 180°).
"""
import math
import os

import numpy as np
import yaml


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------- Marker aus dem Bild
def marker_object_points(size):
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)   # OpenCV: oben links, rechts, …


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def marker_in_base(rvec, tvec, cam_R, cam_t):
    """rvec/tvec: Marker in der Kamera (OpenCV). cam_R/cam_t: Kamera in base_footprint (3×3, 3).
    Rückgabe (x, y, normal) in base_footprint, normal = Winkel der Markernormalen in der Bodenebene."""
    import cv2
    R_cm, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))
    pos = cam_R @ np.asarray(tvec, dtype=np.float64).reshape(3) + cam_t
    n = cam_R @ R_cm @ np.array([0.0, 0.0, 1.0])            # z des Markers zeigt aus der Wand heraus
    return float(pos[0]), float(pos[1]), math.atan2(n[1], n[0]), float(abs(n[2]))


# ---------------------------------------------------------------------- Lage des Roboters zum Marker
def robot_in_marker(mx, my, normal):
    """Roboter im Marker-Rahmen: (x_M, y_M, θ_M)."""
    c, s = math.cos(-normal), math.sin(-normal)
    return c * -mx - s * -my, s * -mx + c * -my, wrap(-normal)


def marker_from_robot(x, y, yaw, xm, ym):
    """Marker (Position in map xm, ym) → in base_footprint, wenn der Roboter bei (x, y, yaw) in map steht."""
    dx, dy = xm - x, ym - y
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def marker_to_map(bx, by, bn, rx, ry, ryaw):
    """Marker aus base_footprint in map, wenn der Roboter bei (rx, ry, ryaw) in map steht → (x, y, normal)."""
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * bx - s * by, ry + s * bx + c * by, wrap(ryaw + bn)


def approach_pose(marker_map, distance):
    """Nav2-Ziel: distance vor dem Marker, Blick zur Wand → (x, y, yaw)."""
    x, y, n = marker_map
    return x + distance * math.cos(n), y + distance * math.sin(n), wrap(n + math.pi)


# ---------------------------------------------------------------------- Regelung
class Servo:
    """Feinfahren auf die Markernormale. Gibt (v, w, Zustand) zurück; Zustand 'fahren' | 'fertig' | 'ungenau'.

    Ein Differentialantrieb fährt nicht seitlich: Der Roboter zielt auf einen Punkt der Normalen, der mit dem Abstand zur
    Linie weiter weg liegt, und richtet sich beim Näherkommen aus. Dreht er sich weiter als turn_only, steht er und dreht
    nur. Die Peilung zum Marker bleibt innerhalb ±max_bearing, damit er nie aus dem Kamerabild läuft.
    """

    def __init__(self, stop=0.48, v_max=0.05, v_min=0.02, w_max=0.3, k_lat=6.0, k_w=1.2, max_cut=math.radians(40),
                 turn_only=math.radians(40), tol_x=0.03, tol_y=0.03, tol_yaw=math.radians(6),
                 max_bearing=math.radians(14), cam_x=0.08):
        self.stop, self.v_max, self.v_min, self.w_max = stop, v_max, v_min, w_max
        self.k_lat, self.k_w, self.max_cut, self.turn_only = k_lat, k_w, max_cut, turn_only
        self.tol_x, self.tol_y, self.tol_yaw = tol_x, tol_y, tol_yaw
        self.max_bearing, self.cam_x = max_bearing, cam_x

    def command(self, mx, my, normal):
        xm, ym, th = robot_in_marker(mx, my, normal)
        err = xm - self.stop                            # Rest bis zum Haltepunkt entlang der Normalen
        yaw_err = wrap(th - math.pi)                    # Abweichung von „Blick zur Wand“
        if err <= self.tol_x:                           # angekommen (oder darüber hinaus)
            ok = abs(ym) <= self.tol_y and abs(yaw_err) <= self.tol_yaw
            return 0.0, 0.0, 'fertig' if ok else 'ungenau'
        # Sollrichtung: zur Wand, geneigt zur Linie hin. Weit weg darf stärker geneigt werden als nahe am Ziel.
        cut = clamp(math.atan(self.k_lat * ym), -self.max_cut, self.max_cut) * clamp(err / 0.4, 0.3, 1.0)
        want = math.pi + cut
        herr = wrap(want - th)
        # Der Marker muss im Bild bleiben: Peilung vom Roboter zum Marker (links +) darf ±max_bearing nicht verlassen.
        # Dreht der Roboter um herr nach links, wandert der Marker um herr nach rechts im Bild.
        bearing = math.atan2(my, mx - self.cam_x)
        herr = clamp(herr, bearing - self.max_bearing, bearing + self.max_bearing)
        w = clamp(self.k_w * herr, -self.w_max, self.w_max)
        if abs(herr) > self.turn_only:
            return 0.0, w, 'fahren'
        v = clamp(0.5 * err, self.v_min, self.v_max) * max(0.0, math.cos(herr)) ** 2
        return v, w, 'fahren'


# ---------------------------------------------------------------------- Datei neben der Karte
def dock_file_for(map_yaml):
    return os.path.splitext(map_yaml)[0] + '.dock.yaml'


def save_dock(path, marker_id, size, x, y, normal):
    data = {'marker_id': int(marker_id), 'marker_size': float(size),
            'x': round(float(x), 4), 'y': round(float(y), 4), 'normal': round(float(normal), 5)}
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f)
    os.replace(tmp, path)
    return data


def load_dock(path):
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    for k in ('marker_id', 'x', 'y', 'normal'):
        if k not in d:
            raise ValueError(f'{path}: {k} fehlt')
    return d


# ---------------------------------------------------------------------- Erkennung (OpenCV)
# Wörterbücher, die bei „nichts gefunden“ zur Fehlersuche durchprobiert werden (nur im Debug-Bild)
PROBE_DICTIONARIES = ['4X4_50', '4X4_100', '4X4_250', '5X5_50', '5X5_100', '5X5_250', '6X6_50', '6X6_250', '7X7_50',
                      'ARUCO_ORIGINAL', 'APRILTAG_16h5', 'APRILTAG_25h9', 'APRILTAG_36h11']


def aruco_dictionary(name):
    """'ARUCO_ORIGINAL' → cv2.aruco-Wörterbuch."""
    import cv2
    const = None
    for cand in ('DICT_' + name.replace('-', '_'), 'DICT_' + name.upper().replace('-', '_')):
        const = getattr(cv2.aruco, cand, None)
        if const is not None:
            break
    if const is None:
        raise ValueError(f'Wörterbuch {name!r} unbekannt (z. B. ARUCO_ORIGINAL, 5X5_100, 4X4_50)')
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(const)
    return cv2.aruco.Dictionary_get(const)


class Found:
    """Ergebnis einer Bildsuche: Ecken (4×2) und IDs aller Treffer, dazu verworfene Kandidaten."""

    def __init__(self, corners=(), ids=(), rejected=()):
        self.corners, self.ids, self.rejected = list(corners), [int(i) for i in ids], list(rejected)


class MarkerDetector:
    """Sucht einen ArUco-Marker mit bestimmter ID und liefert seine Pose in der Kamera (OpenCV-Konvention)."""

    def __init__(self, dictionary='ARUCO_ORIGINAL', marker_id=1, size=0.10):
        import cv2
        self.cv2, self.marker_id, self.size = cv2, int(marker_id), float(size)
        self.dictionary = dictionary
        self.dict = aruco_dictionary(dictionary)
        self._probe = {}                                         # Wörterbuchname → Suche (nur für probe)
        self._det = self._make(self.dict)

    def _make(self, dictionary):
        cv2 = self.cv2
        if hasattr(cv2.aruco, 'ArucoDetector'):                  # OpenCV ≥ 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            return cv2.aruco.ArucoDetector(dictionary, params)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return (dictionary, params)

    def _run(self, det, gray):
        cv2 = self.cv2
        if hasattr(det, 'detectMarkers'):
            corners, ids, rejected = det.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, det[0], parameters=det[1])
        return Found([c.reshape(4, 2).astype(np.float64) for c in corners],
                     ids.flatten() if ids is not None else [],
                     [r.reshape(4, 2).astype(np.float64) for r in rejected])

    def find(self, gray):
        """Alle Marker des eingestellten Wörterbuchs im Bild (jede ID)."""
        return self._run(self._det, gray)

    def probe(self, gray):
        """Fehlersuche: In welchen anderen Wörterbüchern findet sich etwas? → [(Name, [IDs])]"""
        out = []
        for name in PROBE_DICTIONARIES:
            if name.upper() == self.dictionary.upper():
                continue
            if name not in self._probe:
                try:
                    self._probe[name] = self._make(aruco_dictionary(name))
                except ValueError:
                    self._probe[name] = None
            det = self._probe[name]
            if det is None:
                continue
            ids = self._run(det, gray).ids
            if ids:
                out.append((name, sorted(set(ids))))
        return out

    def detect(self, gray, K, dist, cam_R=None):
        """→ (rvec, tvec, ecke_px) oder None (bequem: Suche und Pose in einem)."""
        return self.pose(self.find(gray), K, dist, cam_R)

    def pose(self, found, K, dist, cam_R=None):
        """Pose des Ziel-Markers aus einer Suche → (rvec, tvec, ecken_px) oder None. Mehrere Treffer mit derselben ID:
        der größte.

        Ein quadratischer Marker hat aus einem Bild zwei mögliche Lagen (gespiegelte Neigung). Es zählen nur Lösungen,
        deren Normale zur Kamera zeigt. Bewertung: Bildfehler in px, mit cam_R (Kamera in base_footprint) plus 20 ×
        Neigung der Normalen aus der Waagrechten (der Marker hängt an einer senkrechten Wand); die kleinste gewinnt."""
        cv2 = self.cv2
        best = None
        for c, i in zip(found.corners, found.ids):
            if i == self.marker_id:
                area = cv2.contourArea(c.astype(np.float32))
                if best is None or area > best[0]:
                    best = (area, c)
        if best is None:
            return None
        pts = best[1]
        K, dist = np.asarray(K, dtype=np.float64), np.asarray(dist, dtype=np.float64)
        cands = []
        for rv, tv, err in self._poses(pts, K, dist):
            R, _ = cv2.Rodrigues(rv)
            z = R[:, 2]
            if tv[2] <= 0 or float(z @ tv) >= 0:             # hinter der Kamera oder Rückseite zur Kamera
                continue
            score = err + (20.0 * abs(float((cam_R @ z)[2])) if cam_R is not None else 0.0)   # px + Neigung der Normalen
            cands.append((score, rv, tv))
        if not cands:
            return None
        _, rvec, tvec = min(cands, key=lambda c: c[0])
        return rvec, tvec, pts

    def _poses(self, pts, K, dist):
        """Mögliche Lagen (rvec, tvec, Bildfehler px). IPPE zuerst; liefert es nichts Brauchbares (bekannt: NaN oder
        falsche Lage, wenn der Marker genau auf Kamerahöhe und nur um die Hochachse gedreht hängt), iterativ mit
        mehreren Startneigungen. Nur Lösungen mit Bildfehler < 3 px."""
        cv2 = self.cv2
        obj = marker_object_points(self.size)

        def error(rv, tv):
            proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
            return float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - pts) ** 2, axis=1))))

        out = []
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        for rv, tv in zip(rvecs, tvecs):
            rv, tv = rv.reshape(3), tv.reshape(3)
            if np.all(np.isfinite(rv)) and np.all(np.isfinite(tv)) and error(rv, tv) < 3.0:
                out.append((rv, tv, error(rv, tv)))
        if out:
            return out
        width = max(1.0, float(np.linalg.norm(pts[0] - pts[1])))
        z0 = K[0, 0] * self.size / width
        c = pts.mean(axis=0)
        t0 = np.array([(c[0] - K[0, 2]) * z0 / K[0, 0], (c[1] - K[1, 2]) * z0 / K[1, 1], z0])
        for tilt in (-0.7, 0.0, 0.7):                          # Marker zeigt zur Kamera, um die Hochachse geneigt
            R0 = np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]])
            ct, st = math.cos(tilt), math.sin(tilt)
            R0 = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]]) @ R0
            rv0, _ = cv2.Rodrigues(R0)
            ok, rv, tv = cv2.solvePnP(obj, pts, K, dist, rv0.copy(), t0.reshape(3, 1).copy(), True, cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            rv, tv = rv.reshape(3), tv.reshape(3)
            e = error(rv, tv)
            if e < 3.0 and all(np.linalg.norm(rv - o[0]) > 0.05 or np.linalg.norm(tv - o[1]) > 0.01 for o in out):
                out.append((rv, tv, e))
        return out


# ---------------------------------------------------------------------- Debug-Bild
_ASCII = str.maketrans({'ä': 'ae', 'ö': 'oe', 'ü': 'ue', 'Ä': 'Ae', 'Ö': 'Oe', 'Ü': 'Ue', 'ß': 'ss', '°': ' Grad',
                        '→': '->', '–': '-', '„': '"', '“': '"', '≤': '<=', '≥': '>=', '×': 'x'})


def ascii_text(text):
    """OpenCV zeichnet nur ASCII: Umlaute, Grad und Pfeile umschreiben."""
    return text.translate(_ASCII).encode('ascii', 'replace').decode('ascii')


def draw_debug(gray, found, target_id, lines=(), hint_lines=()):
    """Kamerabild (grau) mit Overlay → BGR. Grün = Ziel-ID, orange = andere ID, grau dünn = verworfene Kandidaten
    (sahen quadratisch aus, ergaben aber keinen gültigen Code). lines/hint_lines: Text oben (weiß) bzw. darunter (gelb)."""
    import cv2
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if gray.ndim == 2 else gray.copy()
    h, w = vis.shape[:2]
    cv2.line(vis, (w // 2, 0), (w // 2, h), (90, 90, 90), 1)             # Bildmitte
    for r in found.rejected[:30]:
        cv2.polylines(vis, [r.astype(np.int32).reshape(-1, 1, 2)], True, (150, 150, 150), 1)
    for c, i in zip(found.corners, found.ids):
        col = (0, 255, 0) if i == target_id else (0, 165, 255)
        pts = c.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts], True, col, 3)
        cv2.circle(vis, tuple(pts[0, 0]), 6, (0, 0, 255), -1)             # Ecke 0 (oben links des Markers)
        cv2.putText(vis, 'ID %d' % i, (int(c[:, 0].min()), max(18, int(c[:, 1].min()) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
    import textwrap
    y = 20
    rows = []
    for color, group in (((255, 255, 255), lines), ((0, 255, 255), hint_lines)):
        for t in group:
            rows += [(part, color) for part in textwrap.wrap(ascii_text(t), max(20, int(w / 8.5))) or ['']]
    for text, color in rows:
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(vis, (4, y - th - 4), (8 + tw, y + 5), (0, 0, 0), -1)
        cv2.putText(vis, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        y += th + 10
    return vis
