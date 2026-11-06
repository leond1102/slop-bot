"""Tests für den Andock-Kern (ohne ROS)."""
import math

import numpy as np
import pytest

from slop_dock import dock_core as dc

K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1.0]])
DIST = np.zeros(5)
# Kamera im base_footprint: 8 cm vor der Achse, 15 cm hoch, optisches Koordinatensystem (z vor, x rechts, y unten)
CAM_R = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])
CAM_T = np.array([0.08, 0.0, 0.15])


def render(marker_x, marker_y, normal, size=0.10, z=0.15, noise=0.0):
    """Bild eines ARUCO_ORIGINAL-Markers ID 1, der bei (marker_x, marker_y) in base_footprint auf Höhe z hängt und dessen
    Normale (von der Wand zum Roboter) in Richtung normal zeigt."""
    cv2 = pytest.importorskip('cv2')
    d = dc.aruco_dictionary('ARUCO_ORIGINAL')
    img = cv2.aruco.generateImageMarker(d, 1, 200) if hasattr(cv2.aruco, 'generateImageMarker') \
        else cv2.aruco.drawMarker(d, 1, 200)
    img = cv2.copyMakeBorder(img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)      # weißer Rand
    # Marker-Achsen in base_footprint: z_M = Normale, x_M = nach rechts vom Roboter aus gesehen, y_M = oben
    n = np.array([math.cos(normal), math.sin(normal), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, n)                                   # vom Betrachter (Kamera) aus gesehen rechts
    R_bm = np.column_stack([right, up, n])
    t_b = np.array([marker_x, marker_y, z])
    R_cb = CAM_R.T
    R_cm = R_cb @ R_bm
    t_cm = R_cb @ (t_b - CAM_T)
    rvec, _ = cv2.Rodrigues(R_cm)
    half = size / 2 * (240 / 200)                             # inkl. Rand
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    pts, _ = cv2.projectPoints(obj, rvec, t_cm, K, DIST)
    src = np.array([[0, 0], [239, 0], [239, 239], [0, 239]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, pts.reshape(4, 2).astype(np.float32))
    out = cv2.warpPerspective(img, H, (640, 480), borderValue=128)
    if noise:
        out = np.clip(out + np.random.default_rng(1).normal(0, noise, out.shape), 0, 255).astype(np.uint8)
    return out


@pytest.mark.parametrize('mx,my,normal_deg', [(0.9, 0.0, 180), (0.7, 0.12, 170), (0.6, -0.1, -160), (1.1, 0.2, 150),
                                              (1.1, 0.0, 180), (1.0, 0.05, 140), (0.45, 0.0, 180)])
def test_detect_matches_truth(mx, my, normal_deg):
    img = render(mx, my, math.radians(normal_deg))
    hit = dc.MarkerDetector('ARUCO_ORIGINAL', 1, 0.10).detect(img, K, DIST, CAM_R)
    assert hit is not None
    x, y, normal, tilt = dc.marker_in_base(hit[0], hit[1], CAM_R, CAM_T)
    assert (x, y) == pytest.approx((mx, my), abs=0.02)
    assert dc.wrap(normal - math.radians(normal_deg)) == pytest.approx(0, abs=math.radians(4))
    assert tilt < 0.1                                          # Marker steht senkrecht


def test_detect_with_sensor_noise():
    img = render(1.0, 0.05, math.radians(165), noise=12.0)
    hit = dc.MarkerDetector('ARUCO_ORIGINAL', 1, 0.10).detect(img, K, DIST, CAM_R)
    assert hit is not None
    x, y, normal, _ = dc.marker_in_base(hit[0], hit[1], CAM_R, CAM_T)
    assert (x, y) == pytest.approx((1.0, 0.05), abs=0.03)
    assert dc.wrap(normal - math.radians(165)) == pytest.approx(0, abs=math.radians(8))


def test_other_id_and_empty_image_not_found():
    cv2 = pytest.importorskip('cv2')
    img = render(0.8, 0, math.pi)
    assert dc.MarkerDetector('ARUCO_ORIGINAL', 2, 0.10).detect(img, K, DIST) is None
    assert dc.MarkerDetector('4X4_50', 1, 0.10).detect(img, K, DIST) is None
    assert dc.MarkerDetector('ARUCO_ORIGINAL', 1, 0.10).detect(np.full((480, 640), 128, np.uint8), K, DIST) is None


def test_unknown_dictionary():
    pytest.importorskip('cv2')
    with pytest.raises(ValueError):
        dc.aruco_dictionary('QUATSCH')


def test_geometry_round_trip():
    # Roboter in map, Marker in map → Marker in base → zurück in map
    rx, ry, ryaw = 2.0, 1.0, math.radians(30)
    mm = (3.5, 1.8, math.radians(-150))
    bx, by = dc.marker_from_robot(rx, ry, ryaw, mm[0], mm[1])
    back = dc.marker_to_map(bx, by, dc.wrap(mm[2] - ryaw), rx, ry, ryaw)
    assert back[:2] == pytest.approx(mm[:2]) and dc.wrap(back[2] - mm[2]) == pytest.approx(0, abs=1e-9)


def test_approach_pose_faces_marker():
    x, y, yaw = dc.approach_pose((3.0, 1.0, math.pi), 0.8)       # Normale zeigt nach -x → Roboter steht bei x = 2,2
    assert (x, y) == pytest.approx((2.2, 1.0), abs=1e-9)
    assert yaw == pytest.approx(0, abs=1e-9)                     # schaut nach +x, zur Wand


def test_robot_in_marker_straight_ahead():
    xm, ym, th = dc.robot_in_marker(0.8, 0.0, math.pi)           # Marker 0,8 m geradeaus, Normale zeigt zurück
    assert (xm, ym) == pytest.approx((0.8, 0.0), abs=1e-9)
    assert abs(dc.wrap(th - math.pi)) < 1e-9


# ---------------------------------------------------------------------- Regelung im Kreis (Einrad-Modell)
def simulate(x, y, yaw, wall_x=3.0, steps=4000, dt=0.05, servo=None, delay=0.1):
    """Wand bei x = wall_x, Marker bei (wall_x, 0) mit Normale 180°. Gibt (Zustand, x, y, yaw, Schritte) zurück."""
    servo = servo or dc.Servo()
    queue = [(0.0, 0.0)] * int(delay / dt)
    simulate.max_bearing = 0.0
    for i in range(steps):
        mx, my = dc.marker_from_robot(x, y, yaw, wall_x, 0.0)
        simulate.max_bearing = max(simulate.max_bearing, abs(math.atan2(my, mx - 0.08)))
        v, w, state = servo.command(mx, my, dc.wrap(math.pi - yaw))
        if state != 'fahren':
            return state, x, y, yaw, i
        queue.append((v, w))
        v, w = queue.pop(0)
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
    return 'zeit', x, y, yaw, steps


@pytest.mark.parametrize('y', [-0.15, -0.08, 0.0, 0.08, 0.15])
@pytest.mark.parametrize('yaw', [-0.26, -0.13, 0.0, 0.13, 0.26])
def test_servo_arrives_centered_from_nav2_tolerance(y, yaw):
    """Anfahrpunkt 1,1 m vor der Wand; Nav2 liefert ±15 cm quer und ±0,26 rad."""
    state, x, yf, yawf, _ = simulate(3.0 - 1.1, y, yaw)
    assert state == 'fertig', (state, x, yf, yawf)
    assert abs(yf) <= 0.03 and abs(yawf) <= math.radians(6)
    assert simulate.max_bearing <= math.radians(26)                  # Marker blieb im Bild (Kamera 640×480 sieht ±27°)
    assert wall_gap(x) == pytest.approx(0.48 - 0.08, abs=0.03)      # Kamera (8 cm vor der Achse) 0,40 m vor der Wand


@pytest.mark.parametrize('start', [(1.9, 0.0, 0.5), (1.9, 0.1, -0.5), (1.9, 0.0, 1.0), (1.8, 0.2, 0.3), (1.5, -0.2, -0.3)])
def test_servo_arrives_from_far_and_turned(start):
    state, x, y, yaw, _ = simulate(*start)
    assert state == 'fertig', (state, x, y, yaw)


def wall_gap(x):
    return 3.0 - x - 0.08


def test_servo_never_exceeds_speed_limits():
    servo = dc.Servo()
    for xm in (0.6, 1.0, 1.5):
        for ym in (-0.3, 0.0, 0.3):
            for th in np.linspace(-math.pi, math.pi, 13):
                mx, my = -(xm * math.cos(th) - ym * math.sin(th)), -(xm * math.sin(th) + ym * math.cos(th))
                v, w, _ = servo.command(mx, my, dc.wrap(-th))
                assert 0.0 <= v <= servo.v_max + 1e-9 and abs(w) <= servo.w_max + 1e-9


def test_servo_reports_inaccurate_when_too_far_off():
    servo = dc.Servo()
    # Haltepunkt erreicht, aber 10 cm seitlich → nicht „fertig“
    mx, my = servo.stop, 0.10
    v, w, state = servo.command(mx, my, math.pi)
    assert (v, w) == (0.0, 0.0) and state == 'ungenau'


def test_servo_turns_in_place_when_facing_away():
    # Roboter 0,8 m vor der Wand, aber um ~115° von ihr weggedreht → erst auf der Stelle drehen
    mx, my = dc.marker_from_robot(2.2, 0.0, 2.0, 3.0, 0.0)
    v, w, state = dc.Servo().command(mx, my, dc.wrap(math.pi - 2.0))
    assert state == 'fahren' and v == 0.0 and w != 0.0


def test_dock_file_round_trip(tmp_path):
    p = str(tmp_path / 'office-3.dock.yaml')
    assert dc.dock_file_for('/m/office-3.yaml') == '/m/office-3.dock.yaml'
    dc.save_dock(p, 1, 0.10, 3.14159265, -1.0, 2.5)
    d = dc.load_dock(p)
    assert d['marker_id'] == 1 and d['x'] == pytest.approx(3.1416) and d['normal'] == pytest.approx(2.5)
    (tmp_path / 'kaputt.yaml').write_text('x: 1')
    with pytest.raises(ValueError):
        dc.load_dock(str(tmp_path / 'kaputt.yaml'))


# ---------------------------------------------------------------------- Debug-Hilfen
def test_find_reports_all_ids_and_other_dictionary():
    img = render(1.0, 0.0, math.pi)
    det = dc.MarkerDetector('ARUCO_ORIGINAL', 2, 0.10)                  # sucht ID 2, an der Wand hängt ID 1
    found = det.find(img)
    assert found.ids == [1]
    assert det.pose(found, K, DIST, CAM_R) is None
    wrong = dc.MarkerDetector('4X4_50', 1, 0.10)                 # falsches Wörterbuch → nichts, aber probe findet es
    assert wrong.find(img).ids == []
    hits = wrong.probe(img)
    assert ('ARUCO_ORIGINAL', [1]) in hits


def test_probe_empty_image():
    assert dc.MarkerDetector('ARUCO_ORIGINAL', 1, 0.10).probe(np.full((480, 640), 120, np.uint8)) == []


def test_draw_debug_overlay():
    img = render(1.0, 0.0, math.pi)
    det = dc.MarkerDetector('ARUCO_ORIGINAL', 1, 0.10)
    found = det.find(img)
    vis = dc.draw_debug(img, found, 1, ['Zeile eins', 'Zeile zwei'], ['Hinweis'])
    assert vis.shape == (480, 640, 3) and vis.dtype == np.uint8
    assert (vis[:, :, 1] > vis[:, :, 0] + 100).any()             # grüner Rahmen um das Ziel
    plain = dc.draw_debug(np.full((480, 640), 128, np.uint8), dc.Found(), 1)
    assert plain.shape == (480, 640, 3)


def test_apriltag_dictionary_name_with_lowercase_h():
    pytest.importorskip('cv2')
    dc.aruco_dictionary('APRILTAG_36h11')
    dc.aruco_dictionary('apriltag_36h11')


def test_debug_text_is_ascii_and_wrapped():
    assert dc.ascii_text('Wörterbuch 5X5 → ok, 5°') == 'Woerterbuch 5X5 -> ok, 5 Grad'
    img = np.full((480, 640), 128, np.uint8)
    vis = dc.draw_debug(img, dc.Found(), 1, ['x' * 400], ['Wörter ' * 40])       # darf weder abstürzen noch aus dem Bild laufen
    assert vis.shape == (480, 640, 3)
