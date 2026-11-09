"""Greifergeometrie ohne ROS: Objektmitte/-breite in Y ↔ Backenpositionen. Alles in Metern.

Aufbau: Zwei Spindeln übereinander, ihre Mitte liegt auf der X-Achse des Roboters, die Backen fahren in ±Y.
Position einer Backe p = Weg von ihrem äußeren Anschlag (Null nach dem Homen) nach INNEN, 0 = ganz offen.

Bei ganz offenen Backen liegen ihre Innenflächen open_width auseinander, symmetrisch zur X-Achse:
  Backe auf der +Y-Seite:  Innenfläche y = +open_width/2 - p
  Backe auf der -Y-Seite:  Innenfläche y = -open_width/2 + p
Ein Objekt mit Mitte y_c und Breite w wird von den Innenflächen bei y_c ± w/2 berührt; squeeze (Gesamtquetschung)
fährt beide Backen je squeeze/2 weiter hinein.
"""


class GripperError(ValueError):
    pass


class Geometry:
    def __init__(self, open_width, travel, squeeze=0.0005, left_is_plus_y=False):
        if open_width <= 0:
            raise GripperError('open_width (Abstand der Backen-Innenflächen bei offenem Greifer) muss > 0 sein')
        if travel <= 0:
            raise GripperError('travel muss > 0 sein')
        self.open_width = float(open_width)
        self.travel = float(travel)
        self.squeeze = float(squeeze)
        self.left_is_plus_y = bool(left_is_plus_y)

    # ------------------------------------------------------------------ Umrechnung
    def jaws_from_object(self, y_center, width):
        """Backenpositionen (p_links, p_rechts) [m von außen], die ein Objekt (Mitte, Breite) klemmen."""
        if width <= 0:
            raise GripperError('Objektbreite muss > 0 sein')
        face_plus = y_center + width / 2.0 - self.squeeze / 2.0     # Innenfläche der +Y-Backe beim Klemmen
        face_minus = y_center - width / 2.0 + self.squeeze / 2.0
        p_plus = self.open_width / 2.0 - face_plus
        p_minus = face_minus + self.open_width / 2.0
        p_left, p_right = (p_plus, p_minus) if self.left_is_plus_y else (p_minus, p_plus)
        self.check(p_left, p_right, f'Objekt y={y_center:+.3f} m, Breite {width:.3f} m')
        return p_left, p_right

    def shift(self, p_left, p_right, dy):
        """Beide Backen so verschieben, dass das gehaltene Objekt um dy (+Y) wandert."""
        d_plus, d_minus = -dy, +dy        # +Y-Backe: p sinkt bei Bewegung nach +Y; -Y-Backe: p steigt
        if self.left_is_plus_y:
            new_left, new_right = p_left + d_plus, p_right + d_minus
        else:
            new_left, new_right = p_left + d_minus, p_right + d_plus
        self.check(new_left, new_right, f'Verschiebung um {dy:+.3f} m')
        return new_left, new_right

    def object_from_jaws(self, p_left, p_right):
        """(Mitte y, lichte Breite) aus den Backenpositionen, auch ohne Objekt (offen: Mitte 0)."""
        p_plus, p_minus = (p_left, p_right) if self.left_is_plus_y else (p_right, p_left)
        face_plus = self.open_width / 2.0 - p_plus
        face_minus = -self.open_width / 2.0 + p_minus
        return (face_plus + face_minus) / 2.0, face_plus - face_minus

    def check(self, p_left, p_right, what=''):
        for name, p in (('links', p_left), ('rechts', p_right)):
            if not -1e-9 <= p <= self.travel + 1e-9:
                raise GripperError(f'{what}: Backe {name} müsste auf {p * 1000:.1f} mm '
                                   f'(erlaubt 0 … {self.travel * 1000:.1f} mm). Objekt zu breit oder zu weit außermittig')
