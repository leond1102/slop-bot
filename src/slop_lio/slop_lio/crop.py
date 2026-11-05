"""Quader im Lidar-Frame, die aus den Scans geschnitten werden (Roboterkörper, Mast), ohne ROS.

Maße des Körpers und die Lidar-Position kommen aus dem URDF (slop_description), damit sie nur dort stehen.
Alle Quader sind achsenparallel im Lidar-Frame (x vorne, y links, z oben, Ursprung = Mitte des Lidars) und werden als
(xmin, xmax, ymin, ymax, zmin, zmax) angegeben. Eine Neigung des Lidars (wenige Grad) wird nicht berücksichtigt; sie ist
gegenüber den Zuschlägen klein.
"""
import xml.etree.ElementTree as ET


def _floats(text):
    return [float(v) for v in text.split()]


def body_box(body_center, body_size, lidar_origin, margin_side=0.10, margin_up=0.0):
    """Roboterkörper im Lidar-Frame.

    body_center / lidar_origin: Mitte des Körpers und Ursprung des Lidars im selben Frame (base_link),
    body_size: Länge, Breite, Höhe. margin_side: Zuschlag auf die halbe Breite (0,10 = 10 % links und rechts),
    margin_up: Zuschlag oben in m (Rauschen der Oberseite)."""
    cx, cy, cz = (c - o for c, o in zip(body_center, lidar_origin))
    sx, sy, sz = body_size
    half_y = sy / 2.0 * (1.0 + margin_side)
    return (cx - sx / 2.0, cx + sx / 2.0, cy - half_y, cy + half_y, cz - sz / 2.0, cz + sz / 2.0 + margin_up)


def rear_box(depth, half_width, height):
    """Quader hinter dem Lidar: beginnt in dessen Mitte, `depth` nach hinten, `half_width` nach links und rechts,
    `height` nach oben."""
    return (-depth, 0.0, -half_width, half_width, 0.0, height)


def boxes_from_urdf(urdf_xml, body_margin_side=0.10, body_margin_up=0.0, rear=(0.40, 15.0, 1.0)):
    """Liste der Quader (flach, je 6 Werte) aus dem URDF-Text: Körper (Link `chassis`, Gelenke `chassis_joint` und
    `livox_joint`) und der Quader hinter dem Lidar. rear = (Tiefe, halbe Breite, Höhe)."""
    root = ET.fromstring(urdf_xml)
    joints = {j.get('name'): j for j in root.findall('joint')}
    body_center = _floats(joints['chassis_joint'].find('origin').get('xyz'))
    lidar_origin = _floats(joints['livox_joint'].find('origin').get('xyz'))
    box = root.find("link[@name='chassis']/visual/geometry/box")
    body_size = _floats(box.get('size'))
    boxes = [body_box(body_center, body_size, lidar_origin, body_margin_side, body_margin_up), rear_box(*rear)]
    return [v for b in boxes for v in b]


def center_and_size(box):
    """(xmin, xmax, ymin, ymax, zmin, zmax) → (Mitte (x, y, z), Größe (sx, sy, sz))."""
    xmin, xmax, ymin, ymax, zmin, zmax = box
    return ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0), (xmax - xmin, ymax - ymin, zmax - zmin)
