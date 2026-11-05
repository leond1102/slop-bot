"""Quader zum Ausschneiden aus den Scans, ohne ROS."""
import pytest

from slop_lio.crop import body_box, boxes_from_urdf, center_and_size, rear_box

URDF = """<robot name="t">
  <link name="chassis"><visual><geometry><box size="0.34 0.24 0.25"/></geometry></visual></link>
  <joint name="chassis_joint" type="fixed"><parent link="base_link"/><child link="chassis"/>
    <origin xyz="-0.17 0 0.125" rpy="0 0 0"/></joint>
  <joint name="livox_joint" type="fixed"><parent link="base_link"/><child link="livox_frame"/>
    <origin xyz="0.03 0.0 0.28" rpy="0 0 0"/></joint>
</robot>"""


def test_body_box_in_lidar_frame():
    b = body_box((-0.17, 0.0, 0.125), (0.34, 0.24, 0.25), (0.03, 0.0, 0.28), margin_side=0.10)
    # Körper x: -0.34…0 im base_link → -0.37…-0.03 im Lidar-Frame; endet 3 cm unter dem Lidar (Oberkante bei z = -0.03)
    assert b == pytest.approx((-0.37, -0.03, -0.132, 0.132, -0.28, -0.03))


def test_body_margin_up():
    assert body_box((0, 0, 0), (1, 1, 1), (0, 0, 0), margin_up=0.02)[5] == pytest.approx(0.52)


def test_rear_box_starts_at_lidar_center():
    assert rear_box(0.40, 15.0, 1.0) == (-0.40, 0.0, -15.0, 15.0, 0.0, 1.0)


def test_boxes_from_urdf():
    boxes = boxes_from_urdf(URDF, rear=(0.40, 15.0, 1.0))
    assert len(boxes) == 12
    assert boxes[:6] == pytest.approx([-0.37, -0.03, -0.132, 0.132, -0.28, -0.03])
    assert boxes[6:] == pytest.approx([-0.40, 0.0, -15.0, 15.0, 0.0, 1.0])


def test_center_and_size():
    center, size = center_and_size((-0.4, 0.0, -15.0, 15.0, 0.0, 1.0))
    assert center == pytest.approx((-0.2, 0.0, 0.5)) and size == pytest.approx((0.4, 30.0, 1.0))


def test_urdf_frames_do_not_collide_with_fast_lio():
    """FAST-LIO sendet die TF camera_init → body (feste Namen im Code). Ein URDF-Link `body` bekam dadurch zwei Eltern und
    sprang in RViz zwischen URDF und IMU-Pose hin und her (Körper, Mast und Fahne "drifteten" vom Rest weg)."""
    import os
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        urdf = os.path.join(get_package_share_directory('slop_description'), 'urdf', 'slop_bot.urdf.xacro')
    except PackageNotFoundError:
        pytest.skip('slop_description nicht installiert')
    links = {link.get('name') for link in ET.fromstring(xacro.process_file(urdf).toxml()).findall('link')}
    assert not links & {'body', 'camera_init', 'map', 'odom'}, f'Namenskonflikt: {links & {"body", "camera_init", "map", "odom"}}'
