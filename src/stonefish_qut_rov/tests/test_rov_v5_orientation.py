"""V5 meshes and actuator stations share a forward/right/down vehicle frame."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def scenario():
    return ET.parse(PACKAGE / "scenarios/rov_v5.scn").find("robot")


def vertices(element):
    mesh = element.find("mesh")
    scale = float(mesh.get("scale"))
    return [tuple(float(x) * scale for x in line.split()[1:4])
            for line in (PACKAGE / mesh.get("filename")).read_text().splitlines()
            if line.startswith("v ")]


def test_physical_frame_and_spawn_do_not_rotate_the_actuators():
    robot = scenario()
    origin = robot.find("base_link/physical/origin")
    assert tuple(map(float, origin.get("rpy").split())) == (0, 0, 0)
    assert tuple(map(float, origin.get("xyz").split())) == (0, 0, 0)
    spawn = robot.find("world_transform")
    assert spawn.get("xyz") == "$(arg position)"
    assert spawn.get("rpy") == "$(arg orientation)"


def test_visual_and_physical_extents_align_in_ned():
    base = scenario().find("base_link")
    origin = base.find("visual/origin")
    roll, pitch, yaw = map(float, origin.get("rpy").split())
    assert roll == yaw == 0
    assert math.isclose(pitch, math.pi, abs_tol=1e-12)
    assert tuple(map(float, origin.get("xyz").split())) == (0, 0, 0)
    visual = [(-x, y, -z) for x, y, z in vertices(base.find("visual"))]
    physical = vertices(base.find("physical"))
    for axis in range(3):
        for extreme in (min, max):
            # Simplified fittings/ducts may differ by up to 1 cm from CAD.
            assert abs(extreme(p[axis] for p in visual)
                       - extreme(p[axis] for p in physical)) < 0.01
    # Nose points forward; the top of the foam is above the origin (negative Z).
    for landmark in [(0.09652, -0.02049, -0.04104),
                     (0.0995, -0.1200, -0.1057)]:
        assert min(math.dist(p, landmark) for p in physical) < 0.01


def test_propellers_are_inside_the_corresponding_ducts():
    robot = scenario()
    physical = vertices(robot.find("base_link/physical"))
    for actuator in robot.findall("actuator"):
        center = tuple(map(float, actuator.find("origin").get("xyz").split()))
        # Vertical ducts revolve around Z; horizontal ducts around X.
        axis = 2 if actuator.get("name") in ("TL", "TR") else 0
        radial_axes = [i for i in range(3) if i != axis]
        for radial_axis in radial_axes:
            for sign in (-1, 1):
                point = list(center)
                point[radial_axis] += sign * 0.04
                # Rim is about 1.5 cm axially from the propeller station.
                assert min(math.dist(p, point) for p in physical) < 0.021
