"""Stonefish's textured OBJ loader requires explicit v/vt/vn indices."""
from pathlib import Path
import xml.etree.ElementTree as ET


def test_real_rov_textured_mesh_has_complete_face_indices():
    package = Path(__file__).resolve().parents[1]
    scenario = ET.parse(package / 'scenarios/real_rov.scn')
    mesh = package / scenario.find('robot/base_link/visual/mesh').get('filename')
    counts = {'v': 0, 'vt': 0, 'vn': 0}
    faces = []
    for line in mesh.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] in counts:
            counts[fields[0]] += 1
        elif fields and fields[0] == 'f':
            faces.append(fields[1:])
    assert 50000 <= len(faces) <= 70000
    for face in faces:
        assert len(face) == 3
        for corner in face:
            indices = corner.split('/')
            assert len(indices) == 3 and all(indices)
            for index, kind in zip(indices, ('v', 'vt', 'vn')):
                assert 1 <= int(index) <= counts[kind]
