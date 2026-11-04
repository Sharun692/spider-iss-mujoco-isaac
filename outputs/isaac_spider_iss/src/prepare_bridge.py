"""Reconstruct the MJCF articulation adaptation without modifying the archived asset."""
from pathlib import Path
import argparse
import xml.etree.ElementTree as E
ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=ROOT/'bridge_mjcf.rebuilt.xml')
args = parser.parse_args()
source = ROOT.parent/'canadarm-capture/spider_truss_iss/physics/assets/spider_modal_plant.xml'
tree = E.parse(source)
body = tree.find('.//body[@name="robot_body"]')
for joint in list(body.findall('joint')):
    body.remove(joint)
body.insert(3, E.Element('freejoint', name='base_free'))
for j in range(4):
    shoulder = body.find(f'body[@name="link_{j}_0"]')
    carrier = E.Element('body', name=f'yaw_carrier_{j}', pos=shoulder.attrib.pop('pos'))
    yaw = shoulder.find(f'joint[@name="yaw_{j}"]')
    shoulder.remove(yaw)
    carrier.append(yaw)
    E.SubElement(carrier, 'inertial', pos='0 0 0', mass='.0001', diaginertia='1e-6 1e-6 1e-6')
    index = list(body).index(shoulder)
    body.remove(shoulder)
    carrier.append(shoulder)
    body.insert(index, carrier)
tree.write(args.output)
print(args.output)
