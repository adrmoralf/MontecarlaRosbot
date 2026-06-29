#!/usr/bin/env python3
"""
analizar_aps_bag.py — lista todos los APs detectados en un bag ordenados por rango RSSI.

Uso:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 montecarla_sim/radiomap_builder/analizar_aps_bag.py ~/real_bags/opcionA_20260628_140433
"""
import sys
from collections import defaultdict
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

if len(sys.argv) < 2:
    print(f'Uso: {sys.argv[0]} <ruta_bag>', file=sys.stderr)
    sys.exit(1)

ruta = sys.argv[1]
lector = rosbag2_py.SequentialReader()
lector.open(rosbag2_py.StorageOptions(uri=ruta, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}

if '/wifi_scan' not in tipos:
    print('ERROR: no hay /wifi_scan en el bag', file=sys.stderr)
    sys.exit(1)

aps = defaultdict(list)
n_scans = 0
while lector.has_next():
    topic, datos, _ = lector.read_next()
    if topic == '/wifi_scan':
        n_scans += 1
        msg = deserialize_message(datos, get_message(tipos[topic]))
        for m in msg.measurements:
            aps[m.bssid].append(m.rssi)

print(f'\n{n_scans} wifi_scan mensajes, {len(aps)} APs únicos\n')
print(f'{"BSSID":<20} {"N":>5} {"Min":>8} {"Max":>8} {"Rango":>7} {"Media":>8}')
print('─' * 62)
for bssid, vals in sorted(aps.items(), key=lambda x: -(max(x[1]) - min(x[1]))):
    v = np.array(vals)
    rango = float(v.max() - v.min())
    print(f'{bssid:<20} {len(v):>5} {v.min():>8.1f} {v.max():>8.1f} {rango:>7.1f} {v.mean():>8.1f}')
