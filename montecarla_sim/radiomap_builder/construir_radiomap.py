#!/usr/bin/env python3
"""
construir_radiomap.py — construye radiomap.npy a partir de un bag Montecarla.

Para cada AP visible en el bag genera:
  radiomap_<bssid>.npy   — array float32 (alto × ancho), NaN = sin medida
  radiomap_meta.yaml     — origin, resolution, width, height (igual que el mapa SLAM)

El radiomap_meta.yaml es el contrato entre este script y montecarla_amcl:
  - montecarla_amcl carga ambos ficheros al arrancar
  - Para cada partícula en (x, y): col = int((x-ox)/res), fila = int((y-oy)/res)
  - Si radiomap[fila,col] es NaN → AP ignorado (ni penalización ni recompensa)
  - Si tiene valor → log_w += -(rssi_obs - rssi_pred)² / (2·σ²)

Uso (dentro del perfil radiomap de compose.sim.yaml):
  BAG_NAME=casa_20260609_152706 docker compose -f compose.sim.yaml \\
    --profile radiomap run --rm radiomap-builder

O directamente con docker run:
  python3 construir_radiomap.py \\
    --bag  /bags/casa_20260609_152706 \\
    --mapa /maps/casa_simple.yaml    \\
    --salida /maps/
"""

import argparse
import sys
import yaml
import numpy as np
from pathlib import Path
from bisect import bisect_left

from scipy.interpolate import RBFInterpolator
from scipy.spatial import KDTree

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ── Lectura del mapa SLAM ──────────────────────────────────────────────────────

def leer_meta_mapa(ruta_yaml):
    """
    Lee metadata del mapa SLAM desde el .yaml de nav2_map_server.
    Devuelve dict con: resolucion, origen_x, origen_y, ancho, alto.
    """
    ruta_yaml = Path(ruta_yaml)
    with open(ruta_yaml) as f:
        meta = yaml.safe_load(f)

    ruta_pgm = ruta_yaml.parent / meta['image']
    ancho, alto = _leer_dimensiones_pgm(ruta_pgm)

    return {
        'resolucion': float(meta['resolution']),
        'origen_x':   float(meta['origin'][0]),
        'origen_y':   float(meta['origin'][1]),
        'ancho':       ancho,
        'alto':        alto,
    }


def _leer_dimensiones_pgm(ruta):
    """Lee ancho y alto de un PGM (P5 binario o P2 ASCII) sin dependencias externas."""
    with open(ruta, 'rb') as f:
        f.readline()  # P5 / P2
        # Saltar líneas de comentario
        while True:
            linea = f.readline()
            if not linea.startswith(b'#'):
                break
        partes = linea.split()
        ancho = int(partes[0])
        alto  = int(partes[1]) if len(partes) > 1 else int(f.readline().strip())
    return ancho, alto


# ── Lectura del bag ────────────────────────────────────────────────────────────

def leer_bag(ruta_bag):
    """
    Lee /wifi_scan y /odometry/filtered del bag.

    IMPORTANTE: se usa el timestamp de grabación del bag (wall clock), NO el
    header.stamp del mensaje. Motivo: wifi_simulator arranca sin use_sim_time,
    por lo que su header.stamp está en wall clock (~Unix epoch), mientras que
    odometry/filtered usa tiempo de simulación (~0–220 s). Sus header.stamps
    no son comparables. El timestamp del bag es wall clock para ambos tópicos
    y permite sincronización correcta.

    Devuelve:
      wifi_msgs: [(t_bag_ns, WifiScan)]  — ordenado por tiempo de grabación
      odom_msgs: [(t_bag_ns, x, y)]      — ordenado por tiempo de grabación
    """
    lector = rosbag2_py.SequentialReader()
    opciones_storage = rosbag2_py.StorageOptions(uri=str(ruta_bag), storage_id='sqlite3')
    opciones_conv    = rosbag2_py.ConverterOptions('', '')
    lector.open(opciones_storage, opciones_conv)

    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}

    wifi_msgs = []
    odom_msgs = []

    while lector.has_next():
        topic, datos, t_bag_ns = lector.read_next()  # t_bag_ns = wall clock del grabador

        if topic == '/wifi_scan':
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            wifi_msgs.append((t_bag_ns, msg))

        elif topic == '/odometry/filtered':
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odom_msgs.append((t_bag_ns, x, y))

    # Los bags sqlite3 suelen llegar en orden, pero ordenamos por si acaso
    odom_msgs.sort(key=lambda o: o[0])
    wifi_msgs.sort(key=lambda w: w[0])

    return wifi_msgs, odom_msgs


# ── Sincronización temporal ────────────────────────────────────────────────────

def _pose_mas_cercana(t_ns, odom_msgs, timestamps_odom):
    """Búsqueda binaria de la pose con timestamp más cercano a t_ns."""
    idx = bisect_left(timestamps_odom, t_ns)
    if idx == 0:
        return odom_msgs[0][1], odom_msgs[0][2]
    if idx >= len(odom_msgs):
        return odom_msgs[-1][1], odom_msgs[-1][2]
    antes   = odom_msgs[idx - 1]
    despues = odom_msgs[idx]
    if abs(antes[0] - t_ns) <= abs(despues[0] - t_ns):
        return antes[1], antes[2]
    return despues[1], despues[2]


# ── Construcción del radiomap ──────────────────────────────────────────────────

def construir_radiomaps(wifi_msgs, odom_msgs, info_mapa):
    """
    Construye {bssid: array float32 (alto × ancho)} con RSSI medio por celda.
    Celdas sin ninguna medida quedan como NaN.
    """
    resolucion = info_mapa['resolucion']
    ox         = info_mapa['origen_x']
    oy         = info_mapa['origen_y']
    ancho      = info_mapa['ancho']
    alto       = info_mapa['alto']

    timestamps_odom = [o[0] for o in odom_msgs]

    # Acumuladores: {bssid: (array_suma, array_cuenta)}
    acum = {}

    omitidas = 0
    for (t_ns, wifi_msg) in wifi_msgs:
        x, y = _pose_mas_cercana(t_ns, odom_msgs, timestamps_odom)

        col  = int((x - ox) / resolucion)
        fila = int((y - oy) / resolucion)

        if not (0 <= col < ancho and 0 <= fila < alto):
            omitidas += 1
            continue

        for medicion in wifi_msg.measurements:
            bssid = medicion.bssid
            if bssid not in acum:
                acum[bssid] = (
                    np.zeros((alto, ancho), dtype=np.float64),
                    np.zeros((alto, ancho), dtype=np.int32),
                )
            acum[bssid][0][fila, col] += medicion.rssi
            acum[bssid][1][fila, col] += 1

    if omitidas:
        print(f'  Advertencia: {omitidas} scans fuera del mapa omitidos')

    # Calcular medias
    radiomaps = {}
    for bssid, (suma, cuenta) in acum.items():
        rm = np.full((alto, ancho), np.nan, dtype=np.float32)
        mascara = cuenta > 0
        rm[mascara] = (suma[mascara] / cuenta[mascara]).astype(np.float32)
        radiomaps[bssid] = rm

    return radiomaps


# ── Interpolación RBF ─────────────────────────────────────────────────────────

def interpolar_radiomap(rm_raw, info_mapa, nan_threshold=1.5):
    """
    Rellena el radiomap disperso con thin-plate-spline RBF.
    Celdas a más de nan_threshold metros del punto medido más cercano → NaN.

    Con 67 puntos de medida en una casa de 70m², la distancia media entre
    medidas es ~1m. Con nan_threshold=1.5m el mapa queda prácticamente completo.
    """
    res   = info_mapa['resolucion']
    ox    = info_mapa['origen_x']
    oy    = info_mapa['origen_y']
    ancho = info_mapa['ancho']
    alto  = info_mapa['alto']

    filas_med, cols_med = np.where(~np.isnan(rm_raw))
    if len(filas_med) < 4:
        print('  RBF: menos de 4 medidas, se guarda el mapa sin interpolar')
        return rm_raw

    # Coordenadas métricas de los puntos medidos
    x_med = cols_med * res + ox + res / 2
    y_med = filas_med * res + oy + res / 2
    pts   = np.column_stack([x_med, y_med])
    vals  = rm_raw[filas_med, cols_med].astype(np.float64)

    # Grid de consulta = todas las celdas del mapa
    cols_q, filas_q = np.meshgrid(np.arange(ancho), np.arange(alto))
    x_q   = cols_q.ravel() * res + ox + res / 2
    y_q   = filas_q.ravel() * res + oy + res / 2
    pts_q = np.column_stack([x_q, y_q])

    rbf         = RBFInterpolator(pts, vals, kernel='thin_plate_spline', smoothing=1.0)
    vals_interp = rbf(pts_q).reshape(alto, ancho).astype(np.float32)
    vals_interp = np.clip(vals_interp, -100.0, -30.0)

    # Máscara NaN para celdas lejos de cualquier medida
    dist, _ = KDTree(pts).query(pts_q)
    vals_interp[dist.reshape(alto, ancho) > nan_threshold] = np.nan

    n_validas = int(np.sum(~np.isnan(vals_interp)))
    print(f'  RBF: {len(filas_med)} medidas → {n_validas} celdas con valor '
          f'(umbral_nan={nan_threshold}m)')
    return vals_interp


# ── Guardado ───────────────────────────────────────────────────────────────────

def guardar(radiomaps, info_mapa, ruta_salida):
    """Guarda radiomap_<bssid>.npy y radiomap_meta.yaml en ruta_salida."""
    ruta_salida = Path(ruta_salida)
    ruta_salida.mkdir(parents=True, exist_ok=True)

    for bssid, rm in radiomaps.items():
        # ':' no permitido en nombres de fichero en algunos sistemas
        nombre = 'radiomap_' + bssid.replace(':', '_') + '.npy'
        np.save(ruta_salida / nombre, rm)
        n_celdas = int(np.sum(~np.isnan(rm)))
        rssi_min = float(np.nanmin(rm))
        rssi_max = float(np.nanmax(rm))
        print(f'  {bssid}: {n_celdas} celdas  RSSI∈[{rssi_min:.1f}, {rssi_max:.1f}] dBm  → {nombre}')

    meta = {
        'resolution': info_mapa['resolucion'],
        'origin':     [info_mapa['origen_x'], info_mapa['origen_y'], 0.0],
        'width':      info_mapa['ancho'],
        'height':     info_mapa['alto'],
        'bssids':     list(radiomaps.keys()),
    }
    with open(ruta_salida / 'radiomap_meta.yaml', 'w') as f:
        yaml.dump(meta, f, default_flow_style=False, allow_unicode=True)

    print(f'  radiomap_meta.yaml guardado  ({len(radiomaps)} APs)')


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Construye radiomap.npy desde un bag Montecarla'
    )
    parser.add_argument('--bag',    required=True, help='Directorio del bag')
    parser.add_argument('--mapa',   required=True, help='Ruta al .yaml del mapa SLAM')
    parser.add_argument('--salida', default='/maps/', help='Directorio de salida')
    args = parser.parse_args()

    print(f'[1/4] Leyendo mapa: {args.mapa}')
    info_mapa = leer_meta_mapa(args.mapa)
    print(f'      {info_mapa["ancho"]}×{info_mapa["alto"]} celdas, '
          f'res={info_mapa["resolucion"]}m, '
          f'origen=({info_mapa["origen_x"]:.2f}, {info_mapa["origen_y"]:.2f})')

    print(f'[2/4] Leyendo bag: {args.bag}')
    wifi_msgs, odom_msgs = leer_bag(args.bag)
    print(f'      {len(wifi_msgs)} wifi_scan,  {len(odom_msgs)} odometry/filtered')

    if not wifi_msgs:
        print('ERROR: no hay mensajes /wifi_scan en el bag', file=sys.stderr)
        sys.exit(1)
    if not odom_msgs:
        print('ERROR: no hay mensajes /odometry/filtered en el bag', file=sys.stderr)
        sys.exit(1)

    print('[3/4] Construyendo radiomaps (medidas brutas)...')
    radiomaps_raw = construir_radiomaps(wifi_msgs, odom_msgs, info_mapa)
    print(f'      {len(radiomaps_raw)} APs encontrados: {list(radiomaps_raw.keys())}')

    print('[4/4] Interpolando con RBF thin-plate-spline...')
    radiomaps = {}
    for bssid, rm_raw in radiomaps_raw.items():
        print(f'  {bssid}:')
        radiomaps[bssid] = interpolar_radiomap(rm_raw, info_mapa, nan_threshold=1.5)

    print(f'[5/5] Guardando en {args.salida}:')
    guardar(radiomaps, info_mapa, args.salida)

    print('¡Hecho!')


if __name__ == '__main__':
    main()
