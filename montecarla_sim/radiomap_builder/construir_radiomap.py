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
import math
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

def leer_bag(ruta_bag, pose_topic='/odometry/filtered'):
    """
    Lee /wifi_scan y el tópico de pose indicado del bag.

    IMPORTANTE: se usa el timestamp de grabación del bag (wall clock), NO el
    header.stamp del mensaje. Motivo: wifi_simulator arranca sin use_sim_time,
    por lo que su header.stamp está en wall clock (~Unix epoch), mientras que
    odometry/filtered usa tiempo de simulación (~0–220 s). Sus header.stamps
    no son comparables. El timestamp del bag es wall clock para ambos tópicos
    y permite sincronización correcta.

    pose_topic puede ser:
      '/odometry/filtered'  — nav_msgs/msg/Odometry      (sim o survey sin SLAM)
      '/amcl_pose'          — geometry_msgs/msg/PoseWithCovarianceStamped (replay con AMCL)
    Ambos exponen la pose en msg.pose.pose.position.{x,y}.

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

        elif topic == pose_topic:
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odom_msgs.append((t_bag_ns, x, y))

    # Los bags sqlite3 suelen llegar en orden, pero ordenamos por si acaso
    odom_msgs.sort(key=lambda o: o[0])
    wifi_msgs.sort(key=lambda w: w[0])

    return wifi_msgs, odom_msgs


# ── Lectura del bag — modo Opción A (SLAM simultáneo) ─────────────────────────

def _quat_to_yaw(qx, qy, qz, qw):
    """Convierte cuaternión a yaw (rotación alrededor del eje Z)."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def _indice_mas_cercano(ts_sorted, t_ns):
    """Índice del elemento más cercano a t_ns en lista ordenada ts_sorted."""
    idx = bisect_left(ts_sorted, t_ns)
    if idx == 0:
        return 0
    if idx >= len(ts_sorted):
        return len(ts_sorted) - 1
    if abs(ts_sorted[idx - 1] - t_ns) <= abs(ts_sorted[idx] - t_ns):
        return idx - 1
    return idx


def leer_bag_tf(ruta_bag):
    """
    Lee /wifi_scan y /tf del bag (Opción A: SLAM + survey simultáneos).

    Extrae los transforms 'map→odom' (slam_toolbox) y 'odom→base_link' (EKF)
    de los mensajes /tf y los compone para obtener la pose del robot en el
    frame 'map' a la cadencia de la odometría (~30 Hz).

    El bag DEBE contener el transform map→odom en /tf, lo que solo ocurre
    si slam_toolbox estaba corriendo durante la grabación.

    Devuelve:
      wifi_msgs:  [(t_bag_ns, WifiScan)]   — ordenado por tiempo de grabación
      poses_map:  [(t_bag_ns, x_map, y_map)] — pose robot en map frame (~30 Hz)
    """
    lector = rosbag2_py.SequentialReader()
    lector.open(rosbag2_py.StorageOptions(uri=str(ruta_bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}

    wifi_msgs     = []
    map_odom_raw  = []   # [(t_ns_tf, tx, ty, yaw)]  — slam_toolbox → map→odom
    odom_base_raw = []   # [(t_bag_ns, tx, ty, yaw)] — EKF → odom→base_link

    while lector.has_next():
        topic, datos, t_bag_ns = lector.read_next()

        if topic == '/wifi_scan':
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            wifi_msgs.append((t_bag_ns, msg))

        elif topic in ('/tf', '/tf_static'):
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            for tf in msg.transforms:
                # Usar timestamp del header del TF (más preciso que t_bag_ns para TF)
                t_tf = (int(tf.header.stamp.sec) * 10**9 +
                        int(tf.header.stamp.nanosec))
                tx  = tf.transform.translation.x
                ty  = tf.transform.translation.y
                q   = tf.transform.rotation
                yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

                if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                    map_odom_raw.append((t_tf, tx, ty, yaw))
                elif tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                    # Para odom→base_link usamos t_bag_ns (wall clock del grabador)
                    # para mantener coherencia con t_bag_ns de wifi_scan
                    odom_base_raw.append((t_bag_ns, tx, ty, yaw))

    map_odom_raw.sort(key=lambda x: x[0])
    odom_base_raw.sort(key=lambda x: x[0])
    wifi_msgs.sort(key=lambda x: x[0])

    if not map_odom_raw:
        raise ValueError(
            'No hay transform map→odom en /tf del bag.\n'
            'Asegúrate de que slam_toolbox estaba corriendo durante la grabación.\n'
            'Verifica con: grep "map" <bag>/metadata.yaml'
        )
    if not odom_base_raw:
        raise ValueError('No hay transform odom→base_link en /tf del bag.')

    # Componer T_map_odom * T_odom_base en cada muestra de odom→base_link
    ts_mo = [x[0] for x in map_odom_raw]
    poses_map = []

    for t_bag_ns, tx_ob, ty_ob, yaw_ob in odom_base_raw:
        # Buscar map→odom más cercano en tiempo
        idx_mo = _indice_mas_cercano(ts_mo, t_bag_ns)
        _, tx_mo, ty_mo, yaw_mo = map_odom_raw[idx_mo]

        # Composición 2D: T_map_base = T_map_odom * T_odom_base
        cos_mo = math.cos(yaw_mo)
        sin_mo = math.sin(yaw_mo)
        x_map  = tx_mo + cos_mo * tx_ob - sin_mo * ty_ob
        y_map  = ty_mo + sin_mo * tx_ob + cos_mo * ty_ob

        poses_map.append((t_bag_ns, x_map, y_map))

    print(f'      {len(map_odom_raw)} transforms map→odom,  '
          f'{len(odom_base_raw)} odom→base_link → '
          f'{len(poses_map)} poses en map frame')

    return wifi_msgs, poses_map


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


def _pose_media_en_ventana(t_ns, dur_s, odom_msgs, timestamps_odom):
    """
    Calcula la pose media del robot durante la ventana [t_ns - dur_s, t_ns].

    El wifi_scan representa un promedio sobre scan_duration segundos. Asignar
    el RSSI a la pose media de esa ventana evita el 'smearing' en las puertas,
    donde la pose puntual puede estar en una habitación pero el promedio del
    scan incluye señal de la otra.
    """
    dur_ns = int(dur_s * 1e9)
    t_inicio = t_ns - dur_ns
    idx_ini = bisect_left(timestamps_odom, t_inicio)
    idx_fin = bisect_left(timestamps_odom, t_ns)
    ventana = odom_msgs[idx_ini:idx_fin]
    if not ventana:
        return _pose_mas_cercana(t_ns, odom_msgs, timestamps_odom)
    xs = [o[1] for o in ventana]
    ys = [o[2] for o in ventana]
    return float(np.mean(xs)), float(np.mean(ys))


# ── Construcción del radiomap ──────────────────────────────────────────────────

def construir_radiomaps(wifi_msgs, odom_msgs, info_mapa, scan_duration=1.0):
    """
    Construye {bssid: array float32 (alto × ancho)} con RSSI medio por celda.
    Celdas sin ninguna medida quedan como NaN.

    scan_duration: ventana temporal del wifi_scan (s). Se usa para calcular la
    pose media del robot durante ese período, evitando smearing en las puertas.
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
        x, y = _pose_media_en_ventana(t_ns, scan_duration, odom_msgs, timestamps_odom)

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

    # smoothing=1.0 → interpolante suavizado: no pasa exactamente por los puntos
    # pero amortigua el ruido de medida (σ=6dBm). Con smoothing=0.0 y datos
    # ruidosos el TPS crea oscilaciones que confunden al filtro.
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
    parser.add_argument('--bag',           required=True, help='Directorio del bag')
    parser.add_argument('--mapa',          required=True, help='Ruta al .yaml del mapa SLAM')
    parser.add_argument('--salida',        default='/maps/', help='Directorio de salida')
    parser.add_argument('--scan-duration', type=float, default=1.0,
                        help='Duración del scan WiFi en segundos (default 1.0, debe coincidir con aps.yaml)')
    parser.add_argument('--bssids', nargs='+', default=[],
                        help='Lista de BSSIDs a incluir (vacío = todos)')
    parser.add_argument('--pose-topic', default='/odometry/filtered',
                        help='Tópico de pose:\n'
                             '  /odometry/filtered  — sim o survey sin SLAM (frame odom)\n'
                             '  /amcl_pose          — replay con AMCL (frame map, Opción B)\n'
                             '  /tf                 — SLAM simultáneo (frame map, Opción A)')
    args = parser.parse_args()

    print(f'[1/4] Leyendo mapa: {args.mapa}')
    info_mapa = leer_meta_mapa(args.mapa)
    print(f'      {info_mapa["ancho"]}×{info_mapa["alto"]} celdas, '
          f'res={info_mapa["resolucion"]}m, '
          f'origen=({info_mapa["origen_x"]:.2f}, {info_mapa["origen_y"]:.2f})')

    print(f'[2/4] Leyendo bag: {args.bag}  (pose: {args.pose_topic})')
    try:
        if args.pose_topic == '/tf':
            wifi_msgs, odom_msgs = leer_bag_tf(args.bag)
        else:
            wifi_msgs, odom_msgs = leer_bag(args.bag, pose_topic=args.pose_topic)
            print(f'      {len(wifi_msgs)} wifi_scan,  {len(odom_msgs)} {args.pose_topic}')
    except ValueError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        sys.exit(1)

    if not wifi_msgs:
        print('ERROR: no hay mensajes /wifi_scan en el bag', file=sys.stderr)
        sys.exit(1)
    if not odom_msgs:
        print(f'ERROR: no hay poses en el bag (topic={args.pose_topic})', file=sys.stderr)
        sys.exit(1)

    print(f'[3/4] Construyendo radiomaps (medidas brutas, scan_duration={args.scan_duration}s)...')
    radiomaps_raw = construir_radiomaps(wifi_msgs, odom_msgs, info_mapa, args.scan_duration)
    if args.bssids:
        bssids_filtro = {b.upper() for b in args.bssids}
        radiomaps_raw = {b: v for b, v in radiomaps_raw.items() if b.upper() in bssids_filtro}
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
