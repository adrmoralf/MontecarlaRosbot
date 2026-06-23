#!/usr/bin/env python3
"""
ate_real.py — calcula ATE (Absolute Trajectory Error) para el experimento
real usando checkpoints físicos como ground truth.

Modos de uso:
  1) Desde CSV generados por registrar_checkpoints.py (recomendado):
       python3 montecarla_real/ate_real.py \\
         --csv-a ~/real_bags/checkpoints_A.csv \\
         --csv-b ~/real_bags/checkpoints_B.csv

  2) Desde bags de replay + checkpoints.yaml (modo automático):
       python3 montecarla_real/ate_real.py \\
         --bag-a ~/real_bags/real_replay_base_XXXX \\
         --bag-b ~/real_bags/real_replay_true_XXXX \\
         --checkpoints docker-compose/config/real/checkpoints.yaml \\
         --ventana 10.0

En el modo 2, para cada checkpoint se busca la media de /amcl_pose
en la ventana de tiempo en que el robot estuvo en ese punto.
La ventana se estima como el intervalo de tiempo en que la pose estimada
está cerca del checkpoint (distancia < umbral).
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


# ── Modo 1: desde CSV ─────────────────────────────────────────────────────────

def leer_csv(ruta: str) -> list[dict]:
    with open(ruta, newline='') as f:
        return list(csv.DictReader(f))


def calcular_ate_csv(registros: list[dict]) -> dict:
    errores = [float(r['error_m']) for r in registros]
    if not errores:
        return {}
    arr = np.array(errores)
    rmse = float(np.sqrt(np.mean(arr ** 2)))
    zonas: dict[str, list[float]] = {}
    for r in registros:
        zonas.setdefault(r['zona'], []).append(float(r['error_m']))
    return {
        'rmse':  rmse,
        'n':     len(errores),
        'p50':   float(np.percentile(arr, 50)),
        'p95':   float(np.percentile(arr, 95)),
        'menos_1m': int(np.sum(arr < 1.0)),
        'menos_05m': int(np.sum(arr < 0.5)),
        'max':   float(np.max(arr)),
        'zonas': {
            z: {
                'rmse': float(np.sqrt(np.mean(np.array(es) ** 2))),
                'n':    len(es),
            }
            for z, es in zonas.items()
        },
    }


# ── Modo 2: desde bag ─────────────────────────────────────────────────────────

def leer_bag_amcl(ruta: str) -> list[tuple[int, float, float]]:
    """Lee /amcl_pose del bag → lista de (timestamp_ns, x, y)."""
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(ruta), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    poses = []
    while reader.has_next():
        topic, datos, _ = reader.read_next()
        if topic == '/amcl_pose' and topic in tipos:
            tipo = get_message(tipos[topic])
            msg = deserialize_message(datos, tipo)
            t_ns = int(msg.header.stamp.sec) * 10 ** 9 + int(msg.header.stamp.nanosec)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            poses.append((t_ns, x, y))
    return poses


def calcular_ate_bag(
    poses: list[tuple[int, float, float]],
    checkpoints: list[dict],
    umbral_m: float = 0.8,
    ventana_s: float = 8.0,
) -> list[dict]:
    """
    Para cada checkpoint, busca el intervalo de tiempo en que la pose estimada
    está dentro de `umbral_m` metros de las coordenadas del checkpoint y
    calcula el error medio en ese intervalo.

    Esta heurística funciona solo si en el experimento real el robot se detiene
    en cada checkpoint durante al menos `ventana_s` segundos.
    """
    registros = []
    arr = np.array([(t, x, y) for t, x, y in poses])

    for cp in checkpoints:
        x_real = float(cp['x'])
        y_real = float(cp['y'])

        distancias = np.sqrt((arr[:, 1] - x_real) ** 2 + (arr[:, 2] - y_real) ** 2)
        cerca = distancias < umbral_m

        if not np.any(cerca):
            print(f'  ⚠ {cp["id"]}: robot nunca estuvo cerca del checkpoint (umbral={umbral_m}m)')
            continue

        # Tomar la ventana más larga en que el robot estuvo cerca
        tiempos_cerca = arr[cerca, 0]
        x_cerca = arr[cerca, 1]
        y_cerca = arr[cerca, 2]

        x_media = float(np.mean(x_cerca))
        y_media = float(np.mean(y_cerca))
        error = math.sqrt((x_media - x_real) ** 2 + (y_media - y_real) ** 2)

        registros.append({
            'id':      cp['id'],
            'zona':    cp.get('zona', '?'),
            'x_real':  x_real,
            'y_real':  y_real,
            'x_amcl':  round(x_media, 4),
            'y_amcl':  round(y_media, 4),
            'error_m': round(error, 4),
            'n_poses': int(np.sum(cerca)),
        })

    return registros


# ── Impresión de resultados ───────────────────────────────────────────────────

def imprimir_tabla(etiqueta: str, stats: dict) -> None:
    print(f'\n{"─"*55}')
    print(f'  {etiqueta}')
    print(f'{"─"*55}')
    if not stats:
        print('  Sin datos.')
        return
    n = stats["n"]
    print(f'  ATE RMSE:     {stats["rmse"]:.3f} m')
    print(f'  Mediana:      {stats["p50"]:.3f} m')
    print(f'  P95:          {stats["p95"]:.3f} m')
    print(f'  Máximo:       {stats["max"]:.3f} m')
    print(f'  < 0.5m:  {stats["menos_05m"]:3d}/{n} ({100*stats["menos_05m"]/n:.0f}%)')
    print(f'  < 1.0m:  {stats["menos_1m"]:3d}/{n} ({100*stats["menos_1m"]/n:.0f}%)')
    print(f'  ── Por zona ──')
    for zona, z in sorted(stats['zonas'].items()):
        print(f'    {zona:12s}: RMSE={z["rmse"]:.3f}m  n={z["n"]}')


def imprimir_comparacion(stats_a: dict, stats_b: dict) -> None:
    if not stats_a or not stats_b:
        return
    mejora = (stats_a['rmse'] - stats_b['rmse']) / stats_a['rmse'] * 100
    print(f'\n{"═"*55}')
    print(f'  COMPARACIÓN A vs B')
    print(f'{"═"*55}')
    print(f'  A — AMCL puro:   {stats_a["rmse"]:.3f} m')
    print(f'  B — Montecarla:  {stats_b["rmse"]:.3f} m')
    simbolo = '↓' if mejora > 0 else '↑'
    print(f'  Mejora RMSE: {abs(mejora):.1f}% {simbolo}')
    print(f'  (sim referencia: A=2.772m  B=1.271m  +54.2%)')


def guardar_csv(registros: list[dict], ruta: str) -> None:
    if not registros:
        return
    Path(ruta).parent.mkdir(parents=True, exist_ok=True)
    with open(ruta, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=registros[0].keys())
        writer.writeheader()
        writer.writerows(registros)
    print(f'  Guardado: {ruta}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Calcula ATE real con checkpoints físicos (A vs B)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Modo 1: CSV
    parser.add_argument('--csv-a', help='CSV experimento A (de registrar_checkpoints.py)')
    parser.add_argument('--csv-b', help='CSV experimento B (de registrar_checkpoints.py)')
    # Modo 2: bags
    parser.add_argument('--bag-a', help='Bag replay experimento A (/amcl_pose)')
    parser.add_argument('--bag-b', help='Bag replay experimento B (/amcl_pose)')
    parser.add_argument('--checkpoints', help='YAML de checkpoints (modo bag)')
    parser.add_argument('--umbral', type=float, default=0.8,
                        help='Radio (m) para considerar que el robot está en el checkpoint (default: 0.8)')
    # Solo A (experimento único)
    parser.add_argument('--csv', help='CSV único sin comparación A/B')

    args = parser.parse_args()

    # ── Modo CSV único ────────────────────────────────────────────────────────
    if args.csv:
        registros = leer_csv(args.csv)
        stats = calcular_ate_csv(registros)
        imprimir_tabla('Resultado único', stats)
        return

    # ── Modo CSV A/B ──────────────────────────────────────────────────────────
    if args.csv_a or args.csv_b:
        stats_a, stats_b = {}, {}
        if args.csv_a:
            registros_a = leer_csv(args.csv_a)
            stats_a = calcular_ate_csv(registros_a)
            imprimir_tabla('A — AMCL puro', stats_a)
        if args.csv_b:
            registros_b = leer_csv(args.csv_b)
            stats_b = calcular_ate_csv(registros_b)
            imprimir_tabla('B — Montecarla (AMCL+WiFi)', stats_b)
        if args.csv_a and args.csv_b:
            imprimir_comparacion(stats_a, stats_b)
        return

    # ── Modo bag A/B ──────────────────────────────────────────────────────────
    if args.bag_a or args.bag_b:
        if not args.checkpoints:
            print('ERROR: --checkpoints es requerido en modo bag', file=sys.stderr)
            sys.exit(1)

        with open(args.checkpoints) as f:
            data = yaml.safe_load(f)
        checkpoints = data['checkpoints']

        import rclpy
        rclpy.init()

        stats_a, stats_b = {}, {}

        if args.bag_a:
            print(f'Leyendo bag A: {args.bag_a}')
            poses_a = leer_bag_amcl(args.bag_a)
            print(f'  {len(poses_a)} poses /amcl_pose')
            registros_a = calcular_ate_bag(poses_a, checkpoints, args.umbral)
            stats_a = calcular_ate_csv(registros_a)
            imprimir_tabla('A — AMCL puro', stats_a)
            csv_a_ruta = str(Path(args.bag_a).parent / 'ate_a_auto.csv')
            guardar_csv(registros_a, csv_a_ruta)

        if args.bag_b:
            print(f'Leyendo bag B: {args.bag_b}')
            poses_b = leer_bag_amcl(args.bag_b)
            print(f'  {len(poses_b)} poses /amcl_pose')
            registros_b = calcular_ate_bag(poses_b, checkpoints, args.umbral)
            stats_b = calcular_ate_csv(registros_b)
            imprimir_tabla('B — Montecarla (AMCL+WiFi)', stats_b)
            csv_b_ruta = str(Path(args.bag_b).parent / 'ate_b_auto.csv')
            guardar_csv(registros_b, csv_b_ruta)

        if args.bag_a and args.bag_b:
            imprimir_comparacion(stats_a, stats_b)

        rclpy.shutdown()
        return

    parser.print_help()
    sys.exit(1)


if __name__ == '__main__':
    main()
