#!/usr/bin/env python3
"""
ate_calc.py — Absolute Trajectory Error entre estimación AMCL y ground truth.

Uso (con ROS 2 Humble nativo en el PC):
  source /opt/ros/humble/setup.bash
  python3 ate_calc.py \\
    --gt   ~/sim_bags/casa_20260609_152706 \\
    --base ~/sim_bags/replay_false_20260610_171324 \\
    --wifi ~/sim_bags/replay_true_20260610_172231
"""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _zona(x_gt: float) -> str:
    if x_gt > 1.5:
        return 'salon'
    if x_gt < -0.5:
        return 'hab'
    return 'pasillo'


def _tiempo_convergencia(parejas, umbral: float = 1.0) -> str:
    """Devuelve el tiempo sim en que el error cae por primera vez por debajo de umbral."""
    for t, _xe, _ye, _xg, _yg, err in parejas:
        if err < umbral:
            return f't={t/1e9:.1f}s'
    return 'nunca'


# ── Lector de bag ─────────────────────────────────────────────────────────────

def leer_poses(ruta_bag, topic):
    """
    Devuelve lista de (stamp_ns, x, y) para un topic de tipo PoseWithCovarianceStamped.
    stamp_ns usa header.stamp (tiempo de simulación), igual para GT y estimado.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(ruta_bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in tipos:
        print(f'  [WARN] {topic} no encontrado en {ruta_bag}')
        return []

    tipo_msg = get_message(tipos[topic])
    poses = []
    while reader.has_next():
        t, datos, _ = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(datos, tipo_msg)
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        poses.append((stamp_ns, x, y))
    return poses


# ── Cálculo ATE ───────────────────────────────────────────────────────────────

def ate_rmse(gt, estimado, ventana_ns=2_000_000_000):
    """
    Calcula ATE RMSE emparejando cada pose estimada con su GT más cercana en tiempo.
    Descarta parejas cuya diferencia temporal supera ventana_ns (por defecto 2 s).
    """
    gt_arr = np.array([(t, x, y) for t, x, y in gt])  # (N, 3)
    errores = []
    parejas = []

    for t_est, x_est, y_est in estimado:
        diffs = np.abs(gt_arr[:, 0] - t_est)
        idx = int(np.argmin(diffs))
        if diffs[idx] > ventana_ns:
            continue
        x_gt, y_gt = gt_arr[idx, 1], gt_arr[idx, 2]
        error = float(np.sqrt((x_est - x_gt) ** 2 + (y_est - y_gt) ** 2))
        errores.append(error)
        parejas.append((t_est, x_est, y_est, x_gt, y_gt, error))

    if not errores:
        return None, []
    return float(np.sqrt(np.mean(np.array(errores) ** 2))), parejas


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Calcula ATE RMSE comparando dos experimentos AMCL contra ground truth'
    )
    parser.add_argument('--gt',   required=True, help='Bag original (tiene /odometry/filtered)')
    parser.add_argument('--base', required=True, help='Replay sin WiFi (tiene /amcl_pose)')
    parser.add_argument('--wifi', required=True, help='Replay con WiFi (tiene /amcl_pose)')
    args = parser.parse_args()

    print('[1/4] Leyendo ground truth (/odometry/filtered)...')
    gt = leer_poses(args.gt, '/odometry/filtered')
    print(f'       {len(gt)} poses GT')
    if not gt:
        print('ERROR: no hay ground truth, comprueba la ruta del bag original')
        return

    # ── debug: rango de stamps GT ──────────────────────────────────────────
    gt_t0 = gt[0][0] / 1e9
    gt_t1 = gt[-1][0] / 1e9
    print(f'       rango sim-time GT: [{gt_t0:.1f}, {gt_t1:.1f}] s')

    print('[2/4] Leyendo Experimento A — baseline (/amcl_pose)...')
    base = leer_poses(args.base, '/amcl_pose')
    print(f'       {len(base)} poses estimadas')
    if base:
        base_t0 = base[0][0] / 1e9
        base_t1 = base[-1][0] / 1e9
        print(f'       rango sim-time A:  [{base_t0:.1f}, {base_t1:.1f}] s')

    print('[3/4] Leyendo Experimento B — WiFi (/amcl_pose)...')
    wifi = leer_poses(args.wifi, '/amcl_pose')
    print(f'       {len(wifi)} poses estimadas')
    if wifi:
        wifi_t0 = wifi[0][0] / 1e9
        wifi_t1 = wifi[-1][0] / 1e9
        print(f'       rango sim-time B:  [{wifi_t0:.1f}, {wifi_t1:.1f}] s')

    print('[4/4] Calculando ATE RMSE...')
    rmse_base, parejas_base = ate_rmse(gt, base)
    rmse_wifi, parejas_wifi = ate_rmse(gt, wifi)

    print()
    print('══════════════════════════════════════════════════════════════')
    print('   ATE (Absolute Trajectory Error)  —  RMSE y calidad        ')
    print('══════════════════════════════════════════════════════════════')
    for etiqueta, rmse, parejas in [
        ('A — AMCL puro', rmse_base, parejas_base),
        ('B — AMCL+WiFi', rmse_wifi, parejas_wifi),
    ]:
        if rmse is None:
            print(f'  {etiqueta}: sin datos')
            continue
        errs = np.array([p[5] for p in parejas])
        t_conv = _tiempo_convergencia(parejas, umbral=1.0)
        print(f'  {etiqueta}: RMSE={rmse:.3f} m  max={errs.max():.3f} m  n={len(errs)}')
        print(f'    <1.0 m: {(errs<1.0).mean()*100:.0f}%   '
              f'<0.5 m: {(errs<0.5).mean()*100:.0f}%   '
              f'convergencia (err<1m): {t_conv}')
        print()

    if rmse_base is not None and rmse_wifi is not None:
        mejora = (rmse_base - rmse_wifi) / rmse_base * 100
        signo = '↓ mejora' if mejora > 0 else '↑ empeora'
        print(f'  Diferencia RMSE: {mejora:+.1f}%  ({signo})')
    print('══════════════════════════════════════════════════════════════')

    # ── tabla detallada de errores por pareja ──────────────────────────────
    if parejas_base:
        print('\nDetalle A (baseline):')
        print('  t_sim(s)   x_est    y_est    x_gt     y_gt    err(m)  zona')
        for t, xe, ye, xg, yg, e in parejas_base:
            print(f'  {t/1e9:8.1f}  {xe:7.3f}  {ye:7.3f}  {xg:7.3f}  {yg:7.3f}'
                  f'  {e:.3f}  {_zona(xg)}')

    if parejas_wifi:
        print('\nDetalle B (WiFi):')
        print('  t_sim(s)   x_est    y_est    x_gt     y_gt    err(m)  zona')
        for t, xe, ye, xg, yg, e in parejas_wifi:
            print(f'  {t/1e9:8.1f}  {xe:7.3f}  {ye:7.3f}  {xg:7.3f}  {yg:7.3f}'
                  f'  {e:.3f}  {_zona(xg)}')


if __name__ == '__main__':
    main()
