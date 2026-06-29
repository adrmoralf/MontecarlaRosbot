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


def _zona_sim(x_gt: float) -> str:
    if x_gt > 1.5:
        return 'salon'
    if x_gt < -0.5:
        return 'hab'
    return 'pasillo'


def _zona_real(x_gt: float) -> str:
    return 'habitaciones' if x_gt > 1.5 else 'pasillo'


def _tiempo_convergencia(parejas, umbral: float = 1.0) -> str:
    """Devuelve el tiempo relativo al inicio en que el error cae por primera vez bajo umbral."""
    if not parejas:
        return 'nunca'
    t0 = parejas[0][0]
    for t, _xe, _ye, _xg, _yg, err in parejas:
        if err < umbral:
            return f'{(t - t0)/1e9:.1f}s desde inicio'
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

def _por_zona(parejas, fn_zona):
    zonas = {}
    for _, _, _, xg, _, e in parejas:
        z = fn_zona(xg)
        zonas.setdefault(z, []).append(e)
    return {z: (float(np.sqrt(np.mean(np.array(v)**2))), len(v)) for z, v in zonas.items()}


def main():
    parser = argparse.ArgumentParser(
        description='Calcula ATE RMSE comparando dos experimentos AMCL contra ground truth'
    )
    parser.add_argument('--gt',     required=True, help='Bag original (tiene /odometry/filtered)')
    parser.add_argument('--base',   required=True, help='Replay sin WiFi (tiene /amcl_pose)')
    parser.add_argument('--wifi',   required=True, help='Replay con WiFi (tiene /amcl_pose)')
    parser.add_argument('--salida', default='ate_resultado.txt',
                        help='Fichero donde guardar el resultado completo (por defecto ate_resultado.txt)')
    parser.add_argument('--real', action='store_true',
                        help='Usar zonas del entorno real (pasillo/habitaciones) en vez de simulación')
    args = parser.parse_args()

    ruta_salida = Path(args.salida)

    lineas = []

    def p(txt=''):
        print(txt)
        lineas.append(txt)

    p('[1/4] Leyendo ground truth (/odometry/filtered)...')
    gt = leer_poses(args.gt, '/odometry/filtered')
    p(f'       {len(gt)} poses GT')
    if not gt:
        p('ERROR: no hay ground truth, comprueba la ruta del bag original')
        return

    gt_t0 = gt[0][0] / 1e9
    gt_t1 = gt[-1][0] / 1e9
    p(f'       rango sim-time GT: [{gt_t0:.1f}, {gt_t1:.1f}] s')

    p('[2/4] Leyendo Experimento A — baseline (/amcl_pose)...')
    base = leer_poses(args.base, '/amcl_pose')
    p(f'       {len(base)} poses estimadas')
    if base:
        p(f'       rango sim-time A:  [{base[0][0]/1e9:.1f}, {base[-1][0]/1e9:.1f}] s')

    p('[3/4] Leyendo Experimento B — WiFi (/amcl_pose)...')
    wifi = leer_poses(args.wifi, '/amcl_pose')
    p(f'       {len(wifi)} poses estimadas')
    if wifi:
        p(f'       rango sim-time B:  [{wifi[0][0]/1e9:.1f}, {wifi[-1][0]/1e9:.1f}] s')

    fn_zona = _zona_real if args.real else _zona_sim

    p('[4/4] Calculando ATE RMSE...')
    rmse_base, parejas_base = ate_rmse(gt, base)
    rmse_wifi, parejas_wifi = ate_rmse(gt, wifi)

    p()
    p('══════════════════════════════════════════════════════════════')
    p('   ATE (Absolute Trajectory Error)  —  RMSE y calidad        ')
    p('══════════════════════════════════════════════════════════════')
    for etiqueta, rmse, parejas in [
        ('A — AMCL puro', rmse_base, parejas_base),
        ('B — AMCL+WiFi', rmse_wifi, parejas_wifi),
    ]:
        if rmse is None:
            p(f'  {etiqueta}: sin datos')
            continue
        errs = np.array([pr[5] for pr in parejas])
        t_conv = _tiempo_convergencia(parejas, umbral=1.0)
        p(f'  {etiqueta}: RMSE={rmse:.3f} m  max={errs.max():.3f} m  n={len(errs)}')
        p(f'    <1.0 m: {(errs<1.0).mean()*100:.0f}%   '
          f'<0.5 m: {(errs<0.5).mean()*100:.0f}%   '
          f'convergencia (err<1m): {t_conv}')
        for z, (rz, nz) in sorted(_por_zona(parejas, fn_zona).items()):
            p(f'    zona {z:<14}: RMSE={rz:.3f} m  n={nz}')
        p()

    if rmse_base is not None and rmse_wifi is not None:
        mejora = (rmse_base - rmse_wifi) / rmse_base * 100
        signo = '↓ mejora' if mejora > 0 else '↑ empeora'
        p(f'  Diferencia RMSE: {mejora:+.1f}%  ({signo})')
    p('══════════════════════════════════════════════════════════════')

    # ── tabla detallada (solo al fichero, para no saturar terminal) ────────
    if parejas_base:
        lineas.append('\nDetalle A (baseline):')
        lineas.append('  t_sim(s)   x_est    y_est    x_gt     y_gt    err(m)  zona')
        for t, xe, ye, xg, yg, e in parejas_base:
            lineas.append(f'  {t/1e9:8.1f}  {xe:7.3f}  {ye:7.3f}  {xg:7.3f}  {yg:7.3f}'
                          f'  {e:.3f}  {fn_zona(xg)}')

    if parejas_wifi:
        lineas.append('\nDetalle B (WiFi):')
        lineas.append('  t_sim(s)   x_est    y_est    x_gt     y_gt    err(m)  zona')
        for t, xe, ye, xg, yg, e in parejas_wifi:
            lineas.append(f'  {t/1e9:8.1f}  {xe:7.3f}  {ye:7.3f}  {xg:7.3f}  {yg:7.3f}'
                          f'  {e:.3f}  {fn_zona(xg)}')

    ruta_salida.write_text('\n'.join(lineas) + '\n')
    print(f'\n[OK] Resultado guardado en: {ruta_salida.resolve()}')


if __name__ == '__main__':
    main()
