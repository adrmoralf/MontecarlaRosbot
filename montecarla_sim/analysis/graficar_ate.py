#!/usr/bin/env python3
"""
graficar_ate.py — Gráficas comparativas A vs B para el TFG Montecarla.

Lee los datos directamente de los bags (igual que ate_calc.py).

Uso:
  python3 graficar_ate.py \
    --gt   ~/sim_bags/casa_XXXX \
    --base ~/sim_bags/replay_false_XXXX \
    --wifi ~/sim_bags/replay_true_XXXX \
    --salida /ruta/carpeta/
"""

import argparse
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ── Lectura de bags ───────────────────────────────────────────────────────────

def leer_poses(ruta_bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(ruta_bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in tipos:
        return []
    tipo_msg = get_message(tipos[topic])
    poses = []
    while reader.has_next():
        t, datos, _ = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(datos, tipo_msg)
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        poses.append((stamp_ns, msg.pose.pose.position.x, msg.pose.pose.position.y))
    return poses


def ate_emparejado(gt, estimado, ventana_ns=2_000_000_000):
    gt_arr = np.array([(t, x, y) for t, x, y in gt])
    parejas = []
    for t_est, x_est, y_est in estimado:
        diffs = np.abs(gt_arr[:, 0] - t_est)
        idx = int(np.argmin(diffs))
        if diffs[idx] > ventana_ns:
            continue
        x_gt, y_gt = gt_arr[idx, 1], gt_arr[idx, 2]
        error = float(np.sqrt((x_est - x_gt)**2 + (y_est - y_gt)**2))
        parejas.append((t_est / 1e9, x_gt, y_gt, error))
    return parejas


def zona(x_gt):
    if x_gt > 1.5:
        return 'salon'
    if x_gt < -0.5:
        return 'hab'
    return 'pasillo'


# ── Gráficas ──────────────────────────────────────────────────────────────────

def graficar(datos_A, datos_B, salida_dir):
    os.makedirs(salida_dir, exist_ok=True)

    tA = np.array([d[0] for d in datos_A])
    eA = np.array([d[3] for d in datos_A])
    tB = np.array([d[0] for d in datos_B])
    eB = np.array([d[3] for d in datos_B])

    zonas_A = [zona(d[1]) for d in datos_A]
    zonas_B = [zona(d[1]) for d in datos_B]

    rmse_A = float(np.sqrt(np.mean(eA**2)))
    rmse_B = float(np.sqrt(np.mean(eB**2)))
    mejora = (rmse_A - rmse_B) / rmse_A * 100

    colores_zona = {'salon': '#e07b39', 'hab': '#4c72b0', 'pasillo': '#55a868'}
    t_max = max(tA[-1], tB[-1]) if len(tA) and len(tB) else 400

    # ── Figura 1: Error a lo largo del tiempo ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(13, 4.5))

    ax.plot(tA, eA, color='#888888', linewidth=1.2, alpha=0.7, label='A — AMCL puro', zorder=1)
    ax.plot(tB, eB, color='#2166ac', linewidth=1.5, label='B — AMCL+WiFi', zorder=2)

    zonas_cambios = [(tA[0], zonas_A[0])]
    for i in range(1, len(zonas_A)):
        if zonas_A[i] != zonas_A[i-1]:
            zonas_cambios.append((tA[i], zonas_A[i]))
    zonas_cambios.append((tA[-1], None))
    for i in range(len(zonas_cambios) - 1):
        t0, z = zonas_cambios[i]
        t1 = zonas_cambios[i+1][0]
        ax.axvspan(t0, t1, alpha=0.08, color=colores_zona[z], zorder=0)

    ax.axhline(1.0, color='#999', linestyle='--', linewidth=0.8, alpha=0.6, label='1 m umbral')
    ax.axhline(rmse_A, color='#888888', linestyle='--', linewidth=1.2, alpha=0.5)
    ax.axhline(rmse_B, color='#2166ac', linestyle='--', linewidth=1.2, alpha=0.5)
    ax.text(t_max * 0.98, rmse_A + 0.12, f'RMSE A={rmse_A:.2f}m', color='#666', fontsize=8, ha='right')
    ax.text(t_max * 0.98, rmse_B - 0.25, f'RMSE B={rmse_B:.2f}m', color='#2166ac', fontsize=8, ha='right')

    leyenda_zonas = [mpatches.Patch(color=colores_zona[z], alpha=0.3, label=z.capitalize())
                     for z in ['hab', 'pasillo', 'salon']]
    leg1 = ax.legend(handles=leyenda_zonas, loc='upper left', fontsize=8, title='Zona')
    ax.add_artist(leg1)
    ax.legend(loc='upper right', fontsize=9)

    ax.set_xlabel('Tiempo de simulación (s)', fontsize=11)
    ax.set_ylabel('Error de localización (m)', fontsize=11)
    ax.set_title(f'ATE a lo largo del tiempo — AMCL puro vs AMCL+WiFi  (mejora {mejora:+.1f}%)',
                 fontsize=12, fontweight='bold')
    ax.set_xlim(0, t_max * 1.02)
    ax.set_ylim(-0.1, min(max(eA.max(), eB.max()) + 1.0, 9.0))
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_temporal.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    # ── Figura 2: Distribución de errores (CDF + histograma) ──────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    ax = axes[0]
    for datos, color, label in [(eA, '#888888', 'A — AMCL puro'), (eB, '#2166ac', 'B — AMCL+WiFi')]:
        datos_sort = np.sort(datos)
        cdf = np.arange(1, len(datos_sort)+1) / len(datos_sort)
        ax.plot(datos_sort, cdf * 100, color=color, linewidth=2, label=label)
    ax.axvline(1.0, color='#e07b39', linestyle='--', linewidth=1, label='1 m')
    ax.axvline(2.0, color='#e07b39', linestyle=':', linewidth=1, label='2 m')
    for datos, color in [(eA, '#888'), (eB, '#2166ac')]:
        p1 = np.mean(datos < 1.0) * 100
        ax.annotate(f'{p1:.0f}%', xy=(1.0, p1), xytext=(2.2, p1),
                    fontsize=8, color=color,
                    arrowprops=dict(arrowstyle='->', color=color, lw=0.8))
    ax.set_xlabel('Error (m)', fontsize=11)
    ax.set_ylabel('% de muestras', fontsize=11)
    ax.set_title('CDF del error de localización', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 7)
    ax.set_ylim(0, 102)

    ax = axes[1]
    e_max = max(eA.max(), eB.max())
    bins = np.arange(0, e_max + 0.5, 0.5)
    ax.hist(eA, bins=bins, color='#888888', alpha=0.6, label='A — AMCL puro', density=True)
    ax.hist(eB, bins=bins, color='#2166ac', alpha=0.6, label='B — AMCL+WiFi', density=True)
    ax.axvline(rmse_A, color='#555', linestyle='--', linewidth=1.5, label=f'RMSE A={rmse_A:.2f}m')
    ax.axvline(rmse_B, color='#2166ac', linestyle='--', linewidth=1.5, label=f'RMSE B={rmse_B:.2f}m')
    ax.set_xlabel('Error (m)', fontsize=11)
    ax.set_ylabel('Densidad', fontsize=11)
    ax.set_title('Distribución del error de localización', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_distribucion.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    # ── Figura 3: Comparativa por zona ────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    zonas_orden = ['hab', 'pasillo', 'salon']
    zonas_label = ['Habitaciones\n(dorm+cocina)', 'Pasillo', 'Salón']

    rmse_por_zona_A, rmse_por_zona_B, n_A_z, n_B_z = [], [], [], []
    for z in zonas_orden:
        eA_z = np.array([d[3] for d in datos_A if zona(d[1]) == z])
        eB_z = np.array([d[3] for d in datos_B if zona(d[1]) == z])
        rmse_por_zona_A.append(float(np.sqrt(np.mean(eA_z**2))) if len(eA_z) else 0)
        rmse_por_zona_B.append(float(np.sqrt(np.mean(eB_z**2))) if len(eB_z) else 0)
        n_A_z.append(len(eA_z))
        n_B_z.append(len(eB_z))

    x = np.arange(len(zonas_orden))
    ancho = 0.35
    ax.bar(x - ancho/2, rmse_por_zona_A, ancho, label='A — AMCL puro', color='#888888', alpha=0.85)
    ax.bar(x + ancho/2, rmse_por_zona_B, ancho, label='B — AMCL+WiFi', color='#2166ac', alpha=0.85)

    for i, (rA, rB) in enumerate(zip(rmse_por_zona_A, rmse_por_zona_B)):
        if rA > 0:
            m = (rA - rB) / rA * 100
            signo = '↓' if m > 0 else '↑'
            color = '#2c7e45' if m > 0 else '#c0392b'
            ax.text(x[i], max(rA, rB) + 0.1, f'{signo}{abs(m):.0f}%',
                    ha='center', fontsize=10, fontweight='bold', color=color)
        ax.text(x[i] - ancho/2, rA + 0.04, f'n={n_A_z[i]}', ha='center', fontsize=7, color='#555')
        ax.text(x[i] + ancho/2, rB + 0.04, f'n={n_B_z[i]}', ha='center', fontsize=7, color='#2166ac')

    ax.set_xticks(x)
    ax.set_xticklabels(zonas_label, fontsize=10)
    ax.set_ylabel('RMSE (m)', fontsize=11)
    ax.set_title('RMSE por zona', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, max(max(rmse_por_zona_A), max(rmse_por_zona_B)) + 0.9)

    ax = axes[1]
    datos_boxplot, colores_box, etiquetas_box = [], [], []
    for z, zl in zip(zonas_orden, zonas_label):
        datos_boxplot.extend([[d[3] for d in datos_A if zona(d[1]) == z],
                               [d[3] for d in datos_B if zona(d[1]) == z]])
        colores_box.extend(['#aaaaaa', '#4488cc'])
        etiquetas_box.extend([f'A\n{zl.split(chr(10))[0]}', f'B\n{zl.split(chr(10))[0]}'])

    bp = ax.boxplot(datos_boxplot, patch_artist=True,
                    medianprops=dict(color='black', linewidth=1.5))
    for patch, color in zip(bp['boxes'], colores_box):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax.set_xticklabels(etiquetas_box, fontsize=8)
    ax.set_ylabel('Error (m)', fontsize=11)
    ax.set_title('Distribución por zona (boxplot)', fontsize=11, fontweight='bold')
    ax.axhline(1.0, color='#e07b39', linestyle='--', linewidth=0.9, alpha=0.7, label='1 m')
    ax.axhline(2.0, color='#e07b39', linestyle=':', linewidth=0.9, alpha=0.5, label='2 m')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    ax.axvline(2.5, color='#ccc', linewidth=0.8)
    ax.axvline(4.5, color='#ccc', linewidth=0.8)

    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_por_zona.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    # ── Figura 4: Mapa de calor 2D ────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    for ax, datos, titulo in [
        (axes[0], datos_A, f'A — AMCL puro\nRMSE={rmse_A:.3f} m'),
        (axes[1], datos_B, f'B — AMCL+WiFi\nRMSE={rmse_B:.3f} m  ({mejora:+.1f}%)'),
    ]:
        xs = [d[1] for d in datos]
        ys = [d[2] for d in datos]
        errs = [d[3] for d in datos]
        cmap = 'Reds' if 'puro' in titulo else 'Blues'

        sc = ax.scatter(xs, ys, c=errs, cmap=cmap, s=55, vmin=0, vmax=6,
                        edgecolors='black', linewidths=0.3, alpha=0.85, zorder=3)
        plt.colorbar(sc, ax=ax, label='Error (m)', shrink=0.85)

        paredes = [
            [(-5.0, -3.5), (5.0, -3.5), (5.0, 3.5), (-5.0, 3.5), (-5.0, -3.5)],
            [(-1.0, -3.5), (-1.0, -2.5)], [(-1.0, -1.5), (-1.0,  1.5)],
            [(-1.0,  2.5), (-1.0,  3.5)],
            [( 1.0, -3.5), ( 1.0, -0.5)], [( 1.0,  0.5), ( 1.0,  3.5)],
            [(-5.0,  0.0), (-1.0,  0.0)],
        ]
        for pared in paredes:
            ax.plot([p[0] for p in pared], [p[1] for p in pared],
                    'k-', linewidth=1.5, zorder=2)

        for apx, apy, apl in [(-3.0, 2.0, 'AP1'), (3.0, 1.0, 'AP2'), (-3.0, -2.0, 'AP3')]:
            ax.plot(apx, apy, 'y*', markersize=14, zorder=4)
            ax.text(apx + 0.2, apy + 0.2, apl, fontsize=7, color='#885500')

        ax.plot(0, 0, 'gP', markersize=10, zorder=4, label='Inicio')
        ax.set_title(titulo, fontsize=10, fontweight='bold')
        ax.set_xlabel('x (m)', fontsize=9)
        ax.set_ylabel('y (m)', fontsize=9)
        ax.set_xlim(-5.3, 5.3)
        ax.set_ylim(-3.8, 3.8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        ax.legend(fontsize=8, loc='lower right')
        for tx, ty, tl in [(-3.0, 1.5, 'DORM'), (-3.0, -1.5, 'COC'),
                            (3.0, 0.0, 'SALÓN'), (0.0, -2.5, 'PAS')]:
            ax.text(tx, ty, tl, fontsize=8, color='#666', ha='center', alpha=0.6)

    plt.suptitle('Posición ground truth coloreada por error — puntos azules oscuros = error bajo',
                 fontsize=11, fontweight='bold')
    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_mapa_calor.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    print(f'\n4 gráficas guardadas en: {salida_dir}')
    print(f'Resumen: A={rmse_A:.3f}m  B={rmse_B:.3f}m  mejora={mejora:+.1f}%  '
          f'<1m A={np.mean(eA<1)*100:.0f}%  <1m B={np.mean(eB<1)*100:.0f}%')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt',     required=True, help='Bag original (ground truth)')
    parser.add_argument('--base',   required=True, help='Replay sin WiFi')
    parser.add_argument('--wifi',   required=True, help='Replay con WiFi')
    parser.add_argument('--salida', default=str(Path(__file__).parent),
                        help='Directorio de salida')
    args = parser.parse_args()

    print('Leyendo bags...')
    gt   = leer_poses(args.gt,   '/odometry/filtered')
    base = leer_poses(args.base, '/amcl_pose')
    wifi = leer_poses(args.wifi, '/amcl_pose')
    print(f'  GT: {len(gt)} poses   A: {len(base)} poses   B: {len(wifi)} poses')

    datos_A = ate_emparejado(gt, base)
    datos_B = ate_emparejado(gt, wifi)
    print(f'  Emparejados: A={len(datos_A)}  B={len(datos_B)}')

    graficar(datos_A, datos_B, args.salida)


if __name__ == '__main__':
    main()
