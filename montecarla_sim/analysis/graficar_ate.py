#!/usr/bin/env python3
"""
graficar_ate.py — Gráficas comparativas A vs B para el TFG Montecarla.

Uso:
  python3 graficar_ate.py
  python3 graficar_ate.py --salida /ruta/carpeta/

Las gráficas se guardan en la carpeta indicada (por defecto: junto al script).
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

# ── Datos embebidos del experimento (bag casa_20260610, 87/85 updates) ────────

# Formato: (t_sim_s, x_gt, y_gt, error_m)
DATOS_A = [
    (4.1, 0.0, 0.0, 0.0), (16.4, 1.313, 0.053, 0.204), (18.4, 2.191, 1.249, 0.794),
    (19.7, 1.977, 2.194, 1.284), (25.6, 4.127, 1.954, 2.944), (25.7, 4.127, 1.888, 3.224),
    (30.4, 4.156, -2.383, 4.817), (32.7, 3.551, -2.992, 5.004), (40.6, 2.371, -1.344, 3.258),
    (41.3, 2.959, -1.188, 3.852), (43.2, 4.077, -0.330, 4.764), (43.4, 4.106, -0.230, 4.886),
    (43.7, 4.117, -0.129, 4.896), (45.5, 3.534, 0.655, 4.394), (49.7, 0.318, 0.193, 1.247),
    (53.5, -0.778, 1.980, 1.925), (53.9, -1.098, 2.006, 2.131), (54.0, -1.193, 2.008, 2.092),
    (56.3, -3.128, 2.710, 3.679), (60.2, -3.947, 0.812, 3.254), (62.5, -2.607, 0.632, 1.895),
    (63.8, -1.652, 0.965, 1.186), (66.3, -1.487, 2.151, 2.252), (66.6, -1.545, 2.251, 2.321),
    (66.8, -1.609, 2.327, 2.347), (70.2, -3.467, 2.288, 3.318), (70.5, -3.524, 2.119, 3.148),
    (71.9, -3.326, 1.302, 2.424), (72.0, -3.265, 1.228, 2.256), (73.2, -3.058, 0.860, 1.785),
    (75.6, -3.339, 2.238, 2.895), (76.1, -3.277, 2.045, 2.629), (92.0, 0.539, -2.184, 1.074),
    (92.5, 0.392, -2.241, 2.834), (94.4, -1.060, -2.070, 2.073), (94.5, -1.151, -2.040, 2.024),
    (95.3, -1.718, -1.972, 2.116), (96.8, -3.053, -2.453, 3.228), (99.2, -4.369, -2.266, 4.050),
    (102.6, -3.891, -0.445, 2.860), (108.1, -1.927, -0.920, 1.197), (109.7, -2.109, -2.097, 2.422),
    (111.1, -2.808, -2.773, 3.291), (125.1, -2.335, -1.899, 2.295), (127.0, -0.723, -1.947, 1.969),
    (132.1, 0.641, -0.252, 1.741), (138.9, -2.439, 2.358, 2.346), (139.0, -2.530, 2.385, 2.675),
    (140.7, -3.557, 2.180, 3.109), (147.2, -1.644, 0.900, 0.893), (149.2, -1.463, 1.785, 1.660),
    (153.1, -3.572, 2.716, 3.634), (153.4, -3.758, 2.598, 3.558), (153.8, -3.873, 2.482, 3.614),
    (155.7, -3.936, 1.327, 2.984), (157.3, -3.364, 0.689, 2.228), (158.1, -2.770, 0.551, 1.602),
    (160.6, -1.311, 1.489, 1.480), (164.1, 0.431, 1.412, 1.944), (166.6, 0.310, -0.594, 1.583),
    (167.7, -0.015, -1.072, 1.458), (170.5, -2.177, -2.497, 2.842), (171.1, -2.713, -2.701, 3.095),
    (177.9, -3.058, -0.470, 2.108), (182.2, -1.764, -2.330, 2.558), (187.6, -4.327, -1.606, 3.660),
    (189.3, -3.785, -0.657, 3.049), (195.3, -0.568, -2.302, 1.900), (196.0, -0.303, -2.394, 2.355),
    (199.4, -0.424, -2.024, 1.987), (202.0, -0.151, 0.345, 0.579), (203.0, 0.167, 1.242, 1.423),
    (203.4, 0.320, 1.595, 1.690), (211.4, 0.484, -3.288, 3.587), (214.0, 0.516, -1.485, 2.021),
    (216.0, 1.016, -0.161, 1.493), (218.4, 3.105, 0.520, 3.111), (223.0, 3.786, 1.803, 3.269),
    (223.4, 3.695, 2.051, 4.850), (224.7, 3.036, 2.518, 4.929), (230.2, 1.932, 0.126, 3.065),
    (230.3, 1.933, 0.029, 3.591), (232.8, 1.950, -1.019, 3.219), (235.2, 2.328, -2.702, 5.031),
    (243.4, 4.033, 1.909, 7.922), (246.3, 2.936, 2.275, 6.293), (250.0, 2.819, -0.079, 4.762),
]

DATOS_B = [
    (6.6, 0.0, 0.0, 0.0), (17.3, 1.819, 0.457, 0.666), (22.7, 2.870, 2.875, 1.432),
    (23.5, 3.405, 2.705, 1.087), (31.5, 3.961, -2.710, 3.156), (42.1, 3.610, -0.813, 0.903),
    (43.9, 4.112, -0.031, 1.368), (50.4, -0.013, 0.475, 2.695), (53.5, -0.778, 1.980, 1.968),
    (54.0, -1.193, 2.008, 1.501), (55.8, -2.699, 2.581, 1.802), (56.3, -3.128, 2.710, 1.738),
    (56.4, -3.195, 2.719, 1.653), (56.7, -3.404, 2.718, 1.598), (57.4, -3.776, 2.585, 1.035),
    (59.6, -4.136, 1.217, 0.855), (61.0, -3.683, 0.627, 0.735), (61.8, -3.218, 0.585, 0.365),
    (65.7, -1.378, 1.708, 1.095), (65.8, -1.392, 1.791, 0.947), (69.4, -3.124, 2.690, 0.619),
    (69.8, -3.332, 2.499, 0.860), (71.3, -3.531, 1.654, 0.809), (71.9, -3.326, 1.302, 0.545),
    (87.0, 0.377, -1.574, 3.584), (88.5, 0.147, -2.361, 3.412), (89.0, -0.173, -2.625, 3.301),
    (92.7, 0.263, -2.272, 5.497), (96.4, -2.684, -2.313, 0.829), (99.7, -4.529, -1.940, 1.950),
    (99.8, -4.558, -1.849, 1.939), (99.9, -4.588, -1.749, 1.889), (101.1, -4.616, -1.104, 1.707),
    (102.1, -4.233, -0.629, 1.526), (111.1, -2.808, -2.773, 0.572), (111.5, -3.050, -2.819, 0.816),
    (112.2, -3.582, -2.752, 1.232), (113.2, -4.223, -2.316, 1.213), (115.9, -3.437, -2.838, 0.876),
    (116.4, -2.966, -2.940, 0.947), (117.0, -2.477, -3.042, 1.126), (119.1, -3.310, -2.859, 0.889),
    (123.6, -3.463, -1.654, 0.555), (125.4, -2.073, -1.967, 0.751), (127.0, -0.723, -1.947, 2.096),
    (128.4, 0.230, -2.057, 2.365), (129.5, 0.295, -2.011, 2.008), (129.6, 0.295, -2.011, 1.059),
    (135.5, 0.207, 1.733, 1.633), (141.9, -3.814, 1.586, 1.625), (142.3, -3.791, 1.259, 1.331),
    (144.2, -3.694, 0.623, 0.704), (151.1, -2.288, 2.541, 0.859), (155.1, -4.022, 1.881, 0.956),
    (160.6, -1.311, 1.489, 2.218), (162.2, -0.575, 1.954, 2.317), (163.7, 0.276, 1.628, 3.109),
    (171.1, -2.713, -2.701, 0.685), (171.8, -3.206, -2.829, 0.330), (172.4, -3.567, -2.791, 0.419),
    (173.3, -4.070, -2.526, 1.448), (174.5, -4.321, -1.892, 1.698), (176.9, -3.733, -0.582, 1.355),
    (182.7, -2.038, -2.530, 0.825), (183.8, -3.024, -2.751, 0.589), (185.9, -4.110, -2.544, 1.007),
    (187.2, -4.381, -1.890, 0.995), (189.3, -3.785, -0.657, 1.029), (193.7, -1.560, -1.946, 1.539),
    (194.2, -1.199, -2.126, 1.585), (194.6, -0.912, -2.259, 1.348), (199.4, -0.424, -2.024, 1.434),
    (200.0, -0.442, -1.520, 0.754), (200.1, -0.432, -1.425, 0.321), (202.0, -0.151, 0.345, 1.918),
    (204.6, 0.523, 2.235, 1.752), (209.0, -0.086, -1.325, 0.877), (209.7, 0.110, -1.974, 1.586),
    (214.3, 0.528, -1.251, 1.072), (221.8, 3.927, 0.826, 5.806), (224.7, 3.036, 2.518, 7.505),
    (231.7, 1.942, -0.434, 1.684), (240.1, 4.196, -1.063, 0.947), (242.0, 4.145, 0.778, 2.972),
    (252.5, 2.819, -0.079, 0.308),
]


def zona(x_gt):
    if x_gt > 1.5:
        return 'salon'
    if x_gt < -0.5:
        return 'hab'
    return 'pasillo'


def graficar(salida_dir):
    os.makedirs(salida_dir, exist_ok=True)

    tA = np.array([d[0] for d in DATOS_A])
    eA = np.array([d[3] for d in DATOS_A])
    tB = np.array([d[0] for d in DATOS_B])
    eB = np.array([d[3] for d in DATOS_B])

    zonas_A = [zona(d[1]) for d in DATOS_A]
    zonas_B = [zona(d[1]) for d in DATOS_B]

    colores_zona = {'salon': '#e07b39', 'hab': '#4c72b0', 'pasillo': '#55a868'}

    # ── Figura 1: Error a lo largo del tiempo ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4.5))

    ax.plot(tA, eA, color='#888888', linewidth=1.2, alpha=0.7, label='A — AMCL puro', zorder=1)
    ax.plot(tB, eB, color='#2166ac', linewidth=1.5, label='B — AMCL+WiFi', zorder=2)

    # Sombrear franjas por zona (usando GT del experimento A como referencia)
    zonas_cambios = [(tA[0], zonas_A[0])]
    for i in range(1, len(zonas_A)):
        if zonas_A[i] != zonas_A[i-1]:
            zonas_cambios.append((tA[i], zonas_A[i]))
    zonas_cambios.append((tA[-1], None))

    for i in range(len(zonas_cambios) - 1):
        t0, z = zonas_cambios[i]
        t1 = zonas_cambios[i+1][0]
        ax.axvspan(t0, t1, alpha=0.08, color=colores_zona[z], zorder=0)

    # Líneas de referencia
    ax.axhline(1.0, color='#999', linestyle='--', linewidth=0.8, alpha=0.6, label='1 m umbral')
    ax.axhline(2.0, color='#bbb', linestyle=':', linewidth=0.8, alpha=0.5)

    rmse_A = float(np.sqrt(np.mean(eA**2)))
    rmse_B = float(np.sqrt(np.mean(eB**2)))
    ax.axhline(rmse_A, color='#888888', linestyle='--', linewidth=1.2, alpha=0.5)
    ax.axhline(rmse_B, color='#2166ac', linestyle='--', linewidth=1.2, alpha=0.5)

    ax.text(252, rmse_A + 0.12, f'RMSE A={rmse_A:.2f}m', color='#666', fontsize=8, ha='right')
    ax.text(252, rmse_B - 0.25, f'RMSE B={rmse_B:.2f}m', color='#2166ac', fontsize=8, ha='right')

    # Leyenda zonas
    leyenda_zonas = [mpatches.Patch(color=colores_zona[z], alpha=0.3, label=z.capitalize())
                     for z in ['hab', 'pasillo', 'salon']]
    leg1 = ax.legend(handles=leyenda_zonas, loc='upper left', fontsize=8, title='Zona')
    ax.add_artist(leg1)
    ax.legend(loc='upper right', fontsize=9)

    ax.set_xlabel('Tiempo de simulación (s)', fontsize=11)
    ax.set_ylabel('Error de localización (m)', fontsize=11)
    ax.set_title('ATE a lo largo del tiempo — AMCL puro vs AMCL+WiFi', fontsize=12, fontweight='bold')
    ax.set_xlim(0, 256)
    ax.set_ylim(-0.1, 8.5)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_temporal.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    # ── Figura 2: Distribución de errores (CDF + histograma) ──────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # CDF
    ax = axes[0]
    for datos, color, label in [(eA, '#888888', 'A — AMCL puro'), (eB, '#2166ac', 'B — AMCL+WiFi')]:
        datos_sort = np.sort(datos)
        cdf = np.arange(1, len(datos_sort)+1) / len(datos_sort)
        ax.plot(datos_sort, cdf * 100, color=color, linewidth=2, label=label)
    ax.axvline(1.0, color='#e07b39', linestyle='--', linewidth=1, label='1 m')
    ax.axvline(2.0, color='#e07b39', linestyle=':', linewidth=1, label='2 m')
    ax.set_xlabel('Error (m)', fontsize=11)
    ax.set_ylabel('% de muestras', fontsize=11)
    ax.set_title('CDF del error de localización', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 8.5)
    ax.set_ylim(0, 102)

    # Anotar % < 1m y < 2m
    for datos, color, offset in [(eA, '#888', 0.02), (eB, '#2166ac', -0.02)]:
        p1 = np.mean(datos < 1.0) * 100
        p2 = np.mean(datos < 2.0) * 100
        ax.annotate(f'{p1:.0f}%', xy=(1.0, p1), xytext=(1.6, p1 + offset*100),
                    fontsize=8, color=color,
                    arrowprops=dict(arrowstyle='->', color=color, lw=0.8))

    # Histograma
    ax = axes[1]
    bins = np.arange(0, 8.6, 0.5)
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

    # Bar chart RMSE por zona
    ax = axes[0]
    zonas_orden = ['hab', 'pasillo', 'salon']
    zonas_label = ['Habitaciones\n(dorm+cocina)', 'Pasillo', 'Salón']

    rmse_por_zona_A, rmse_por_zona_B = [], []
    n_A, n_B = [], []
    for z in zonas_orden:
        eA_z = np.array([d[3] for d in DATOS_A if zona(d[1]) == z])
        eB_z = np.array([d[3] for d in DATOS_B if zona(d[1]) == z])
        rmse_por_zona_A.append(float(np.sqrt(np.mean(eA_z**2))) if len(eA_z) else 0)
        rmse_por_zona_B.append(float(np.sqrt(np.mean(eB_z**2))) if len(eB_z) else 0)
        n_A.append(len(eA_z))
        n_B.append(len(eB_z))

    x = np.arange(len(zonas_orden))
    ancho = 0.35
    barras_A = ax.bar(x - ancho/2, rmse_por_zona_A, ancho, label='A — AMCL puro', color='#888888', alpha=0.85)
    barras_B = ax.bar(x + ancho/2, rmse_por_zona_B, ancho, label='B — AMCL+WiFi', color='#2166ac', alpha=0.85)

    for i, (rA, rB) in enumerate(zip(rmse_por_zona_A, rmse_por_zona_B)):
        mejora = (rA - rB) / rA * 100
        signo = '↓' if mejora > 0 else '↑'
        color = '#2c7e45' if mejora > 0 else '#c0392b'
        ax.text(x[i], max(rA, rB) + 0.1, f'{signo}{abs(mejora):.0f}%',
                ha='center', fontsize=10, fontweight='bold', color=color)
        ax.text(x[i] - ancho/2, rA + 0.05, f'n={n_A[i]}', ha='center', fontsize=7, color='#555')
        ax.text(x[i] + ancho/2, rB + 0.05, f'n={n_B[i]}', ha='center', fontsize=7, color='#2166ac')

    ax.set_xticks(x)
    ax.set_xticklabels(zonas_label, fontsize=10)
    ax.set_ylabel('RMSE (m)', fontsize=11)
    ax.set_title('RMSE por zona', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, max(max(rmse_por_zona_A), max(rmse_por_zona_B)) + 0.8)

    # Boxplot por zona de B vs A
    ax = axes[1]
    datos_boxplot = []
    colores_box = []
    etiquetas_box = []
    for z, zl in zip(zonas_orden, zonas_label):
        eA_z = [d[3] for d in DATOS_A if zona(d[1]) == z]
        eB_z = [d[3] for d in DATOS_B if zona(d[1]) == z]
        datos_boxplot.extend([eA_z, eB_z])
        colores_box.extend(['#aaaaaa', '#4488cc'])
        etiquetas_box.extend([f'A\n{zl.split(chr(10))[0]}', f'B\n{zl.split(chr(10))[0]}'])

    bp = ax.boxplot(datos_boxplot, patch_artist=True, notch=False,
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

    # Separadores de zona
    ax.axvline(2.5, color='#ccc', linewidth=0.8)
    ax.axvline(4.5, color='#ccc', linewidth=0.8)

    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_por_zona.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    # ── Figura 4: Mapa de calor de errores en planta 2D ───────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    for ax, datos, titulo, cmap in [
        (axes[0], DATOS_A, 'A — AMCL puro', 'Reds'),
        (axes[1], DATOS_B, 'B — AMCL+WiFi', 'Blues'),
    ]:
        xs = [d[1] for d in datos]
        ys = [d[2] for d in datos]
        errs = [d[3] for d in datos]

        sc = ax.scatter(xs, ys, c=errs, cmap=cmap, s=60, vmin=0, vmax=6,
                        edgecolors='black', linewidths=0.3, alpha=0.85, zorder=3)
        plt.colorbar(sc, ax=ax, label='Error (m)', shrink=0.85)

        # Plano simplificado de la casa (paredes)
        paredes = [
            # Exterior
            [(-5.0, -3.5), (5.0, -3.5), (5.0, 3.5), (-5.0, 3.5), (-5.0, -3.5)],
            # Pared pasillo-hab (vertical x=-0.5)
            [(-0.5, -3.5), (-0.5, 0.5)],
            # Separación dormitorio/cocina (horizontal y=0)
            [(-5.0, 0.0), (-0.5, 0.0)],
            # Pared salon-pasillo (vertical x=1.5)
            [(1.5, -3.5), (1.5, 3.5)],
        ]
        for pared in paredes:
            px = [p[0] for p in pared]
            py = [p[1] for p in pared]
            ax.plot(px, py, 'k-', linewidth=1.5, zorder=2)

        # APs
        aps = [(-3.0, 2.0, 'AP1\nDorm'), (3.0, 1.0, 'AP2\nSalon'), (-3.0, -2.0, 'AP3\nCoc')]
        for apx, apy, apl in aps:
            ax.plot(apx, apy, 'y*', markersize=14, zorder=4)
            ax.text(apx + 0.15, apy + 0.2, apl, fontsize=7, color='#885500')

        # Spawn
        ax.plot(0, 0, 'gP', markersize=10, zorder=4, label='Inicio')

        rmse_z = float(np.sqrt(np.mean(np.array(errs)**2)))
        ax.set_title(f'{titulo}\nRMSE={rmse_z:.3f} m', fontsize=10, fontweight='bold')
        ax.set_xlabel('x (m)', fontsize=9)
        ax.set_ylabel('y (m)', fontsize=9)
        ax.set_xlim(-5.3, 5.3)
        ax.set_ylim(-3.8, 3.8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        ax.legend(fontsize=8, loc='lower right')

        # Etiquetas de zonas
        for tx, ty, tl in [(-2.5, 2.5, 'DORM'), (-2.5, -2.0, 'COCINA'),
                             (3.3, 0.5, 'SALON'), (0.4, -2.0, 'PASILLO')]:
            ax.text(tx, ty, tl, fontsize=8, color='#666', ha='center', alpha=0.6)

    plt.suptitle('Posición ground truth coloreada por error de localización',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    ruta = os.path.join(salida_dir, 'ate_mapa_calor.png')
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Guardada: {ruta}')

    print()
    print(f'4 graficas guardadas en: {salida_dir}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--salida', default=str(Path(__file__).parent),
                        help='Directorio de salida de las graficas')
    args = parser.parse_args()
    graficar(args.salida)


if __name__ == '__main__':
    main()
