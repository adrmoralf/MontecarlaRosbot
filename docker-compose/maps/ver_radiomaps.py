#!/usr/bin/env python3
"""Visualiza los radiomaps generados superpuestos sobre el mapa SLAM."""

import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from pathlib import Path
from PIL import Image

MAPS_DIR = Path(__file__).parent

# ── Cargar metadata ────────────────────────────────────────────────────────────
with open(MAPS_DIR / 'radiomap_meta.yaml') as f:
    meta = yaml.safe_load(f)

res  = meta['resolution']
ox   = meta['origin'][0]
oy   = meta['origin'][1]
W    = meta['width']
H    = meta['height']

# Extensión del mapa en metros (para ejes)
extent = [ox, ox + W * res, oy, oy + H * res]

# ── Cargar mapa SLAM (PGM) ─────────────────────────────────────────────────────
pgm = np.array(Image.open(MAPS_DIR / 'casa_simple.pgm'))
# nav2_map_server: 0=ocupado(negro), 205=desconocido(gris), 254=libre(blanco)
# Invertimos para que libre = blanco, ocupado = negro
mapa_display = pgm.astype(float)

# ── Cargar radiomaps ───────────────────────────────────────────────────────────
nombres_ap = {
    '02:00:00:00:00:01': 'AP1 · Dormitorio  (-3, 2)',
    '02:00:00:00:00:02': 'AP2 · Salón        ( 3, 1)',
    '02:00:00:00:00:03': 'AP3 · Cocina       (-3,-2)',
}
colores_ap = ['Blues_r', 'Reds_r', 'Greens_r']

bssids = meta['bssids']
radiomaps = {}
for bssid in bssids:
    nombre_fichero = 'radiomap_' + bssid.replace(':', '_') + '.npy'
    radiomaps[bssid] = np.load(MAPS_DIR / nombre_fichero)

# ── Figura: un subplot por AP + uno combinado ──────────────────────────────────
fig, axes = plt.subplots(1, 4, figsize=(20, 5))
fig.suptitle('Radiomaps Montecarla — casa_simple (10×7 m²)', fontsize=13, fontweight='bold')

for idx, (bssid, cmap_nombre) in enumerate(zip(bssids, colores_ap)):
    ax = axes[idx]
    rm = radiomaps[bssid]

    # Fondo: mapa SLAM en escala de grises
    ax.imshow(mapa_display, cmap='gray', origin='lower', extent=extent, alpha=0.5)

    # Radiomap encima (solo celdas con dato)
    rm_masked = np.ma.masked_invalid(rm)
    im = ax.imshow(rm_masked, cmap=cmap_nombre, origin='lower', extent=extent,
                   alpha=0.85, vmin=-90, vmax=-35)

    # Posición del AP
    ap_pos = {
        '02:00:00:00:00:01': (-3.0, 2.0),
        '02:00:00:00:00:02': ( 3.0, 1.0),
        '02:00:00:00:00:03': (-3.0,-2.0),
    }[bssid]
    ax.plot(*ap_pos, 'k*', markersize=12, label='AP')
    ax.annotate('AP', ap_pos, textcoords='offset points', xytext=(5, 5), fontsize=8)

    # Spawn del robot
    ax.plot(0, 0, 'go', markersize=8, label='Spawn')

    plt.colorbar(im, ax=ax, label='RSSI (dBm)', fraction=0.046, pad=0.04)
    ax.set_title(nombres_ap.get(bssid, bssid), fontsize=9)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(ox, ox + W * res)
    ax.set_ylim(oy, oy + H * res)

# ── Subplot combinado: los 3 APs superpuestos ─────────────────────────────────
ax = axes[3]
ax.imshow(mapa_display, cmap='gray', origin='lower', extent=extent, alpha=0.5)

alphas_comb = [0.55, 0.55, 0.55]
for bssid, cmap_nombre, alpha in zip(bssids, colores_ap, alphas_comb):
    rm_masked = np.ma.masked_invalid(radiomaps[bssid])
    ax.imshow(rm_masked, cmap=cmap_nombre, origin='lower', extent=extent,
              alpha=alpha, vmin=-90, vmax=-35)

for bssid in bssids:
    ap_pos = {
        '02:00:00:00:00:01': (-3.0, 2.0),
        '02:00:00:00:00:02': ( 3.0, 1.0),
        '02:00:00:00:00:03': (-3.0,-2.0),
    }[bssid]
    num = bssid[-2:]
    ax.plot(*ap_pos, 'k*', markersize=12)
    ax.annotate(f'AP{int(num,16)}', ap_pos, textcoords='offset points',
                xytext=(4, 4), fontsize=8, fontweight='bold')

ax.plot(0, 0, 'go', markersize=8)
ax.set_title('Los 3 APs superpuestos\n(azul=dorm, rojo=salón, verde=cocina)', fontsize=9)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_xlim(ox, ox + W * res)
ax.set_ylim(oy, oy + H * res)

plt.tight_layout()
salida = MAPS_DIR / 'radiomaps_visualizacion.png'
plt.savefig(salida, dpi=150, bbox_inches='tight')
print(f'Guardado: {salida}')
plt.show()
