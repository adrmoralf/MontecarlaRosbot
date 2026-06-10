#!/usr/bin/env python3
"""Visualiza los radiomaps generados superpuestos sobre el mapa SLAM."""

import numpy as np
import yaml
import matplotlib.pyplot as plt
from pathlib import Path
from PIL import Image
from scipy.ndimage import gaussian_filter

MAPS_DIR = Path(__file__).parent

# ── Cargar metadata ────────────────────────────────────────────────────────────
with open(MAPS_DIR / 'radiomap_meta.yaml') as f:
    meta = yaml.safe_load(f)

res = meta['resolution']
ox  = meta['origin'][0]
oy  = meta['origin'][1]
W   = meta['width']
H   = meta['height']
extent = [ox, ox + W * res, oy, oy + H * res]

# ── Cargar mapa SLAM (PGM) ─────────────────────────────────────────────────────
pgm = np.array(Image.open(MAPS_DIR / 'casa_simple.pgm')).astype(float)

# ── Cargar radiomaps ───────────────────────────────────────────────────────────
bssids = meta['bssids']
radiomaps = {}
for bssid in bssids:
    nombre = 'radiomap_' + bssid.replace(':', '_') + '.npy'
    radiomaps[bssid] = np.load(MAPS_DIR / nombre)

# ── Interpolación gaussiana para visualizar el gradiente ──────────────────────
def suavizar_radiomap(rm, sigma=40):
    """
    Suaviza el radiomap disperso con un filtro gaussiano ponderado.
    sigma en celdas (sigma=40 → radio ~1.6m, cubre toda una habitación).
    """
    valido = (~np.isnan(rm)).astype(float)
    valores = np.where(np.isnan(rm), 0.0, rm)

    peso_suav  = gaussian_filter(valido,  sigma=sigma)
    valor_suav = gaussian_filter(valores, sigma=sigma)

    rm_out = np.full_like(rm, np.nan)
    mascara = peso_suav > 0.001
    rm_out[mascara] = valor_suav[mascara] / peso_suav[mascara]
    return rm_out


# ── Figura principal: un subplot por AP ───────────────────────────────────────
titulos = {
    '02:00:00:00:00:01': 'AP1 · Dormitorio  (-3, 2)',
    '02:00:00:00:00:02': 'AP2 · Salón        ( 3, 1)',
    '02:00:00:00:00:03': 'AP3 · Cocina       (-3,-2)',
}
colores = ['Blues_r', 'Reds_r', 'Greens_r']
posiciones_ap = {
    '02:00:00:00:00:01': (-3.0,  2.0),
    '02:00:00:00:00:02': ( 3.0,  1.0),
    '02:00:00:00:00:03': (-3.0, -2.0),
}

fig, axes = plt.subplots(1, 4, figsize=(22, 6))
fig.suptitle('Radiomaps Montecarla — casa_simple (10×7 m²)\n'
             '(gradiente interpolado con suavizado gaussiano σ=48 cm)',
             fontsize=12, fontweight='bold')

vmin, vmax = -90, -35

for idx, (bssid, cmap) in enumerate(zip(bssids, colores)):
    ax = axes[idx]
    rm_suav = suavizar_radiomap(radiomaps[bssid], sigma=12)

    # Fondo: mapa SLAM
    ax.imshow(pgm, cmap='gray', origin='lower', extent=extent, alpha=0.6)

    # Radiomap interpolado (sólo donde hay datos suficientes)
    rm_masked = np.ma.masked_invalid(rm_suav)
    im = ax.imshow(rm_masked, cmap=cmap, origin='lower', extent=extent,
                   alpha=0.85, vmin=vmin, vmax=vmax)

    # Posición del AP y spawn
    ax.plot(*posiciones_ap[bssid], 'k*', markersize=14, zorder=5)
    ax.annotate('AP', posiciones_ap[bssid],
                textcoords='offset points', xytext=(6, 4), fontsize=9, fontweight='bold')
    ax.plot(0, 0, 'go', markersize=9, zorder=5, label='Spawn (0,0)')

    plt.colorbar(im, ax=ax, label='RSSI (dBm)', fraction=0.046, pad=0.04)
    ax.set_title(titulos.get(bssid, bssid), fontsize=10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(ox, ox + W * res)
    ax.set_ylim(oy, oy + H * res)
    ax.grid(True, alpha=0.2)

# ── 4º subplot: diferencia AP1 − AP3 (firma dormitorio vs cocina) ─────────────
ax = axes[3]
rm1 = suavizar_radiomap(radiomaps['02:00:00:00:00:01'], sigma=12)
rm3 = suavizar_radiomap(radiomaps['02:00:00:00:00:03'], sigma=12)
diff = rm1 - rm3
diff_masked = np.ma.masked_invalid(diff)

ax.imshow(pgm, cmap='gray', origin='lower', extent=extent, alpha=0.6)
im_diff = ax.imshow(diff_masked, cmap='RdBu', origin='lower', extent=extent,
                    alpha=0.8, vmin=-30, vmax=30)
for bssid in bssids:
    ax.plot(*posiciones_ap[bssid], 'k*', markersize=12, zorder=5)
ax.plot(0, 0, 'go', markersize=9, zorder=5)
plt.colorbar(im_diff, ax=ax, label='RSSI AP1 − AP3 (dBm)', fraction=0.046, pad=0.04)
ax.set_title('Diferencia AP1 − AP3\n(rojo = más AP1, azul = más AP3)', fontsize=10)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_xlim(ox, ox + W * res)
ax.set_ylim(oy, oy + H * res)
ax.grid(True, alpha=0.2)

plt.tight_layout()
salida = MAPS_DIR / 'radiomaps_visualizacion.png'
plt.savefig(salida, dpi=150, bbox_inches='tight')
print(f'Guardado: {salida}')
plt.show()
