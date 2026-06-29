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

# ── Cargar mapa SLAM (PGM — usa map_image del meta si existe) ─────────────────
if 'map_image' in meta and (MAPS_DIR / meta['map_image']).exists():
    pgm_path = MAPS_DIR / meta['map_image']
else:
    for pgm_name in ['casa_real.pgm', 'casa_simple.pgm']:
        pgm_path = MAPS_DIR / pgm_name
        if pgm_path.exists():
            break
pgm_name = pgm_path.name
pgm = np.array(Image.open(pgm_path)).astype(float)

# ── Cargar radiomaps ───────────────────────────────────────────────────────────
bssids = meta['bssids']
radiomaps = {}
for bssid in bssids:
    nombre = 'radiomap_' + bssid.replace(':', '_') + '.npy'
    radiomaps[bssid] = np.load(MAPS_DIR / nombre)

# ── Máscara de espacio libre (interior del mapa PGM) ──────────────────────────
# nav2/slam_toolbox: 254=libre, 205=desconocido/exterior, 0=ocupado (paredes)
# El PGM tiene row 0 = top de imagen = y_max del mapa → flipud para alinear con
# los arrays del radiomap donde fila 0 = y_min (origen del mapa)
interior_mask = np.flipud(pgm) > 200

# ── Máscara de trayectoria (solo zona accesible por el robot) ──────────────────
traj_mask = None
traj_path = MAPS_DIR / 'radiomap_trayectoria.npy'
if traj_path.exists():
    traj_mask = np.load(traj_path)

# Aplicar ambas máscaras a los radiomaps
for bssid in bssids:
    if traj_mask is not None:
        radiomaps[bssid][~traj_mask] = np.nan
    radiomaps[bssid][~interior_mask] = np.nan

# ── Interpolación gaussiana para visualizar el gradiente ──────────────────────
def suavizar_radiomap(rm, sigma=40):
    valido = (~np.isnan(rm)).astype(float)
    valores = np.where(np.isnan(rm), 0.0, rm)
    peso_suav  = gaussian_filter(valido,  sigma=sigma)
    valor_suav = gaussian_filter(valores, sigma=sigma)
    rm_out = np.full_like(rm, np.nan)
    mascara = peso_suav > 0.001
    rm_out[mascara] = valor_suav[mascara] / peso_suav[mascara]
    return rm_out


# ── Figura principal ───────────────────────────────────────────────────────────
colores = ['Blues_r', 'Reds_r', 'Greens_r', 'Purples_r']
n_aps = len(bssids)
n_cols = n_aps + 1  # +1 para la diferencia

fig, axes = plt.subplots(1, n_cols, figsize=(6 * n_cols, 6))
if n_cols == 1:
    axes = [axes]
fig.suptitle('Radiomaps Montecarla\n'
             '(gradiente interpolado con suavizado gaussiano)',
             fontsize=12, fontweight='bold')

vmin = min(np.nanmin(radiomaps[b]) for b in bssids)
vmax = max(np.nanmax(radiomaps[b]) for b in bssids)

for idx, (bssid, cmap) in enumerate(zip(bssids, colores)):
    ax = axes[idx]
    rm_suav = suavizar_radiomap(radiomaps[bssid], sigma=12)

    ax.imshow(pgm, cmap='gray', origin='lower', extent=extent, alpha=0.6)
    rm_masked = np.ma.masked_invalid(rm_suav)
    im = ax.imshow(rm_masked, cmap=cmap, origin='lower', extent=extent,
                   alpha=0.85, vmin=vmin, vmax=vmax)
    ax.plot(0, 0, 'go', markersize=9, zorder=5, label='Spawn (0,0)')
    plt.colorbar(im, ax=ax, label='RSSI (dBm)', fraction=0.046, pad=0.04)
    ax.set_title(f'AP {idx + 1}', fontsize=9)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(ox, ox + W * res)
    ax.set_ylim(oy, oy + H * res)
    ax.grid(True, alpha=0.2)

# ── Último subplot: diferencia AP0 − AP1 ──────────────────────────────────────
ax = axes[n_aps]
rm0 = suavizar_radiomap(radiomaps[bssids[0]], sigma=12)
rm1 = suavizar_radiomap(radiomaps[bssids[1]], sigma=12) if len(bssids) > 1 else rm0
diff = rm0 - rm1
diff_masked = np.ma.masked_invalid(diff)
ax.imshow(pgm, cmap='gray', origin='lower', extent=extent, alpha=0.6)
im_diff = ax.imshow(diff_masked, cmap='RdBu', origin='lower', extent=extent,
                    alpha=0.8, vmin=-30, vmax=30)
ax.plot(0, 0, 'go', markersize=9, zorder=5)
plt.colorbar(im_diff, ax=ax, label='RSSI AP0 − AP1 (dBm)', fraction=0.046, pad=0.04)
ax.set_title('Diferencia\nAP 1 − AP 2', fontsize=9)
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
