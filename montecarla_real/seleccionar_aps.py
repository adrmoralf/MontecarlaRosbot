#!/usr/bin/env python3
"""
seleccionar_aps.py — analiza el bag del survey y elige los N APs con mayor
poder discriminatorio entre zonas.

Un AP discriminador es el que tiene RSSI muy diferente entre habitaciones
(max diferencia de medias) y estable dentro de cada habitación (std bajo).
Métrica: discriminabilidad = max_diff_medias / (std_medio + 0.1)

Uso:
    source /opt/ros/humble/setup.bash
    python3 montecarla_real/seleccionar_aps.py \\
        --bag  ~/real_bags/survey_YYYYMMDD_HHMMSS \\
        --mapa docker-compose/maps/casa_real.yaml  \\
        --n 3 \\
        --salida docker-compose/config/real/

Salida:
    aps_seleccionados.yaml   — BSSIDs elegidos con perfil RSSI por zona
    aps_analisis.png         — gráfica de discriminabilidad + perfil RSSI

IMPORTANTE: editar la función zona() con los límites X del mapa real
antes de ejecutar. Ver instrucciones al final del fichero.
"""

import argparse
import sys
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    MATPLOTLIB = True
except ImportError:
    MATPLOTLIB = False

import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ── Función de zona — EDITAR según el mapa real ──────────────────────────────
#
# Abre RViz con el mapa real y anota las coordenadas X de cada zona.
# Ejemplo con casa_simple (simulación):
#   Salón:      x >  1.0  (pared derecha del pasillo)
#   Pasillo:   -1.0 <= x <= 1.0
#   Habitaciones: x < -1.0  (pared izquierda del pasillo)
#
# Para discriminar dormitorio vs cocina, añadir eje Y:
#   Dormitorio: x < -1.0 AND y > 0.0
#   Cocina:     x < -1.0 AND y < 0.0

def zona(x: float, y: float) -> str:
    """
    Clasifica la pose (x, y) en una zona de la casa.
    EDITAR estos umbrales según el mapa real.
    """
    if x > 1.0:
        return 'salon'
    if x < -1.0:
        if y > 0.0:
            return 'dormitorio'
        return 'cocina'
    return 'pasillo'


# ── Lectura del bag ───────────────────────────────────────────────────────────

def leer_bag(ruta: str):
    """
    Lee /wifi_scan y /odometry/filtered del bag.
    Retorna:
      wifi_msgs: [(t_bag_ns, msg)]
      odom_msgs: [(t_bag_ns, x, y)]
    """
    lector = rosbag2_py.SequentialReader()
    lector.open(
        rosbag2_py.StorageOptions(uri=str(ruta), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}
    wifi_msgs, odom_msgs = [], []

    while lector.has_next():
        topic, datos, t_bag_ns = lector.read_next()
        if topic == '/wifi_scan' and topic in tipos:
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            wifi_msgs.append((t_bag_ns, msg))
        elif topic == '/odometry/filtered' and topic in tipos:
            tipo = get_message(tipos[topic])
            msg  = deserialize_message(datos, tipo)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odom_msgs.append((t_bag_ns, x, y))

    wifi_msgs.sort(key=lambda w: w[0])
    odom_msgs.sort(key=lambda o: o[0])
    return wifi_msgs, odom_msgs


def emparejar_pose(wifi_msgs, odom_msgs, ventana_ns=2_000_000_000):
    """
    Empareja cada wifi_scan con la pose de odometría más cercana en tiempo.
    Descarta pares cuya diferencia temporal supera ventana_ns.
    """
    if not odom_msgs:
        return []
    ts_odom = np.array([o[0] for o in odom_msgs])
    pares   = []
    for t_wifi, msg in wifi_msgs:
        idx = int(np.searchsorted(ts_odom, t_wifi))
        # Elegir el más cercano entre idx-1 e idx
        if idx == 0:
            mejor = 0
        elif idx >= len(odom_msgs):
            mejor = len(odom_msgs) - 1
        else:
            d_ant = abs(ts_odom[idx-1] - t_wifi)
            d_sig = abs(ts_odom[idx]   - t_wifi)
            mejor = idx - 1 if d_ant <= d_sig else idx
        if abs(ts_odom[mejor] - t_wifi) > ventana_ns:
            continue
        x, y = odom_msgs[mejor][1], odom_msgs[mejor][2]
        pares.append((x, y, msg))
    return pares


# ── Análisis discriminabilidad ────────────────────────────────────────────────

def analizar(pares: list) -> list[dict]:
    """
    Por cada AP visible en el bag, calcula:
      - RSSI medio y std por zona
      - discriminabilidad: max(diff_medias) / std_medio
    """
    datos: dict[str, dict[str, list]] = defaultdict(lambda: defaultdict(list))
    bssid_ssid: dict[str, str] = {}

    for x, y, msg in pares:
        z = zona(x, y)
        for m in msg.measurements:
            datos[m.bssid][z].append(m.rssi)
            bssid_ssid[m.bssid] = getattr(m, 'ssid', '')

    resultados = []
    for bssid, por_zona in datos.items():
        # Solo considerar zonas con al menos 3 medidas (estadísticamente válido)
        zonas_validas = {z: v for z, v in por_zona.items() if len(v) >= 3}
        if len(zonas_validas) < 2:
            continue  # AP solo visible en una zona → no discrimina

        medias = {z: float(np.mean(v)) for z, v in zonas_validas.items()}
        stds   = {z: float(np.std(v))  for z, v in zonas_validas.items()}
        std_medio = float(np.mean(list(stds.values()))) if stds else 10.0

        # Max diferencia de medias entre cualquier par de zonas
        vals = list(medias.values())
        max_dif = max(
            abs(vals[i] - vals[j])
            for i in range(len(vals))
            for j in range(i + 1, len(vals))
        )
        discriminabilidad = max_dif / (std_medio + 0.1)

        n_total = sum(len(v) for v in por_zona.values())
        resultados.append({
            'bssid':              bssid,
            'ssid':               bssid_ssid.get(bssid, ''),
            'discriminabilidad':  discriminabilidad,
            'max_dif_dbm':        max_dif,
            'std_medio':          std_medio,
            'n_muestras':         n_total,
            'medias_por_zona':    medias,
            'stds_por_zona':      stds,
        })

    resultados.sort(key=lambda r: r['discriminabilidad'], reverse=True)
    return resultados


# ── Guardado de resultados ─────────────────────────────────────────────────────

def guardar_yaml(seleccionados: list[dict], ruta_salida: Path):
    datos = {
        'aps_seleccionados': [
            {
                'bssid':           r['bssid'],
                'ssid':            r['ssid'],
                'discriminabilidad': round(r['discriminabilidad'], 2),
                'max_dif_dbm':     round(r['max_dif_dbm'], 1),
                'std_medio_dbm':   round(r['std_medio'], 1),
                'n_muestras':      r['n_muestras'],
                'rssi_medio_por_zona': {
                    z: round(v, 1) for z, v in r['medias_por_zona'].items()
                },
            }
            for r in seleccionados
        ]
    }
    ruta = ruta_salida / 'aps_seleccionados.yaml'
    with open(ruta, 'w') as f:
        yaml.dump(datos, f, allow_unicode=True, sort_keys=False)
    print(f'  → {ruta}')
    return ruta


def graficar(todos: list[dict], seleccionados: list[dict], ruta_salida: Path):
    if not MATPLOTLIB:
        print('  matplotlib no disponible — gráfica omitida')
        return

    zonas_orden = sorted({
        z for r in todos for z in r['medias_por_zona'].keys()
    })

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ── Gráfica 1: discriminabilidad de todos los APs ─────────────────────
    ax = axes[0]
    n_mostrar  = min(15, len(todos))
    nombres    = [f"{r['ssid'][:12] or r['bssid'][-8:]}" for r in todos[:n_mostrar]]
    discs      = [r['discriminabilidad'] for r in todos[:n_mostrar]]
    bssids_sel = {r['bssid'] for r in seleccionados}
    colores    = ['#2166ac' if r['bssid'] in bssids_sel else '#aaaaaa'
                  for r in todos[:n_mostrar]]

    ax.barh(range(n_mostrar), discs, color=colores)
    ax.set_yticks(range(n_mostrar))
    ax.set_yticklabels(nombres, fontsize=8)
    ax.set_xlabel('Discriminabilidad (Δ_medias / σ_medio)')
    ax.set_title(f'Top {n_mostrar} APs — azul = seleccionados')
    ax.invert_yaxis()
    ax.grid(True, alpha=0.3, axis='x')

    # ── Gráfica 2: perfil RSSI por zona de los APs seleccionados ──────────
    ax = axes[1]
    x_pos = np.arange(len(zonas_orden))
    ancho = 0.8 / max(len(seleccionados), 1)

    for i, r in enumerate(seleccionados):
        medias = [r['medias_por_zona'].get(z, np.nan) for z in zonas_orden]
        stds   = [r['stds_por_zona'].get(z, 0.0)     for z in zonas_orden]
        offset = x_pos + i * ancho - ancho * (len(seleccionados) - 1) / 2
        label  = f"{r['ssid'][:10] or r['bssid'][-8:]}"
        ax.bar(offset, medias, ancho * 0.9, yerr=stds,
               label=label, capsize=3, alpha=0.85)

    ax.set_xticks(x_pos)
    ax.set_xticklabels(zonas_orden)
    ax.set_ylabel('RSSI medio ± std (dBm)')
    ax.set_title('Perfil RSSI de APs seleccionados por zona')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    ruta = ruta_salida / 'aps_analisis.png'
    plt.savefig(ruta, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  → {ruta}')


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Selecciona los N APs más discriminadores del bag de survey'
    )
    parser.add_argument('--bag',    required=True, help='Directorio del bag')
    parser.add_argument('--n',      type=int, default=3,
                        help='Número de APs a seleccionar (default 3)')
    parser.add_argument('--salida', default='docker-compose/config/real/',
                        help='Directorio de salida para YAML y PNG')
    args = parser.parse_args()

    salida = Path(args.salida)
    salida.mkdir(parents=True, exist_ok=True)

    print(f'[1/4] Leyendo bag: {args.bag}')
    rclpy.init()
    wifi_msgs, odom_msgs = leer_bag(args.bag)
    rclpy.shutdown()
    print(f'      {len(wifi_msgs)} wifi_scan  |  {len(odom_msgs)} odometry')

    if not wifi_msgs:
        print('ERROR: no hay mensajes /wifi_scan en el bag.', file=sys.stderr)
        print('  ¿Estaba wifi-scanner corriendo durante el survey?', file=sys.stderr)
        sys.exit(1)

    print('[2/4] Emparejando poses...')
    pares = emparejar_pose(wifi_msgs, odom_msgs)
    print(f'      {len(pares)} pares válidos (de {len(wifi_msgs)} scans)')

    if not pares:
        print('ERROR: no se pudieron emparejar poses con scans WiFi.', file=sys.stderr)
        sys.exit(1)

    print('[3/4] Analizando discriminabilidad por zona...')
    todos = analizar(pares)

    if not todos:
        print('ERROR: ningún AP tiene datos suficientes (≥3 muestras en ≥2 zonas).', file=sys.stderr)
        print('  Posibles causas:', file=sys.stderr)
        print('  — El robot no recorrió todas las zonas durante el survey.', file=sys.stderr)
        print('  — Los umbrales de zona() no coinciden con el mapa real.', file=sys.stderr)
        print('  — Ejecutar: python3 montecarla_real/seleccionar_aps.py --help', file=sys.stderr)
        sys.exit(1)

    print(f'\n  Top {min(10, len(todos))} APs por discriminabilidad:')
    print(f"  {'BSSID':19} {'SSID':18} {'Disc':6} {'ΔdBm':7} {'σ':5} {'n':5}")
    print(f"  {'-'*65}")
    for r in todos[:10]:
        print(f"  {r['bssid']:19} {r['ssid'][:18]:18} "
              f"{r['discriminabilidad']:6.1f} {r['max_dif_dbm']:7.1f} "
              f"{r['std_medio']:5.1f} {r['n_muestras']:5}")

    n_real = min(args.n, len(todos))
    seleccionados = todos[:n_real]
    print(f'\n  Seleccionados ({n_real}):')
    for r in seleccionados:
        zonas_str = ', '.join(
            f"{z}:{v:.1f}dBm" for z, v in r['medias_por_zona'].items()
        )
        print(f"  ✓ {r['bssid']}  {r['ssid']}  [{zonas_str}]")

    print(f'\n[4/4] Guardando resultados en {salida}:')
    guardar_yaml(seleccionados, salida)
    graficar(todos, seleccionados, salida)

    # Verificación: ¿los APs seleccionados discriminan bien?
    print('\n  Verificación de calidad:')
    for r in seleccionados:
        disc = r['discriminabilidad']
        if disc >= 3.0:
            print(f"  ✓ {r['bssid'][-8:]} disc={disc:.1f} — BUENO")
        elif disc >= 1.5:
            print(f"  ~ {r['bssid'][-8:]} disc={disc:.1f} — ACEPTABLE")
        else:
            print(f"  ✗ {r['bssid'][-8:]} disc={disc:.1f} — DÉBIL (WiFi no ayudará mucho)")

    print('\n¡Hecho! Siguientes pasos:')
    print('  1. Ver aps_analisis.png para confirmar que los perfiles RSSI son distintos')
    print('  2. Añadir los BSSIDs al parámetro bssids_permitidos del wifi-scanner')
    print('  3. Reconstruir radiomap con: BAG_NAME=survey_XXXX docker compose ...')


if __name__ == '__main__':
    main()
