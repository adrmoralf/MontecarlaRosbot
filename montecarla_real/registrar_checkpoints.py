#!/usr/bin/env python3
"""
registrar_checkpoints.py — registra la pose AMCL cuando el robot está
estático en cada checkpoint físico marcado en el suelo.

Uso:
    source /opt/ros/humble/setup.bash
    python3 montecarla_real/registrar_checkpoints.py \\
        --checkpoints docker-compose/config/real/checkpoints.yaml \\
        --salida ~/real_bags/checkpoints_A.csv

Controles interactivos:
    <ID>  [Enter]  — registrar el checkpoint con ese ID (ej: C01)
    l     [Enter]  — listar checkpoints pendientes
    q     [Enter]  — guardar CSV y salir
"""

import argparse
import csv
import sys
import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class RegistradorCheckpoints(Node):

    def __init__(self, checkpoints: list[dict], n_promedio: int = 5):
        super().__init__('registrador_checkpoints')
        self._checkpoints = {c['id'].upper(): c for c in checkpoints}
        self._n_promedio = n_promedio  # número de poses a promediar por checkpoint
        self._buffer: list[tuple[float, float]] = []
        self._registros: list[dict] = []
        self._registrados: set[str] = set()

        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._cb_pose,
            10,
        )
        self.get_logger().info(
            f'Escuchando /amcl_pose — promediando {n_promedio} poses por checkpoint'
        )

    def _cb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._buffer.append((x, y))
        if len(self._buffer) > self._n_promedio * 3:
            self._buffer = self._buffer[-self._n_promedio * 3:]

    def registrar(self, cp_id: str) -> None:
        cp_id = cp_id.upper()
        if cp_id not in self._checkpoints:
            print(f'  ✗ ID desconocido: {cp_id}')
            print(f'    IDs válidos: {sorted(self._checkpoints.keys())}')
            return
        if not self._buffer:
            print('  ✗ Sin pose AMCL todavía — ¿está corriendo montecarla_amcl?')
            return

        # Usar las últimas N poses como promedio (robot debe estar inmóvil)
        ultimas = self._buffer[-self._n_promedio:]
        x_amcl = sum(p[0] for p in ultimas) / len(ultimas)
        y_amcl = sum(p[1] for p in ultimas) / len(ultimas)

        cp = self._checkpoints[cp_id]
        x_real = float(cp['x'])
        y_real = float(cp['y'])
        error = ((x_amcl - x_real) ** 2 + (y_amcl - y_real) ** 2) ** 0.5

        self._registros.append({
            'id':     cp_id,
            'zona':   cp.get('zona', '?'),
            'x_real': x_real,
            'y_real': y_real,
            'x_amcl': round(x_amcl, 4),
            'y_amcl': round(y_amcl, 4),
            'error_m': round(error, 4),
            'n_poses': len(ultimas),
        })
        self._registrados.add(cp_id)

        indicador = '✓' if error < 1.0 else ('⚠' if error < 2.0 else '✗')
        print(
            f'  {indicador} {cp_id} ({cp.get("zona","?"):12s}) '
            f'real=({x_real:+.2f},{y_real:+.2f})  '
            f'amcl=({x_amcl:+.2f},{y_amcl:+.2f})  '
            f'error={error:.3f}m'
        )

    def listar_pendientes(self) -> None:
        todos = sorted(self._checkpoints.keys())
        pendientes = [cp for cp in todos if cp not in self._registrados]
        if not pendientes:
            print('  Todos los checkpoints registrados.')
            return
        print(f'  Pendientes ({len(pendientes)}): {pendientes}')

    def resumen(self) -> None:
        if not self._registros:
            print('  Sin registros.')
            return
        import math
        errores = [r['error_m'] for r in self._registros]
        rmse = math.sqrt(sum(e**2 for e in errores) / len(errores))
        menos_1m = sum(1 for e in errores if e < 1.0)
        print(f'\n  ── Resumen ({len(errores)} checkpoints) ──')
        print(f'  RMSE global: {rmse:.3f} m')
        print(f'  < 1m: {menos_1m}/{len(errores)} ({100*menos_1m/len(errores):.0f}%)')
        # Por zona
        zonas: dict[str, list[float]] = {}
        for r in self._registros:
            zonas.setdefault(r['zona'], []).append(r['error_m'])
        for zona, es in sorted(zonas.items()):
            rmse_z = math.sqrt(sum(e**2 for e in es) / len(es))
            print(f'  {zona:12s}: RMSE={rmse_z:.3f}m  n={len(es)}')

    def guardar(self, ruta: str) -> None:
        if not self._registros:
            print('Sin registros — no se guarda nada.')
            return
        from pathlib import Path
        Path(ruta).parent.mkdir(parents=True, exist_ok=True)
        with open(ruta, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._registros[0].keys())
            writer.writeheader()
            writer.writerows(self._registros)
        print(f'\n  Guardado: {ruta}  ({len(self._registros)} checkpoints)')


def main() -> None:
    parser = argparse.ArgumentParser(description='Registra pose AMCL en checkpoints físicos')
    parser.add_argument('--checkpoints', required=True,
                        help='YAML con lista de checkpoints (id, x, y, zona)')
    parser.add_argument('--salida', required=True,
                        help='CSV de salida con errores por checkpoint')
    parser.add_argument('--n-promedio', type=int, default=5,
                        help='Número de poses a promediar por checkpoint (default: 5)')
    args = parser.parse_args()

    with open(args.checkpoints) as f:
        data = yaml.safe_load(f)
    checkpoints = data['checkpoints']

    rclpy.init()
    nodo = RegistradorCheckpoints(checkpoints, args.n_promedio)

    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()

    time.sleep(1.0)  # dejar que el suscriptor se registre
    print('\n' + '─' * 60)
    print('  Registrador de checkpoints — TFG Montecarla')
    print('─' * 60)
    print(f'  Checkpoints cargados: {sorted(nodo._checkpoints.keys())}')
    print('  Comandos: <ID> para registrar, l para listar, q para guardar y salir')
    print('─' * 60 + '\n')

    while True:
        try:
            entrada = input('>> ').strip()
        except (KeyboardInterrupt, EOFError):
            break

        if not entrada:
            continue
        if entrada.lower() == 'q':
            break
        if entrada.lower() == 'l':
            nodo.listar_pendientes()
            continue

        nodo.registrar(entrada)

    nodo.resumen()
    nodo.guardar(args.salida)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
