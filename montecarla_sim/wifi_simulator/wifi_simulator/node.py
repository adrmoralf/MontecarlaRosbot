"""
wifi_simulator_node — simula medidas WiFi para el experimento Montecarla.

Modelo RSSI:
    RSSI = RSSI_ref - 10·n·log10(d/d_ref) - W·atten_pared + N(0, sigma_sim)
    con d = max(d, d_ref)  [antes del log]
         W = paredes raycast sobre OccupancyGrid (Bresenham 2D)

Publica: /wifi_scan  (montecarla_msgs/WifiScan)
         header.stamp = ahora - scan_duration/2
         (blur temporal: promedia RSSI sobre buffer_poses de scan_duration s)

Suscribe: /ground_truth (nav_msgs/Odometry)  — NUNCA pose estimada del filtro
          /map          (nav_msgs/OccupancyGrid)
"""

import math
import yaml
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from montecarla_msgs.msg import WifiScan, WifiMeasurement

from .wall_raycast import count_walls


class WifiSimulatorNode(Node):

    def __init__(self):
        super().__init__('wifi_simulator')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('aps_file', '/aps.yaml')
        self.declare_parameter('publish_rate', 0.5)   # Hz

        fichero_aps = self.get_parameter('aps_file').get_parameter_value().string_value
        with open(fichero_aps) as f:
            config = yaml.safe_load(f)

        self._aps          = config['aps']
        self._atten_pared  = float(config.get('atten_pared', 5.0))
        self._sigma_sim    = float(config.get('sigma_sim',   6.0))
        self._dur_scan     = float(config.get('scan_duration', 2.0))

        # Normalizar BSSIDs a mayúsculas (contrato del mensaje)
        for ap in self._aps:
            ap['bssid'] = ap['bssid'].strip().upper()

        # ── Estado ────────────────────────────────────────────────────────────
        self._buffer_poses = deque()   # (t_seg, x, y)
        self._datos_mapa   = None
        self._info_mapa    = None
        self._rng          = np.random.default_rng()

        # ── QoS para /map (map_server publica con TRANSIENT_LOCAL) ───────────
        qos_mapa = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Suscripciones ────────────────────────────────────────────────────
        self.create_subscription(Odometry,       '/ground_truth', self._on_ground_truth, 50)
        self.create_subscription(OccupancyGrid,  '/map',          self._on_map,          qos_mapa)

        # ── Publicador ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(WifiScan, '/wifi_scan', 10)

        # ── Timer de publicación ─────────────────────────────────────────────
        frecuencia = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.create_timer(1.0 / frecuencia, self._publish)

        self.get_logger().info(
            f'wifi_simulator listo: {len(self._aps)} APs, '
            f'scan_dur={self._dur_scan}s, sigma={self._sigma_sim}dBm'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_ground_truth(self, msg: Odometry):
        t_seg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._buffer_poses.append((t_seg, x, y))

        # Eliminar poses más antiguas que scan_duration
        limite = t_seg - self._dur_scan
        while self._buffer_poses and self._buffer_poses[0][0] < limite:
            self._buffer_poses.popleft()

    def _on_map(self, msg: OccupancyGrid):
        self._info_mapa = msg.info
        self._datos_mapa = list(msg.data)
        self.get_logger().info(
            f'Mapa recibido: {msg.info.width}×{msg.info.height} celdas '
            f'@ {msg.info.resolution}m/celda'
        )

    # ── Publicación ───────────────────────────────────────────────────────────

    def _publish(self):
        if not self._buffer_poses:
            return
        if self._datos_mapa is None:
            self.get_logger().warn('Sin mapa aún — WiFi scan omitido', throttle_duration_sec=5.0)
            return

        ahora_seg = (self.get_clock().now().nanoseconds) * 1e-9
        historial = list(self._buffer_poses)   # snapshot para no iterar sobre deque que cambia

        msg = WifiScan()
        # stamp = centro del scan
        t_centro_ns = int((ahora_seg - self._dur_scan / 2.0) * 1e9)
        msg.header.stamp.sec     = t_centro_ns // 10**9
        msg.header.stamp.nanosec = t_centro_ns %  10**9
        msg.header.frame_id      = 'base_link'
        msg.scan_duration        = float(self._dur_scan)

        for ap in self._aps:
            pos_ap = (float(ap['position'][0]), float(ap['position'][1]))
            rssi_ref = float(ap['rssi_ref'])
            d_ref    = float(ap['d_ref'])
            n        = float(ap['n'])

            valores_rssi = []
            for (_, px, py) in historial:
                d = math.sqrt((px - pos_ap[0])**2 + (py - pos_ap[1])**2)
                d = max(d, d_ref)   # clamp ANTES del log

                W = count_walls(
                    (px, py), pos_ap,
                    self._datos_mapa, self._info_mapa
                )

                rssi = (rssi_ref
                        - 10.0 * n * math.log10(d / d_ref)
                        - W * self._atten_pared
                        + self._rng.normal(0.0, self._sigma_sim))

                rssi = float(np.clip(rssi, -100.0, -30.0))
                valores_rssi.append(rssi)

            rssi_medio = float(np.mean(valores_rssi))

            medicion = WifiMeasurement()
            medicion.bssid     = ap['bssid']
            medicion.ssid      = ap.get('ssid', '')
            medicion.rssi      = rssi_medio
            medicion.frequency = float(ap.get('frequency', 2.4e9))
            msg.measurements.append(medicion)

        self._pub.publish(msg)

        # Log para validación (comprobar firmas R1/R2 y W∈{0,1,2})
        pose_actual = historial[-1]
        texto_rssi = '  '.join(
            f"{m.bssid[-2:]}:{m.rssi:.1f}dBm" for m in msg.measurements
        )
        self.get_logger().debug(
            f'pos=({pose_actual[1]:.1f},{pose_actual[2]:.1f})  {texto_rssi}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WifiSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
