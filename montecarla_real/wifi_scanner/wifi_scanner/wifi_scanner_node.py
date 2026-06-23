"""
wifi_scanner_node — publica medidas WiFi reales desde wlan0.

Método primario:  iw dev <iface> scan dump
  — lee la caché de APs del kernel (nl80211), SIN scan activo.
  — NO interrumpe el tráfico de datos.
  — Requiere CAP_NET_ADMIN en Docker (--cap-add NET_ADMIN).
  — La caché se renueva automáticamente por NetworkManager (~2 min).
  — Al arranque, si la caché está vacía, hace UN scan activo para poblarla.

Método fallback: nmcli (si iw falla por permisos)
  — usa el demonio NetworkManager vía D-Bus.
  — RSSI: porcentaje NM convertido a dBm (resolución 0.5 dBm, suficiente).
  — Requiere montar /var/run/dbus en el contenedor.
  — Avisa en log que se está usando el fallback.

Publica: /wifi_scan (montecarla_msgs/WifiScan)
  — scan_duration = 0.0 (medida instantánea, no promedio temporal)
  — header.stamp = ahora (wall clock, igual que wifi_simulator en sim)
  — BSSID en mayúsculas

Parámetros ROS:
  interfaz           (str)   — interfaz WiFi, default "wlan0"
  frecuencia_hz      (float) — frecuencia de publicación, default 0.5
  bssids_permitidos  (list)  — lista de BSSIDs a incluir; vacío = todos
"""

import time
import rclpy
from rclpy.node import Node

from montecarla_msgs.msg import WifiScan, WifiMeasurement

from .iw_parser import scan_dump, scan_activo, scan_nmcli, AP


class WifiScannerNode(Node):

    def __init__(self):
        super().__init__('wifi_scanner')

        self.interfaz = self.declare_parameter('interfaz', 'wlan0').value
        frecuencia    = self.declare_parameter('frecuencia_hz', 0.5).value
        bssids_param  = self.declare_parameter('bssids_permitidos', []).value

        self._bssids_permitidos: set[str] = {b.upper() for b in (bssids_param or [])}
        self._backend: str = 'iw'   # 'iw' o 'nmcli'
        self._cache_poblada: bool = False

        self._pub = self.create_publisher(WifiScan, '/wifi_scan', 10)
        self.create_timer(1.0 / frecuencia, self._publicar)

        self.get_logger().info(
            f'wifi_scanner iniciado — interfaz={self.interfaz}  '
            f'frecuencia={frecuencia}Hz  '
            f'filtro_bssid={self._bssids_permitidos or "todos"}'
        )

        # Timer de arranque: verifica backend una sola vez tras 0.5s
        self._timer_arranque = self.create_timer(0.5, self._verificar_backend_una_vez)

    # ── Arranque: verificar backend ───────────────────────────────────────────

    def _verificar_backend_una_vez(self):
        """
        Se ejecuta una sola vez tras 0.5s del arranque.
        1) Prueba iw scan dump.
        2) Si la caché está vacía, hace UN scan activo para poblarla.
        3) Si iw falla por permisos → switch a nmcli.
        """
        self._timer_arranque.cancel()
        self.destroy_timer(self._timer_arranque)

        aps, error = scan_dump(self.interfaz)

        if error and 'permission' in error.lower():
            self.get_logger().warn(
                f'iw scan dump falló (permisos insuficientes: "{error}"). '
                f'Usando fallback nmcli. '
                f'Para usar iw, añadir "cap_add: [NET_ADMIN]" al servicio Docker.'
            )
            self._backend = 'nmcli'
            return

        if error:
            self.get_logger().warn(
                f'iw scan dump error: "{error}". Reintentando en cada ciclo.'
            )
            return

        if not aps:
            self.get_logger().info(
                'Caché de APs vacía al arranque. '
                'Realizando scan activo inicial (puede interrumpir ~100ms de tráfico)...'
            )
            _, err_activo = scan_activo(self.interfaz)
            if err_activo:
                self.get_logger().warn(f'Scan activo falló: "{err_activo}"')
            else:
                self.get_logger().info('Scan activo completado. La caché se renovará cada ~2 min.')
            self._cache_poblada = True
        else:
            self.get_logger().info(
                f'Backend iw operativo. '
                f'{len(aps)} APs en caché al arranque.'
            )
            self._cache_poblada = True

    # ── Ciclo de publicación ──────────────────────────────────────────────────

    def _publicar(self):
        aps, error = self._leer_aps()

        if error:
            self.get_logger().warn(
                f'[{self._backend}] error leyendo APs: "{error}"',
                throttle_duration_sec=30.0,
            )
            return

        if not aps:
            self.get_logger().debug(
                'Caché vacía — NetworkManager aún no ha hecho background scan.',
                throttle_duration_sec=10.0,
            )
            return

        # Filtrar por BSSID si se especificó lista
        if self._bssids_permitidos:
            aps = [a for a in aps if a.bssid in self._bssids_permitidos]
            if not aps:
                self.get_logger().debug(
                    'Ningún AP de la lista permitida visible.',
                    throttle_duration_sec=10.0,
                )
                return

        msg = self._construir_mensaje(aps)
        self._pub.publish(msg)

        texto = '  '.join(f'{a.bssid[-5:]}:{a.rssi_dbm:.1f}dBm' for a in aps)
        self.get_logger().debug(f'[{self._backend}] {len(aps)} APs → {texto}')

    def _leer_aps(self) -> tuple[list[AP], str]:
        if self._backend == 'iw':
            return scan_dump(self.interfaz)
        return scan_nmcli(self.interfaz)

    def _construir_mensaje(self, aps: list[AP]) -> WifiScan:
        msg = WifiScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.scan_duration   = 0.0  # medida instantánea (sin promedio temporal)

        for ap in aps:
            m = WifiMeasurement()
            m.bssid     = ap.bssid
            m.rssi      = float(ap.rssi_dbm)
            m.ssid      = ap.ssid
            m.frequency = ap.freq_hz
            msg.measurements.append(m)

        return msg


def main(args=None):
    rclpy.init(args=args)
    nodo = WifiScannerNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
