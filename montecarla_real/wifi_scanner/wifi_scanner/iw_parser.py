"""
iw_parser.py — parsea la salida de 'iw dev <iface> scan [dump]'.

Formato de entrada (fragmento real):
    BSS aa:bb:cc:dd:ee:ff(on wlan0) -- associated
        freq: 2412
        signal: -52.00 dBm
        SSID: MiRed
    BSS 11:22:33:44:55:66(on wlan0)
        freq: 5180
        signal: -78.00 dBm
        SSID: OtraRed

Devuelve lista de AP namedtuples con:
    bssid    str   — mayúsculas, formato XX:XX:XX:XX:XX:XX
    rssi_dbm float — dBm exacto (−100 a −20), sin redondear
    ssid     str   — puede estar vacío (redes ocultas) o ser no-UTF8 → ''
    freq_hz  float — Hz (2.4/5 GHz)
"""

import re
import subprocess
from dataclasses import dataclass, field
from typing import Optional

_RE_BSS  = re.compile(r'^BSS\s+([\dA-Fa-f:]{17})', re.MULTILINE)
_RE_FREQ = re.compile(r'^\s+freq:\s+(\d+(?:\.\d+)?)', re.MULTILINE)
_RE_SIG  = re.compile(r'^\s+signal:\s+([-\d.]+)\s+dBm', re.MULTILINE)
_RE_SSID = re.compile(r'^\s+SSID:\s*(.*)', re.MULTILINE)


@dataclass
class AP:
    bssid:    str
    rssi_dbm: float
    ssid:     str   = ''
    freq_hz:  float = 0.0


def parse_iw_output(text: str) -> list[AP]:
    """
    Parsea el texto completo de 'iw scan' o 'iw scan dump'.
    Descarta APs sin campo 'signal' (no debería ocurrir pero por robustez).
    """
    aps: list[AP] = []
    # Dividir en bloques: cada bloque empieza con "BSS XX:XX:..."
    bloques = re.split(r'(?=^BSS\s)', text, flags=re.MULTILINE)

    for bloque in bloques:
        m_bss = _RE_BSS.search(bloque)
        if not m_bss:
            continue
        m_sig = _RE_SIG.search(bloque)
        if not m_sig:
            continue  # sin RSSI → ignorar

        bssid    = m_bss.group(1).upper()
        rssi_dbm = float(m_sig.group(1))

        freq_hz = 0.0
        if m_freq := _RE_FREQ.search(bloque):
            freq_hz = float(m_freq.group(1)) * 1e6  # MHz → Hz

        ssid = ''
        if m_ssid := _RE_SSID.search(bloque):
            raw = m_ssid.group(1).strip()
            # iw puede mostrar SSID como bytes hexadecimales si no es UTF-8:
            # SSID: \xc3\xa9... → decodificar o dejar vacío
            try:
                ssid = raw.encode('latin-1').decode('utf-8')
            except (UnicodeDecodeError, UnicodeEncodeError):
                ssid = raw  # usar como está

        aps.append(AP(bssid=bssid, rssi_dbm=rssi_dbm, ssid=ssid, freq_hz=freq_hz))

    return aps


def scan_dump(interfaz: str = 'wlan0') -> tuple[list[AP], str]:
    """
    Ejecuta 'iw dev <interfaz> scan dump' y devuelve (lista_APs, error_msg).
    scan dump lee la caché del kernel: NO hace un scan activo, NO interrumpe tráfico.
    Requiere CAP_NET_ADMIN.
    """
    resultado = subprocess.run(
        ['iw', 'dev', interfaz, 'scan', 'dump'],
        capture_output=True, text=True, timeout=5,
    )
    if resultado.returncode != 0:
        return [], resultado.stderr.strip()
    return parse_iw_output(resultado.stdout), ''


def scan_activo(interfaz: str = 'wlan0') -> tuple[list[AP], str]:
    """
    Ejecuta 'iw dev <interfaz> scan' (activo).
    Popula la caché del kernel. Puede interrumpir ~100ms de tráfico WiFi.
    Solo se llama UNA VEZ al arranque si scan_dump devuelve lista vacía.
    Requiere CAP_NET_ADMIN.
    """
    resultado = subprocess.run(
        ['iw', 'dev', interfaz, 'scan'],
        capture_output=True, text=True, timeout=15,
    )
    if resultado.returncode != 0:
        return [], resultado.stderr.strip()
    return parse_iw_output(resultado.stdout), ''


def scan_nmcli(interfaz: str = 'wlan0') -> tuple[list[AP], str]:
    """
    Fallback: usa 'nmcli' via D-Bus (sin CAP_NET_ADMIN).
    RSSI: convierte porcentaje NM (0-100) a dBm con la fórmula oficial de NM:
        dBm = quality / 2 - 100   (NM usa 2*(rssi+100) para -100≤rssi≤-50)
    Resolución: 0.5 dBm/unidad de porcentaje (suficiente para sigma=8dBm real).
    Necesita: /var/run/dbus montado en el contenedor.
    """
    # -t: modo terse (sin cabeceras, separador ':')
    # --escape no: no escapar ':' en valores (simplifica parseo)
    # -f BSSID,SIGNAL,SSID: campos que necesitamos
    resultado = subprocess.run(
        ['nmcli', '-t', '--escape', 'no', '-f', 'BSSID,SIGNAL,SSID',
         'dev', 'wifi', 'list', 'ifname', interfaz],
        capture_output=True, text=True, timeout=10,
    )
    if resultado.returncode != 0:
        return [], resultado.stderr.strip()

    aps: list[AP] = []
    for linea in resultado.stdout.splitlines():
        linea = linea.strip()
        if not linea:
            continue
        partes = linea.split(':')
        # BSSID siempre ocupa los 6 primeros campos (XX:XX:XX:XX:XX:XX)
        if len(partes) < 7:
            continue
        bssid = ':'.join(partes[:6]).upper()
        if not _RE_BSS.match(f'BSS {bssid}'):
            continue
        try:
            signal_pct = int(partes[6])
        except ValueError:
            continue
        # Fórmula NM: quality = 2*(rssi+100) para -100≤rssi≤-50
        # Inversa: rssi = quality/2 - 100
        rssi_dbm = float(signal_pct) / 2.0 - 100.0
        ssid = ':'.join(partes[7:]) if len(partes) > 7 else ''
        aps.append(AP(bssid=bssid, rssi_dbm=rssi_dbm, ssid=ssid, freq_hz=0.0))

    return aps, ''
