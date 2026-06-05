# Diario técnico — Sesión 2026-06-04

## Problema resuelto

### 1. Comunicación ROS 2 entre ROSbot y PC fallaba (WSL2 tiene IP separada del host Windows)

**Síntoma:** Al intentar conectar ROSbot y PC por IP en la misma LAN, los nodos ROS 2 del PC no descubrían los del robot y viceversa. El `ros2 topic list` en el PC no mostraba los topics del ROSbot.

**Causa raíz:** El entorno de desarrollo del PC era **WSL2 (Windows Subsystem for Linux)**. WSL2 crea una interfaz de red virtual con su propia subred interna (típicamente `172.x.x.x`), distinta a la IP real de la máquina Windows en la LAN. Desde el ROSbot, la IP visible era la de Windows; desde WSL2, la IP era otra. El tráfico DDS (UDP multicast/unicast) no cruzaba correctamente entre la red de WSL2 y la LAN física.

**Diagnóstico:**
```bash
# Dentro de WSL2
ip addr show eth0
# → 172.x.x.x  ← IP virtual de WSL2, no visible desde el ROSbot

# En Windows (PowerShell)
ipconfig
# → 192.168.x.x  ← IP real en la LAN, la que ve el ROSbot
```

El ROSbot intentaba comunicarse con `192.168.x.x` (Windows), pero ROS 2 corría en `172.x.x.x` (WSL2). El DDS no podía establecer la sesión.

**Solución adoptada:** Instalar **Ubuntu 22.04 nativo** (dual boot) en el PC, eliminando la capa de virtualización de WSL2. Con Ubuntu nativo, la interfaz de red del PC tiene directamente la IP de la LAN, igual que el ROSbot, y el descubrimiento DDS funciona sin configuración adicional.

---

## Estado al final de la sesión

```
WSL2 como entorno de desarrollo    ✗  descartado (IP virtual incompatible con DDS)
Ubuntu 22.04 nativo instalado      ✓  dual boot en el PC
Comunicación ROSbot ↔ PC por LAN   pendiente de verificar en Ubuntu nativo
```

---

# Diario técnico — Sesión 2026-05-31

## Problemas resueltos

### 1. stm32flash fallaba con "Failed to init device"

**Síntoma:** Todos los intentos de flashear el STM32F4 devolvían `Failed to init device`
independientemente de cómo se configurara el GPIO (sysfs, RPi.GPIO, gpiod en Docker).

**Causa raíz:** El snap `rosbot` de Husarion (`/snap/rosbot/223`) tenía un servicio
(`rosbot.daemon`) corriendo `micro_ros_agent` directamente en el sistema, fuera de Docker.
Ese proceso bloqueaba `/dev/ttyAMA0` de forma permanente.

**Diagnóstico:**
```bash
sudo fuser /dev/ttyAMA0          # devolvió PID 2839
cat /proc/2839/cmdline | tr '\0' ' '
# → /snap/rosbot/223/opt/ros/snap/lib/micro_ros_agent/micro_ros_agent serial -b 576000 -D /dev/ttyAMA0
```

**Solución:**
```bash
sudo snap stop --disable rosbot
```

---

### 2. micro_ros_agent ciclaba cada ~4s (firmware incompatible)

**Síntoma:** El agente mostraba `delete_client` / `create_client` con `client_key` diferente
cada 4 segundos. El `controller_manager` nunca arrancaba, el frame `odom` no existía.

**Causa raíz:** El STM32 llevaba un firmware antiguo incompatible con la versión del
Docker image `husarion/rosbot:humble-0.13.1-20240201`.

**Secuencia de flash** (con snap parado y UART libre):

Hardware ROSbot 2R CM4:
- `/dev/ttyAMA0` = PL011 UART (soporta 8E1)
- GPIO17 = BOOT0 (HIGH → modo bootloader)
- GPIO18 = RESET (HIGH via transistor NPN → NRST low → STM32 en reset)

GPIOs ya exportados via sysfs de intentos anteriores:
```bash
# Extraer firmware del contenedor Docker
docker run --rm husarion/rosbot:humble-0.13.1-20240201 cat /root/firmware.bin > /tmp/firmware.bin

# Entrar en modo bootloader
echo 1 > /sys/class/gpio/gpio17/value   # BOOT0 HIGH
echo 1 > /sys/class/gpio/gpio18/value   # RESET assert
sleep 0.2
echo 0 > /sys/class/gpio/gpio18/value   # RESET release → STM32 arranca en bootloader
sleep 0.5

# Flashear (STM32F40xxx/41xxx detectado: 0x0413)
sudo stm32flash -b 115200 -w /tmp/firmware.bin /dev/ttyAMA0

# Salir del bootloader
echo 0 > /sys/class/gpio/gpio17/value   # BOOT0 LOW
echo 1 > /sys/class/gpio/gpio18/value
sleep 0.1
echo 0 > /sys/class/gpio/gpio18/value
```

**Resultado:** Firmware flasheado al 100% (`0x0803bbe8`). STM32F40xxx/41xxx,
firmware 0.11.0 del Docker image.

---

### 3. RPLidar fallaba con OPERATION_TIMEOUT

**Síntoma:** `sllidar_node` moría con `SL_RESULT_OPERATION_TIMEOUT` a los 2-3s de arrancar.
El motor del lidar giraba físicamente pero no respondía por serial.

**Diagnóstico:**
```bash
# Test Python a 115200 → 0 bytes
# Test Python a 256000 → 91 bytes de respuesta:
# "RP LIDAR System. Firmware Ver 1.32, HW Ver 6, Model: 2c"
```

El lidar es un **RPLIDAR modelo 0x2C** que requiere **256000 baud**, no 115200.
El chip USB-serial es un CP210x (Silicon Labs).

**Causa raíz en compose:** `serial_baudrate:=115200` en el servicio `rplidar`.

**Solución** en `docker-compose/compose.rosbot.yaml`:
```yaml
command: >
  ros2 launch sllidar_ros2 sllidar_launch.py
    serial_port:=/dev/ttyUSB0
    serial_baudrate:=256000   # era 115200
```

**Nota importante:** `docker compose restart` NO recrea el contenedor con nuevos parámetros.
Usar `docker compose up -d rplidar` para que tome el nuevo baudrate.

---

## Estado final del stack

Con el snap deshabilitado y Docker compose arriba:

```
STM32 firmware          ✓  0.11.0 flasheado
micro_ros_agent         ✓  sesión estable, no cicla
controller_manager      ✓  update rate 20 Hz
hardware 'wheels'       ✓  activated
hardware 'imu'          ✓  activated
imu_broadcaster         ✓  configured and activated
joint_state_broadcaster ✓  configured and activated
rosbot_base_controller  ✓  configured and activated
RPLidar                 ✓  Sensitivity mode, 16 KHz, 16m, 10 Hz
slam_toolbox            ✓  Registering sensor: Custom Described Lidar
nav2                    ✓  Managed nodes are active
frame odom              ✓  publicándose
frame map               ✓  generándose por slam-toolbox
```

## Configuración LAN (sin Husarnet)

Ambos compose files (`compose.rosbot.yaml` y `compose.pc.yaml`) tienen:
```yaml
network_mode: host
ipc: host
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  - DDS_CONFIG=DEFAULT
```

El snap `rosbot` permanece **deshabilitado** en el sistema para evitar conflictos con Docker.
