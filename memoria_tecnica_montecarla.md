# Memoria Técnica: Sistema de Localización WiFi Multi-Red con ROSbot 2R
## TFG "Montecarla" — Extensión del repositorio `rosbot-demo-wifi-heatmap`

**Autor:** Adrián Morales Alfonso  
**Tutor:** David Alejo Teissiere  
**Plataforma:** ROSbot 2R (Husarion) · ROS 2 Humble · Ubuntu 22.04 / WSL2  
**Repositorio base:** https://github.com/husarion/rosbot-demo-wifi-heatmap

---

## 1. Descripción General del Repositorio Base

El repositorio `rosbot-demo-wifi-heatmap` de Husarion implementa un sistema de **generación autónoma de mapas de calor de señal WiFi (RSSI)** para espacios interiores, usando un robot móvil ROSbot 2 PRO y un PC conectados mediante VPN peer-to-peer (Husarnet).

### ¿Qué hace el sistema original?

El flujo completo es el siguiente:

1. El robot mapea autónomamente el entorno mediante SLAM (SLAM Toolbox + LiDAR RPLIDAR A2).
2. A partir del mapa generado, el PC calcula un conjunto de waypoints de medición distribuidos por el espacio libre.
3. El robot recorre de forma autónoma cada waypoint usando Nav2 Waypoint Follower.
4. En cada waypoint, un **plugin personalizado de Nav2** mide la RSSI de la red WiFi activa durante 10 segundos y envía el dato al PC.
5. El PC acumula las mediciones `(x, y, RSSI)` y genera **dos heatmaps interpolados** en 2D (absoluto y relativo) usando `scipy.interpolate`.

El resultado es una imagen de calor sobre el plano del entorno que muestra la cobertura WiFi espacial.

### Limitación clave del sistema original

El sistema original solo mide la **red WiFi a la que el robot está conectado activamente**, leyendo `/proc/net/wireless`. Esto es suficiente para visualizar cobertura de un único AP, pero es insuficiente para fingerprinting WiFi multi-red orientado a localización.

---

## 2. Arquitectura del Sistema Original

El sistema se divide en dos entidades físicas con sus propios contenedores Docker:

```
┌─────────────────────────────────────────────────────────────────────┐
│                          ROSbot 2R                                   │
│                                                                      │
│  ┌──────────┐  ┌─────────────────────┐  ┌────────────┐  ┌────────┐ │
│  │ rplidar  │  │  nav2 (nav2-wifi-   │  │slam-toolbox│  │rosbot  │ │
│  │ (galactic│  │  heatmap container) │  │ (galactic) │  │(noetic)│ │
│  │  image)  │  │  - Nav2 stack       │  │            │  │        │ │
│  │          │  │  - Plugin RSSI      │  │  SLAM sync │  │STM32   │ │
│  └──────────┘  └─────────────────────┘  └────────────┘  └────────┘ │
│                         │                                            │
│                 ┌────────────────┐  ┌────────────┐                  │
│                 │  ros1_bridge   │  │ ros-master │                  │
│                 │  (galactic)    │  │ (noetic)   │                  │
│                 └────────────────┘  └────────────┘                  │
│                                                                      │
│  net_expose.sh → expone /proc/net/wireless al contenedor nav2       │
└─────────────────────────────────────────────────────────────────────┘
                              │ Husarnet VPN (P2P)
┌─────────────────────────────────────────────────────────────────────┐
│                              PC                                      │
│                                                                      │
│  ┌──────────────────────────┐     ┌──────────────────────────────┐  │
│  │   mappers container      │     │       rviz2 container        │  │
│  │  (mapper-packages)       │     │  - Visualización del mapa    │  │
│  │  - Generación waypoints  │     │  - Goal pose para SLAM       │  │
│  │  - Recepción datos RSSI  │     │  - Config: rosbot_pro_       │  │
│  │  - Interpolación 2D      │     │    mapping.rviz              │  │
│  │  - Generación heatmaps   │     └──────────────────────────────┘  │
│  └──────────────────────────┘                                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Nota sobre versiones ROS

El sistema original usa **ROS 2 Galactic** en el lado del PC y los contenedores principales, mientras que la capa de control del robot (rosbot + ros-master) corre sobre **ROS Noetic**, conectados mediante `ros1_bridge`. Esta arquitectura híbrida es consecuencia de la antigüedad del diseño original.

**En el TFG "Montecarla"**, la arquitectura se actualiza a **ROS 2 Humble** (LTS hasta 2027), eliminando la necesidad del bridge ROS1↔ROS2 ya que el ROSbot 2R dispone de soporte nativo ROS 2 Humble.

---

## 3. Estructura de Directorios del Repositorio

```
rosbot-demo-wifi-heatmap/
├── docker-compose/
│   ├── compose.rosbot.yaml          # Servicios Docker para el robot
│   ├── compose.rosbot.husarnet.yaml # Overlay: añade Husarnet al robot
│   ├── compose.pc.yaml              # Servicios Docker para el PC
│   ├── compose.pc.husarnet.yaml     # Overlay: añade Husarnet al PC
│   ├── config/
│   │   ├── nav2_params.yaml         # Parámetros Nav2 (costmap, planner, etc.)
│   │   ├── slam_params.yaml         # Parámetros SLAM Toolbox
│   │   └── rosbot_pro_mapping.rviz  # Config RViz para la fase de mapping
│   └── maps/                        # Directorio donde se guarda el mapa
├── nav2-wifi-heatmap/               # Paquete ROS 2 C++ (plugin Nav2)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   │   └── read_rssi_at_waypoint.cpp  # Plugin: mide RSSI en cada waypoint
│   └── plugins.xml                  # Declaración del plugin Nav2
├── mapper-packages/                 # Paquete ROS 2 Python (PC side)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── mapper_packages/
│       ├── waypoint_generator.py    # Genera waypoints desde el mapa SLAM
│       ├── rssi_receiver.py         # Recibe datos RSSI del robot vía ROS 2
│       └── heatmap_generator.py     # Interpolación 2D y generación de imagen
├── sample-images/                   # Ejemplos de resultado
│   ├── Figure_1.png                 # Heatmap relativo
│   └── Figure_2.png                 # Heatmap absoluto
├── net_expose.sh                    # Script: expone /proc/net/wireless al contenedor
├── generate-vpn-config.sh           # Script: genera secrets Husarnet
├── .gitignore
├── LICENSE                          # Apache 2.0
└── README.md
```

---

## 4. Componentes Clave — Análisis Técnico Detallado

### 4.1 Plugin Nav2: `nav2_read_rssi_at_waypoint` (C++)

Este componente es el corazón del sistema en el lado del robot. Es un **plugin de Nav2 Waypoint Follower** que se ejecuta cada vez que el robot llega a un waypoint.

**Interfaz de plugin Nav2:** implementa `nav2_core::WaypointTaskExecutor`, que expone el método `processAtWaypoint(geometry_msgs::msg::PoseStamped, int)`.

**Comportamiento:**
- Al llegar a un waypoint, el plugin abre el fichero expuesto por `net_expose.sh` (ruta montada en el contenedor como volumen).
- Lee el nivel RSSI de la interfaz WiFi activa desde `/proc/net/wireless` en formato texto.
- Acumula N lecturas durante ~10 segundos (configurable) y calcula la media.
- Publica el dato `(x, y, rssi_promedio)` en un topic ROS 2, que el nodo del PC recibe vía Husarnet VPN.

**Registro del plugin en Nav2 (`nav2_params.yaml`):**
```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_read_rssi_at_waypoint::ReadRSSIAtWaypoint"
      enabled: true
      timeout: 10.0
```

**Exposición de `/proc/net/wireless` al contenedor (`net_expose.sh`):**
```bash
#!/bin/bash
# Expone los datos de red inalámbrica del kernel al sistema de archivos
# en ~/net_expose, que se monta como volumen en el contenedor nav2
mkdir -p ~/net_expose
while true; do
    cat /proc/net/wireless > ~/net_expose/wireless
    sleep 1
done
```

El contenedor monta este directorio:
```yaml
nav2:
  volumes:
    - ~/net_expose:/net_expose
```

**Errores comunes:**
- Si `net_expose.sh` no está corriendo, el plugin no encuentra el fichero y lanza excepción. Síntoma: el robot llega al waypoint pero no publica datos RSSI.
- Si la interfaz WiFi no está en `/proc/net/wireless` (el robot no está conectado a ninguna red), el fichero estará vacío. Solución: verificar con `iwconfig` o `ip link`.

### 4.2 Script `net_expose.sh`

Este script resuelve un problema de seguridad de los contenedores Docker: por defecto, un contenedor no tiene acceso al sistema de archivos del host, incluyendo `/proc/net/wireless`. En lugar de ejecutar el contenedor en modo privilegiado (mala práctica), se copia periódicamente el contenido a un directorio montado como volumen.

**Ejecución en el ROSbot:**
```bash
chmod +x rosbot-demo-wifi-heatmap/net_expose.sh
rosbot-demo-wifi-heatmap/net_expose.sh &   # Lanzar en background
```

### 4.3 Paquete `mapper-packages` (Python, PC side)

Tres nodos/scripts Python que corren en el contenedor del PC:

**`waypoint_generator.py`:** Lee el mapa OGM (Occupancy Grid Map) guardado por SLAM Toolbox. Aplica filtrado para descartar celdas próximas a obstáculos o zonas desconocidas (ocupación > umbral). Distribuye waypoints de medición sobre las celdas libres con una resolución configurable (ej. cada 50 cm). Publica los waypoints descartados en rojo y los válidos en verde sobre el mapa en RViz.

**`rssi_receiver.py`:** Nodo ROS 2 subscrito al topic de datos RSSI publicado por el plugin del robot. Acumula las mediciones `(x, y, rssi)` en memoria. Guarda los datos crudos en un fichero (CSV o pickle) para post-procesado.

**`heatmap_generator.py`:** Toma las mediciones acumuladas y aplica interpolación 2D mediante `scipy.interpolate.griddata` (por defecto interpolación cúbica). Genera dos imágenes con `matplotlib`:
- **Heatmap absoluto:** colormap sobre valores de RSSI en dBm.
- **Heatmap relativo:** normalizado entre el mínimo y máximo medidos.

Las imágenes se guardan en `/heatmaps` (montado sobre `~/heatmaps` en el host).

**Comando de ejecución:**
```bash
docker exec docker-compose-mappers-1 /run.sh
```

---

## 5. Configuración de Red: Husarnet VPN

Husarnet es una VPN peer-to-peer basada en WireGuard que permite comunicar el ROSbot y el PC sin depender de una red local compartida. La comunicación ROS 2 (DDS) fluye sobre esta VPN.

### Configuración paso a paso

**1. Crear cuenta y red en Husarnet:**
```
https://app.husarnet.com/
→ Crear nueva red
→ [Add element] → pestaña Join Code
→ Copiar: JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxx...
```

**2. Fichero `.env` en `docker-compose/`:**
```bash
JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

**3. Generar configuración DDS:**
```bash
./generate-vpn-config.sh
# Genera el fichero 'secrets' → copiar a ambos dispositivos
```

**Nota para el TFG:** En el contexto del TFG "Montecarla", la dependencia de Husarnet puede simplificarse si robot y PC están en la misma red LAN. En ese caso, basta con configurar correctamente las variables `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION` para que el DDS descubra los nodos sin VPN. **Requisito:** el PC debe correr Ubuntu 22.04 nativo — WSL2 no es válido porque su red virtual impide el descubrimiento DDS con el ROSbot (ver sesión 2026-06-04 en diario.md).

---

## 6. Flujo de Uso Completo (Sistema Original)

### Fase 1: Preparación

```bash
# En ROSbot
git clone https://github.com/husarion/rosbot-demo-wifi-heatmap.git
chmod +x rosbot-demo-wifi-heatmap/net_expose.sh
rosbot-demo-wifi-heatmap/net_expose.sh &

# En PC
git clone https://github.com/husarion/rosbot-demo-wifi-heatmap.git
xhost local:root   # Permitir X11 forwarding para RViz
```

### Fase 2: Lanzamiento de contenedores

```bash
# En ROSbot
cd rosbot-demo-wifi-heatmap/docker-compose
docker compose -f compose.rosbot.yaml -f compose.rosbot.husarnet.yaml up

# En PC (terminal separada)
cd rosbot-demo-wifi-heatmap/docker-compose
docker compose -f compose.pc.yaml -f compose.pc.husarnet.yaml up
```

### Fase 3: Mapping del entorno (manual)

1. En RViz, usar **2D Goal Pose** para enviar waypoints al robot.
2. El robot navega hacia cada objetivo mientras SLAM Toolbox construye el mapa.
3. Consideraciones importantes:
   - Evitar zonas con obstáculos transparentes (no detectados por LiDAR).
   - Cubrir todo el espacio libre para maximizar los puntos de medición.
   - El mapa se guarda automáticamente en `docker-compose/maps/`.

### Fase 4: Medición autónoma de RSSI

```bash
# En PC, una vez completado el mapa
docker exec docker-compose-mappers-1 /run.sh
```

El sistema muestra el mapa con waypoints marcados en verde (válidos) y rojo (descartados por proximidad a obstáculos). El robot inicia el recorrido autónomo, midiendo RSSI 10 segundos en cada waypoint verde.

### Fase 5: Generación y guardado de heatmaps

Al completar todos los waypoints, se generan automáticamente las dos imágenes de heatmap. Las imágenes crudas se guardan en `/heatmaps`. Para las figuras completas con barra de color, usar el botón **SAVE** en la GUI de matplotlib y guardar en `~/heatmaps/`.

---

## 7. El Proyecto TFG "Montecarla": Extensión Multi-Red

### 7.1 Motivación y Diferencia respecto al Repositorio Base

El sistema de Husarion genera **un único heatmap** para la red activa. El objetivo del TFG es extender esto a **N redes WiFi simultáneas**, produciendo:

- N heatmaps individuales (uno por SSID detectado), para visualización.
- Un **mapa de fingerprinting vectorial** `{(x,y): {SSID_1: RSSI_1, SSID_2: RSSI_2, ..., SSID_N: RSSI_N}}`, como base de datos de referencia para localización.
- Integración de este fingerprint en `nav2_amcl` como fuente de observación adicional al LiDAR (método "Montecarla").

### 7.2 Cambio en la Lectura RSSI: de Red Activa a Escaneo Multi-Red

**Sistema original:** Lee `/proc/net/wireless` → solo red activa.

**Sistema Montecarla:** Usa `nmcli` para escanear todas las redes visibles sin necesidad de estar conectado a ninguna:

```bash
nmcli -t -f SSID,BSSID,SIGNAL device wifi list
```

Ejemplo de salida:
```
MiRed_5G:AA:BB:CC:DD:EE:FF:-45
MiRed_2G:AA:BB:CC:DD:EE:F0:-62
RedVecino:11:22:33:44:55:66:-78
```

Este comando no requiere conexión activa, solo que la interfaz WiFi (`wlan0`) esté levantada. La latencia es de ~2-3 segundos por scan, lo que es compatible con la pausa de 10 segundos en cada waypoint.

**Cambio en el contenedor Docker:** En lugar de montar `/proc/net/wireless` vía `net_expose.sh`, se añade capacidad `NET_ADMIN` al contenedor para ejecutar `nmcli` directamente:

```yaml
nav2:
  cap_add:
    - NET_ADMIN
  network_mode: host   # Necesario para ver las interfaces de red del host
```

**Advertencia:** `network_mode: host` implica que el contenedor comparte la pila de red del host. Esto es aceptable en el ROSbot (dispositivo dedicado) pero debe tenerse en cuenta si se ejecuta en un PC compartido.

### 7.3 Modificación del Plugin C++: `read_multi_rssi_at_waypoint`

El plugin original lee un único valor RSSI. La nueva versión ejecuta `nmcli` mediante `subprocess` (o `popen` en C++), parsea la salida y construye un mapa `{SSID → RSSI_promedio}` acumulando N scans durante 10 segundos.

**Estructura del mensaje publicado (custom ROS 2 message):**

```
# wifi_msgs/msg/WifiScan.msg
std_msgs/Header header
geometry_msgs/Point position
string[] ssids
float32[] rssi_values
```

El plugin publica este mensaje en el topic `/wifi_scan_at_waypoint`.

### 7.4 Modificación del Mapper Python: Generación Multi-Heatmap

El `heatmap_generator.py` se extiende para:

1. **Agrupar mediciones por SSID:** Construye un diccionario `{SSID: [(x, y, rssi), ...]}` a partir de los mensajes recibidos.
2. **Generar N heatmaps:** Para cada SSID con suficientes puntos de medición (umbral mínimo configurable), aplica `scipy.interpolate.griddata` y genera una imagen.
3. **Generar el fingerprint vectorial:** Exporta un fichero `fingerprint.pkl` (pickle de Python) o `fingerprint.csv` con la estructura `{(x,y): {SSID: RSSI}}`.

**Estructura del fichero fingerprint (CSV):**
```csv
x,y,SSID_1,SSID_2,SSID_3,...
1.5,2.0,-45.2,-62.1,-78.4,...
1.5,2.5,-43.8,-61.5,-80.2,...
...
```

### 7.5 Integración MCL: Modelo de Observación WiFi en nav2_amcl (Nivel 2)

La fase de localización extiende `nav2_amcl` añadiendo el RSSI WiFi como fuente de observación complementaria al LiDAR.

**Principio de funcionamiento:**

En el filtro de partículas estándar, el peso de cada partícula se actualiza según la verosimilitud del scan LiDAR:

```
w_i = p(z_lidar | x_i, mapa)
```

En "Montecarla", se añade el término WiFi:

```
w_i = p(z_lidar | x_i, mapa) · p(z_wifi | x_i, fingerprint)
```

La función de verosimilitud WiFi `p(z_wifi | x_i, fingerprint)` se modela típicamente como una gaussiana sobre la diferencia RSSI:

```
p(z_wifi | x_i) = ∏_k exp(-(rssi_k_medido - rssi_k_fingerprint(x_i))² / (2·σ²))
```

donde `σ` es la desviación estándar del modelo de ruido WiFi (parámetro empírico, típicamente 5-10 dBm para entornos interiores).

**Estrategia de implementación sobre nav2_amcl:**

`nav2_amcl` en ROS 2 Humble expone sus modelos de sensor mediante la clase abstracta `nav2_amcl::SensorModel`. La integración se realiza mediante un **plugin adicional de modelo de sensor** que:

1. Se subscribe al topic `/wifi_scan` (scan RSSI en tiempo real).
2. Carga el fichero `fingerprint.pkl` al inicializar.
3. Para cada partícula, interpola el vector RSSI esperado en su posición usando el fingerprint.
4. Calcula el peso WiFi y lo multiplica al peso existente de la partícula.

**Parámetro de confianza adaptativo:** En zonas con pocas redes visibles o RSSI muy ruidoso, se puede reducir el peso del término WiFi dinámicamente para evitar degradar la localización LiDAR.

### 7.6 Validación: Simulación + Robot Real

**Fase de simulación (Gazebo):**

Para validar el modelo de observación WiFi sin hardware real, se implementa un **nodo simulador de RSSI** que:
- Conoce la posición real del robot (ground truth de Gazebo).
- Publica un vector RSSI sintético calculado con el modelo de propagación de path loss: `RSSI(d) = RSSI_ref - 10·n·log10(d/d_ref) + ε`, donde `ε ~ N(0, σ²)` simula el ruido.
- Permite evaluar el error de localización (distancia euclídea pose estimada vs. ground truth) en escenarios con ambigüedad geométrica (pasillos largos).

**Métrica de evaluación:**
- Error de localización medio (ATE — Absolute Trajectory Error).
- Comparación: AMCL puro vs. AMCL+WiFi (Montecarla).
- Escenario de prueba: pasillo simétrico donde el LiDAR produce scans idénticos en múltiples posiciones.

**Fase con robot real (ROSbot 2R):**
- Entorno con múltiples APs WiFi distribuidos (mínimo 3 para triangulación robusta).
- Se genera el fingerprint real con el sistema de heatmap multi-red.
- Se valida la convergencia del filtro de partículas en zona de ambigüedad geométrica.
- Ground truth: marcas físicas en el suelo medidas con cinta métrica.

---

## 8. Instalación y Puesta en Marcha (Sistema Montecarla)

### 8.1 Requisitos previos

> **Importante:** el PC debe correr **Ubuntu 22.04 nativo** (no WSL2). WSL2 tiene su propia interfaz de red virtual (`172.x.x.x`) distinta a la IP del host Windows en la LAN. Eso impide que el DDS de ROS 2 descubra los nodos del ROSbot, que está en la red física. Se comprobó en sesión 2026-06-04: con WSL2, `ros2 topic list` no mostraba ningún topic del robot.

```bash
# En el PC de desarrollo (Ubuntu 22.04 nativo — no WSL2)
# ROS 2 Humble instalado
source /opt/ros/humble/setup.bash

# Docker y Docker Compose
sudo apt install docker.io docker-compose-plugin
sudo usermod -aG docker $USER

# nmcli (normalmente preinstalado en Ubuntu)
sudo apt install network-manager
```

### 8.2 Clonado y configuración

```bash
git clone https://github.com/TU_USUARIO/montecarla_ros2.git
cd montecarla_ros2

# Crear fichero de variables de entorno
cp docker-compose/.env.example docker-compose/.env
# Editar .env: añadir JOINCODE si se usa Husarnet, o configurar ROS_DOMAIN_ID para LAN
```

### 8.3 Lanzamiento en ROSbot 2R

```bash
# SSH al robot
ssh husarion@<IP_ROSBOT>

# Lanzar stack completo
cd montecarla_ros2/docker-compose
docker compose -f compose.rosbot.yaml up

# Verificar que los topics están activos
ros2 topic list
# Esperado: /scan, /odom, /tf, /wifi_scan_at_waypoint
```

### 8.4 Lanzamiento en PC

```bash
# Habilitar X11 para RViz
xhost local:root

cd montecarla_ros2/docker-compose
docker compose -f compose.pc.yaml up

# Verificar conectividad con el robot
ros2 topic echo /wifi_scan_at_waypoint
```

### 8.5 Generación del fingerprint

```bash
# Una vez completado el mapping y la medición autónoma
docker exec docker-compose-mappers-1 /run.sh

# El fingerprint se guarda en ~/heatmaps/fingerprint.csv
# Los heatmaps individuales en ~/heatmaps/<SSID>_heatmap.png
```

### 8.6 Localización con Montecarla

```bash
# Lanzar AMCL extendido con el modelo WiFi
ros2 launch montecarla_ros2 localization_launch.py \
  fingerprint_file:=~/heatmaps/fingerprint.csv \
  wifi_sigma:=8.0 \
  use_wifi:=true
```

---

## 9. Errores Comunes y Soluciones

| Error | Causa probable | Solución |
|-------|----------------|----------|
| Plugin no encuentra fichero RSSI | `net_expose.sh` no está en ejecución | `ps aux | grep net_expose` y relanzar si es necesario |
| `nmcli` no devuelve redes | Interfaz WiFi apagada o en modo monitor | `ip link set wlan0 up` |
| Contenedor sin acceso a `nmcli` | Falta `cap_add: NET_ADMIN` | Añadir al `compose.rosbot.yaml` y reiniciar |
| Robot no recibe waypoints | Topic `/navigate_through_poses` no activo | Verificar que Nav2 está en estado ACTIVE con `ros2 lifecycle list` |
| Heatmap con interpolación errónea | Pocos puntos de medición | Reducir resolución de waypoints o ampliar zona de mapping |
| AMCL no converge con WiFi | `σ` WiFi demasiado bajo | Aumentar `wifi_sigma` (empezar con 10.0 dBm) |
| DDS no descubre nodos entre robot y PC | `ROS_DOMAIN_ID` distinto o firewall | `export ROS_DOMAIN_ID=0` en ambos lados; abrir puertos UDP 7400-7500 |

---

## 10. Tecnologías y Dependencias

| Componente | Tecnología | Versión |
|------------|-----------|---------|
| Middleware robótico | ROS 2 Humble Hawksbill | LTS hasta 2027 |
| Navegación autónoma | Nav2 (Navigation2) | Humble |
| SLAM | SLAM Toolbox | Humble |
| Sensor LiDAR | RPLIDAR A2M8 | rplidar_ros |
| Contenedores | Docker + Docker Compose | 24.x |
| Escaneo WiFi | nmcli (NetworkManager CLI) | ≥1.36 |
| Interpolación 2D | scipy.interpolate.griddata | ≥1.9 |
| Visualización | matplotlib | ≥3.6 |
| Fingerprint | numpy, pandas | ≥1.23 |
| Lenguajes | C++ (plugin Nav2), Python 3.10 (mapper) | — |
| VPN (opcional) | Husarnet | ≥2.0 |
| Simulación | Gazebo (Ignition Fortress) | ROS 2 Humble |

---

## 11. Resumen de Contribuciones Originales del TFG

| Componente | Estado en Husarion | Contribución Montecarla |
|------------|-------------------|------------------------|
| Plugin Nav2 RSSI (C++) | Lee 1 red activa vía `/proc/net/wireless` | Lee N redes vía `nmcli`, publica mensaje vectorial |
| Mapper Python (PC) | Genera 2 heatmaps de 1 red | Genera N heatmaps + fingerprint vectorial CSV/pickle |
| Mensaje ROS 2 | `std_msgs/Float32` con RSSI único | Custom `wifi_msgs/WifiScan` con arrays SSID+RSSI |
| Modelo de observación MCL | No implementado | Función de verosimilitud gaussiana sobre RSSI multi-red |
| Integración AMCL | No implementado | Plugin `nav2_amcl` extendido con canal WiFi |
| Validación | No documentada | Simulación Gazebo (RSSI sintético) + ROSbot 2R real |
| Arquitectura ROS | ROS2 Galactic + ROS1 bridge | ROS 2 Humble puro |

---

*Documento generado como referencia técnica para el desarrollo del TFG "Montecarla".*  
*Universidad de Sevilla — Escuela Técnica Superior de Ingeniería — Dpto. Ingeniería de Sistemas y Automática*
