# Búsqueda Extensa: Proyectos de Localización WiFi por Mapas de Calor
## Sin dependencia de Husarnet — Referencia para TFG "Montecarla"

---

## Resumen Ejecutivo

La localización interior basada en RSSI WiFi es un campo activo tanto en la comunidad open-source como en la investigación académica. Ningún proyecto encontrado combina exactamente las tres características del TFG Montecarla: **(1) multi-red simultánea, (2) heatmap por SSID generado autónomamente con Nav2 Waypoint Follower, y (3) fusión con filtro de partículas AMCL en ROS 2 Humble sin VPN externa**. Esto confirma la originalidad del TFG y proporciona bibliografía comparativa sólida.

---

## CATEGORÍA 1: Proyectos ROS / ROS 2 con WiFi y Heatmap (más cercanos al tuyo)

---

### 1.1 `ros-wifi-localization` — Miyagusuku et al. (Universidad de Tokyo)
**Repositorio:** https://github.com/RMiyagusuku/ros-wifi-localization  
**Estado:** ROS 1 (Kinetic/Melodic), última actualización 2019  
**Relevancia para el TFG:** ⭐⭐⭐⭐⭐ (el más relevante encontrado)

**Qué hace:**
- Implementa un paquete ROS completo para **localización basada en señal WiFi** con múltiples APs simultáneos.
- Incluye el subpaquete `WLRF` (WiFi and LiDAR Range Fusion): un **AMCL modificado** que fusiona datos WiFi + LiDAR mediante filtro de partículas. Esto es exactamente el Nivel 2 del TFG Montecarla.
- El mapa WiFi se construye usando **Procesos Gaussianos** (GPy) para interpolación espacial — más sofisticado que `scipy.interpolate.griddata`.
- Publica el mapa RSSI como un `nav_msgs/OccupancyGrid` overlay sobre el mapa LiDAR, permitiendo visualización directa en RViz.

**Diferencias respecto a Montecarla:**
- Solo ROS 1, no hay port a ROS 2.
- No usa Nav2 Waypoint Follower; la recogida de datos es semi-manual.
- No genera heatmaps visuales (imágenes PNG/matplotlib), solo la estructura de datos.
- Un único proceso de mapeo WiFi por ejecución (no multi-SSID simultáneo).

**Publicaciones asociadas:**
- Miyagusuku et al., "Data Information Fusion from Multiple Access Points for WiFi-based Self-localization", *IEEE Robotics and Automation Letters*, Vol. 4, No. 2, 2019. DOI: 10.1109/LRA.2018.2885583
- Miyagusuku et al., "Precise and accurate wireless signal strength mappings using Gaussian processes and path loss models", *Robotics and Autonomous Systems*, 2018. DOI: 10.1016/j.robot.2018.02.011

**Valor para la memoria del TFG:** Citar como trabajo relacionado principal — misma filosofía de fusión WiFi+LiDAR en filtro de partículas, publicado en revista de alto impacto.

---

### 1.2 `WiFi_localization` — sldai (GitHub)
**Repositorio:** https://github.com/sldai/WiFi_localization  
**Estado:** Python puro, sin dependencia ROS, 2019  
**Relevancia para el TFG:** ⭐⭐⭐⭐

**Qué hace:**
- Sistema completo de **mapeo autónomo de fingerprint WiFi con robot** + localización posterior.
- Fase de mapping: robot recorre el entorno, smartphone montado recoge RSSI de todos los APs visibles. Construye un mapa Gaussiano `{posición → vector RSSI}`.
- Fase de localización: filtro de partículas que fusiona el fingerprint WiFi con un detector de pasos y brújula (IMU del smartphone).
- Arquitectura conceptualmente idéntica a Montecarla: mapa offline → localización online con filtro de partículas.

**Diferencias:**
- Implementación en Python puro para smartphone, no en ROS/robot.
- No usa LiDAR ni Nav2.
- No genera heatmaps visuales.

**Valor para la memoria:** Referencia directa para el diseño del modelo de observación WiFi del filtro de partículas.

---

### 1.3 `ROS_WiFi_Scanner` — macTracyHuang (GitHub)
**Repositorio:** https://github.com/macTracyHuang/ROS_WiFi_Scanner  
**Estado:** ROS 1, 2020  
**Relevancia para el TFG:** ⭐⭐⭐

**Qué hace:**
- Nodo ROS que escanea todos los APs visibles (no solo la red activa) usando `iw` en lugar de `iwlist`.
- Publica RSSI de múltiples redes en un topic ROS.
- Motivado por el bug de `iwlist` con muchos APs (`"Argument list too long"`), que resuelve usando `iw`.

**Diferencias:**
- ROS 1, no ROS 2.
- Solo el nodo de escaneo; no incluye heatmap ni localización.

**Valor para el TFG:** Justificación técnica del uso de `iw`/`nmcli` en lugar de `/proc/net/wireless`. Citar en la sección de diseño del nodo de escaneo.

---

### 1.4 `rssi_module` — jvillagomez (GitHub)
**Repositorio:** https://github.com/jvillagomez/rssi_module  
**Estado:** Python 2/3, 2020, sin ROS  
**Relevancia para el TFG:** ⭐⭐

**Qué hace:**
- Librería Python ligera para escanear APs WiFi (`RSS_Scan`) y localización por trilateración (`RSSI_Localizer`).
- Requiere mínimo 3 APs con posición conocida para trilateración.
- Diseñada para entornos IoT/tiempo real.
- El TODO del repositorio menciona explícitamente añadir un paquete ROS — que nunca fue implementado.

**Diferencias:** Trilateración pura, sin filtro de partículas ni mapa de fingerprint.

---

### 1.5 `wilson_ros` — WilsonROS (GitHub)
**Repositorio:** https://github.com/WilsonROS/wilson_ros  
**Estado:** ROS 1, sin documentación extensa  
**Relevancia para el TFG:** ⭐⭐

**Qué hace:** Sistema de localización inalámbrica en interiores sobre ROS. Escasa documentación pública, pero incluye nodos de escaneo WiFi y localización básica.

---

### 1.6 `wifi_scan` — ROS Index (paquete oficial ROS)
**Página:** https://index.ros.org/p/wifi_scan/  
**Estado:** ROS 1 (Hydro+), mantenido hasta 2019  
**Relevancia para el TFG:** ⭐⭐⭐

**Qué hace:**
- Driver de escaneo WiFi para ROS que publica RSSI de todos los APs visibles en el topic `/wifi_fingerprint` como mensaje `wifi_scan/Fingerprint`.
- Basado en `iw` (no en `iwlist`), compatible con entornos con muchos APs.
- Requiere permisos SUID sobre el ejecutable (alternativa al `NET_ADMIN` de Docker).

**Diferencias:** No existe port a ROS 2. No incluye heatmap ni localización.

**Valor para el TFG:** El tipo de mensaje `wifi_scan/Fingerprint` es una referencia directa para diseñar el mensaje custom `wifi_msgs/WifiScan` de Montecarla.

---

### 1.7 Proyecto Hackaday: "Generating a Wi-Fi heatmap" (Wild Thumper)
**URL:** https://hackaday.io/project/25406-wild-thumper-based-ros-robot/log/157556-generating-a-wi-fi-heatmap  
**Estado:** ROS 1, 2018  
**Relevancia para el TFG:** ⭐⭐⭐

**Qué hace:**
- Robot Wild Thumper con ROS 1 mide RSSI cada 0.5 segundos mientras navega manualmente por el entorno.
- Genera un **mapa de contorno** (contour plot) con OpenLayers superpuesto al plano SLAM.
- No usa Nav2 Waypoint Follower; la navegación es manual con teleoperación.
- Inspirado directamente en la charla de ROSCon 2018 sobre heatmaps WiFi.

**Valor para el TFG:** Precursor conceptual del sistema Husarion. Muestra que la idea base existe desde 2018 sin necesidad de VPN.

---

## CATEGORÍA 2: Investigación Académica — Sistemas de Fusión WiFi + LiDAR con Filtro de Partículas

---

### 2.1 "Hybrid Indoor Localization System" — KNNBP + HPFL (NCB/MDPI 2018)
**URL:** https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6211104/  
**Relevancia:** ⭐⭐⭐⭐⭐

**Qué hace:**
- Sistema de localización híbrido de dos fases: (1) posición aproximada con fingerprint WiFi (KNN por posibilidad), (2) posición precisa con filtro de partículas que fusiona WiFi + LiDAR + brújula + encoders.
- El algoritmo KNNBP usa similitud por probabilidad en lugar de distancia euclídea, lo que elimina el impacto negativo de señales RSSI ausentes.
- Resultados: convergencia más rápida y menos partículas necesarias que AMCL puro.

**Equivalencia con Montecarla:** Exactamente la misma arquitectura del Nivel 2 — filtro de partículas extendido con canal WiFi. El KNNBP es una alternativa al modelo gaussiano propuesto en Montecarla.

**Cita para la memoria:** Wu et al., "Design of a Hybrid Indoor Location System Based on Multi-Sensor Fusion for Robot Navigation", *Sensors*, 2018. DOI: 10.3390/s18103600.

---

### 2.2 "An Adaptive Indoor Localization Approach Using WiFi RSSI Fingerprinting with SLAM-Enabled Robotic Platform and Deep Neural Networks" (arXiv 2024)
**URL:** https://arxiv.org/abs/2407.09242  
**Relevancia:** ⭐⭐⭐⭐⭐ (publicación más reciente y más cercana al TFG)

**Qué hace:**
- Sistema sobre **ROS 2 Galactic + SLAM Toolbox** con robot móvil (ruedas Mecanum + Jetson Nano).
- Robot recorre el entorno mientras recopila RSSI de **38 APs simultáneos** y construye el mapa SLAM en paralelo.
- Alinea el mapa WiFi con el mapa geométrico generado por SLAM.
- Genera **heatmap del fingerprint WiFi** sobre el mapa 2D (idéntico al objetivo de Montecarla).
- Entrena una **red neuronal profunda** (DNN) como localizador — en lugar de filtro de partículas.

**Diferencias respecto a Montecarla:**
- Usa DNN en lugar de filtro de partículas MCL.
- No integra el WiFi en AMCL (son sistemas separados).
- No usa Nav2 Waypoint Follower; navegación por interfaz web personalizada.
- Publicado en ROS 2 Galactic, no Humble.

**Valor para el TFG:** Es el trabajo relacionado más cercano publicado en 2024. La diferencia clave de Montecarla es la integración directa en AMCL como canal adicional, en lugar de un localizador WiFi independiente. **Cita obligatoria en la memoria.**

---

### 2.3 "Online Indoor Localization Using DOA of Wireless Signals" — Latif & Parasuraman (2022)
**URL:** https://arxiv.org/pdf/2201.05105  
**Relevancia:** ⭐⭐⭐

**Qué hace:**
- Filtro de partículas que usa la **Dirección de Llegada (DOA)** de señales WiFi (no solo RSSI) para localización online sin fase offline de fingerprinting.
- Actualiza pesos de partículas con distribución gaussiana sobre el error DOA.
- Evaluado en simulación y datasets reales.

**Diferencia principal:** DOA requiere hardware especial (antenas direccionales); RSSI es más simple y universal. Montecarla usa RSSI, que es más accesible con hardware estándar.

---

### 2.4 "WiFi-based Global Localization in Large-Scale Environments Leveraging Structural Priors from osmAG" (arXiv 2025)
**URL:** https://arxiv.org/pdf/2508.10144  
**Repositorio:** https://github.com/XuMa369/osmag-wifi-localization  
**Relevancia:** ⭐⭐⭐⭐

**Qué hace:**
- Framework de localización global WiFi para entornos de gran escala (11.025 m² multi-planta).
- Usa OpenStreetMap Area Graph (osmAG) como mapa geométrico base.
- Error medio de localización: 3,12 m en zonas con fingerprint, 3,83 m sin fingerprint.
- Resuelve el **problema del robot secuestrado** (kidnapped robot problem) — crucial para validar Montecarla en zonas con ambigüedad LiDAR.

**Valor para el TFG:** Referencia para el escenario de validación de Montecarla en pasillos simétricos — exactamente el caso donde el WiFi aporta más respecto a LiDAR solo.

---

### 2.5 "Particle filter robot localisation through robust fusion of laser, WiFi, compass, and external cameras" (ResearchGate, 2015)
**URL:** https://www.researchgate.net/publication/274903089  
**Relevancia:** ⭐⭐⭐

**Qué hace:**
- Filtro de partículas que combina LiDAR + WiFi + brújula + cámaras externas.
- Error medio de localización: 1,28 m (peor que solo LiDAR en entornos ricos en features geométricas, mejor en entornos simétricos).
- Resultado clave: **el WiFi solo aporta mejora significativa cuando el LiDAR tiene ambigüedad geométrica**.

**Valor para el TFG:** Justificación teórica del escenario de validación de Montecarla. Documenta que la fusión WiFi+LiDAR es más eficaz en pasillos simétricos.

---

## CATEGORÍA 3: Proyectos Multi-Robot con WiFi (menos directos pero relevantes)

---

### 3.1 `MGPRL` — HeRoLab, University of Georgia (ROS, 2025)
**Repositorio:** https://github.com/herolab-uga/MGPRL  
**Publicación:** arXiv 2506.23514 (2025)  
**Relevancia:** ⭐⭐⭐

**Qué hace:**
- Localización relativa multi-robot usando **Procesos Gaussianos Distribuidos** sobre RSSI de múltiples APs.
- Cada robot predice el campo RSSI de su entorno escaneando APs visibles (sin red activa requerida).
- No requiere calibración offline ni fingerprinting previo.
- 43% de mejora respecto al estado del arte en tests con 3 robots y 4 APs.
- Open-source como paquete ROS (ROS 1 + Gazebo).

**Diferencia con Montecarla:** Sistema multi-robot para localización *relativa* entre robots, no localización *global* en mapa fijo. Sin Nav2, sin AMCL.

---

### 3.2 `hgprl` — HeRoLab (ROS, IROS 2024)
**Repositorio:** https://github.com/herolab-uga/hgprl  
**Relevancia:** ⭐⭐⭐

**Qué hace:**
- Procesos Gaussianos Jerárquicos para localización relativa multi-robot usando WiFi.
- Incluye demostración con robots reales en ROS y simulación en Robotarium.
- Aplicación a algoritmo de rendezvous (todos los robots se encuentran en un punto).

---

### 3.3 "Collaborative Radio SLAM for Multiple Robots based on WiFi Fingerprint Similarity" (arXiv 2021)
**URL:** https://arxiv.org/pdf/2110.06541  
**Relevancia:** ⭐⭐

**Qué hace:**
- SLAM colaborativo multi-robot usando únicamente señal WiFi (sin LiDAR).
- Modelo de similitud que combina RSS y probabilidad de detección de AP.
- Solución centralizada que optimiza trayectorias en base a odometría + fingerprints de múltiples robots.

---

### 3.4 "AuF: Autonomous WiFi Fingerprinting" (arXiv 2019)
**URL:** https://arxiv.org/pdf/1911.11825  
**Relevancia:** ⭐⭐⭐

**Qué hace:**
- Robot recorre el entorno **sin detenerse** (travel-without-sojourn) para construir el fingerprint WiFi más rápidamente.
- Resuelve el problema de muestras reducidas por movimiento continuo con métodos de recuperación de señal.
- Más eficiente en energía y tiempo que el enfoque de stop-and-measure (como el de Husarion/Montecarla).

**Valor para el TFG:** Trabajo futuro o mejora de Montecarla — en lugar de 10 segundos en cada waypoint, medir en movimiento continuo.

---

## CATEGORÍA 4: Cómo Eliminar Husarnet (Comunicación ROS 2 en LAN)

> **Prerequisito verificado (2026-06-04):** el PC debe correr **Ubuntu 22.04 nativo**. WSL2 crea una interfaz virtual con IP `172.x.x.x` distinta a la IP real del host Windows en la LAN. El ROSbot ve la IP de Windows, pero ROS 2 corre en la IP de WSL2 — el DDS no puede establecer sesión. Solución adoptada en el TFG: dual boot Ubuntu 22.04 nativo.

La dependencia de Husarnet en el proyecto original de Husarion es **completamente eliminable** cuando robot y PC están en la misma red LAN (con Ubuntu nativo en el PC). La referencia directa es el repositorio oficial de Husarion de navegación:

**`rosbot-navigation`:** https://github.com/husarion/rosbot-navigation  
El fichero `.env` contiene:
```bash
DDS_CONFIG=DEFAULT   # LAN local — sin Husarnet
# DDS_CONFIG=HUSARNET_SIMPLE_AUTO   # Solo si se quiere usar sobre Internet
```

Con `DDS_CONFIG=DEFAULT` y `ROS_DOMAIN_ID` igual en ambos dispositivos, el DDS de ROS 2 (FastDDS o CycloneDDS) descubre los nodos automáticamente en LAN sin ningún componente externo.

**Configuración mínima para LAN (sin Husarnet):**

En el ROSbot y el PC, en el fichero `.env`:
```bash
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
DDS_CONFIG=DEFAULT
```

En el `compose.rosbot.yaml`:
```yaml
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # Sin nada de Husarnet
network_mode: host   # Necesario para multicast DDS en la misma red
```

**Puertos UDP a abrir si hay firewall:**
```
7400/UDP — DDS discovery (RTPS)
7401-7500/UDP — DDS data channels
```

---

## Tabla Comparativa Resumen

| Proyecto | ROS versión | Multi-SSID | Heatmap visual | Nav2 Waypoint | Fusión AMCL | Sin VPN | Open Source |
|---|---|---|---|---|---|---|---|
| **Montecarla (TFG)** | ROS 2 Humble | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Husarion rosbot-demo | ROS 2 Galactic | ❌ (1 red) | ✅ | ✅ | ❌ | ❌ | ✅ |
| ros-wifi-localization | ROS 1 | ✅ | ❌ | ❌ | ✅ AMCL mod. | ✅ | ✅ |
| WiFi_localization (sldai) | Sin ROS | ✅ | ❌ | ❌ | ✅ PF+IMU | ✅ | ✅ |
| ROS_WiFi_Scanner | ROS 1 | ✅ | ❌ | ❌ | ❌ | ✅ | ✅ |
| wifi_scan (ROS Index) | ROS 1 | ✅ | ❌ | ❌ | ❌ | ✅ | ✅ |
| Azghadi et al. 2024 | ROS 2 Galactic | ✅ (38 APs) | ✅ | ❌ | ❌ (DNN) | ✅ | Parcial |
| Wu et al. 2018 (HPFL) | Sin ROS | ✅ | ❌ | ❌ | ✅ HPFL | ✅ | ❌ |
| MGPRL (HeRoLab) | ROS 1 | ✅ | ❌ | ❌ | ❌ (relativa) | ✅ | ✅ |
| Wild Thumper Hackaday | ROS 1 | ❌ (1 red) | ✅ | ❌ | ❌ | ✅ | Parcial |

---

## Conclusión: Originalidad del TFG Montecarla

Ningún proyecto encontrado combina simultáneamente:
1. **Multi-SSID en tiempo real** (todos los APs visibles, sin red activa)
2. **Recogida autónoma con Nav2 Waypoint Follower** (stop-and-measure por waypoint)
3. **Heatmaps individuales por SSID** + **fingerprint vectorial**
4. **Fusión del fingerprint en nav2_amcl** como canal de observación adicional
5. **ROS 2 Humble** (sin bridge ROS 1)
6. **Sin dependencia de VPN externa** (LAN local)

El trabajo más cercano es `ros-wifi-localization` (Miyagusuku, 2019), que implementa la fusión WiFi+AMCL pero en ROS 1, con un único AP por configuración y sin heatmap visual. El artículo de Azghadi et al. (2024) es el más reciente y completo en términos de pipeline, pero usa DNN en lugar de filtro de partículas y no integra el WiFi en AMCL.

Esta combinación específica — **Nav2 Waypoint Follower + multi-SSID + AMCL extendido + ROS 2 Humble + LAN** — representa una contribución original documentable en el TFG.

---

## Referencias para la Memoria del TFG

1. **Miyagusuku et al.** (2019). "Data Information Fusion from Multiple Access Points for WiFi-based Self-localization". *IEEE Robotics and Automation Letters, 4(2), 269–276.* DOI: 10.1109/LRA.2018.2885583 — **[cita principal de trabajo relacionado]**

2. **Miyagusuku et al.** (2018). "Precise and accurate wireless signal strength mappings using Gaussian processes and path loss models". *Robotics and Autonomous Systems.* DOI: 10.1016/j.robot.2018.02.011

3. **Wu et al.** (2018). "Design of a Hybrid Indoor Location System Based on Multi-Sensor Fusion for Robot Navigation". *Sensors, 18(10), 3600.* DOI: 10.3390/s18103600 — **[referencia para el diseño del filtro de partículas WiFi]**

4. **Azghadi et al.** (2024). "An Adaptive Indoor Localization Approach Using WiFi RSSI Fingerprinting with SLAM-Enabled Robotic Platform and Deep Neural Networks". arXiv:2407.09242 — **[trabajo más reciente en ROS 2, cita obligatoria]**

5. **Ma et al.** (2025). "WiFi-based Global Localization in Large-Scale Environments Leveraging Structural Priors from osmAG". arXiv:2508.10144. GitHub: XuMa369/osmag-wifi-localization

6. **Latif & Parasuraman** (2025). "MGPRL: Distributed Multi-Gaussian Processes for Wi-Fi-based Multi-Robot Relative Localization". arXiv:2506.23514. GitHub: herolab-uga/MGPRL

7. **Dai et al.** (2019). "AuF: Autonomous WiFi Fingerprinting for Indoor Localization". arXiv:1911.11825 — **[referencia para trabajo futuro: navegación continua sin sojourn]**

8. **Biswas & Veloso** (2010). "WiFi localization and navigation for autonomous indoor mobile robots". *IEEE ICRA.* — Referencia clásica de fusión WiFi+partículas.

---

*Búsqueda realizada el 31 de mayo de 2026 — cobertura: GitHub, arXiv, IEEE, MDPI, ResearchGate, ROSIndex.*
