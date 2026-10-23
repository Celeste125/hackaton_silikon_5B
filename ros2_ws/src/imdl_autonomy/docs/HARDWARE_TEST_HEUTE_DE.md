# IMDL Hardwaretest: vollstaendiger Ablauf fuer Einsteiger

Diese Anleitung beschreibt den echten Create3 mit:

- Create3-Themen im Namespace `/Team2`,
- Lidar und RealSense am Jetson,
- RViz, AMCL, Nav2, Safety und Task-Solver auf dem Laptop,
- einer 1,63 x 1,63 m grossen realen Arena,
- unbekannten Hindernispositionen,
- optionalem Ultraschall und optionalem Manipulator.

Es wird im Realbetrieb niemals Gazebo und niemals eine `.world`-Datei
gestartet.

## 1. Was heute ehrlich getestet werden kann

| Ausstattung | Vollstaendig testbar |
| --- | --- |
| Create3, Lidar | Task 1 und Task 2 |
| Zusaetzlich RealSense/Perception | Objekterkennung, Tracking und Anfahrt fuer Task 3-5 diagnostizierbar |
| Kein Ultraschall | Alle Navigationstests; RealSense bleibt Distanzquelle |
| Kein Greifarm | Task 3-5 koennen nicht vollstaendig erfolgreich enden |
| Greifarm plus Hardware-Adapter | Task 3-5 koennen end-to-end getestet werden |

Ohne Greifarm melden Task 3, 4 und 5 absichtlich
`MANIPULATOR_UNAVAILABLE`. Das ist kein Navigationsfehler. Der Solver darf
nicht so tun, als habe er einen Block aufgenommen.

## 2. Verteilung der Programme

### Jetson

Der Jetson besitzt die USB-Verbindungen und startet:

- RealSense-Treiber,
- RGB/Depth-Alignment,
- Perception,
- SLLIDAR-Treiber,
- Korrektur der Lidar-Zeitstempel.

Er startet kein RViz, kein Gazebo, kein AMCL, kein Nav2 und keinen Solver.

### Laptop

Der Laptop startet:

- RobotModel und RViz,
- Wande-Occupancy-Map,
- AMCL und `map -> odom`,
- Nav2 Planner, Controller, Velocity Smoother und Collision Monitor,
- Object Registry, Safety Monitor und Localization Guard,
- den eigentlichen Task-Solver.

### Create3

Der Create3 stellt unter anderem bereit:

```text
/Team2/odom
/Team2/cmd_vel
/Team2/hazard_detection
/Team2/kidnap_status
/Team2/tf
/Team2/tf_static
```

## 3. Dateien, die auf beiden Rechnern zusammenpassen muessen

Auf dem Jetson muessen mindestens diese Source-Pakete vorhanden sein:

```text
ros2_ws/src/imdl_msgs
ros2_ws/src/perception
ros2_ws/src/slam
ros2_ws/src/sllidar_ros2
ros2_ws/src/imdl_simulation
```

`imdl_msgs` muss auf Jetson und Laptop exakt dieselbe Version haben. Ein
unterschiedliches `DetectedObject.msg` kann DDS-Nachrichten unlesbar machen.

Member As Stand aus `feature/slam_perception` umfasst neben dem
`perception`-Paket auch:

- `imdl_msgs`,
- `perception/config/perception.yaml`,
- `perception/models/yolo11n_seg_imdl.pt`,
- `perception/models/yolo11n_seg_imdl.sha256`.

Nur `real_sensors.launch.py` zu kopieren reicht fuer diesen Stand nicht.

## 4. Einmalige Jetson-Vorbereitung

### 4.1 Pakete kontrollieren

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
colcon list
```

In der Ausgabe muessen mindestens `imdl_msgs`, `perception`, `slam`,
`sllidar_ros2` und `imdl_simulation` erscheinen. Fehlt ein Paket, ist dessen
Verzeichnis unvollstaendig. Jedes ROS-Paket braucht insbesondere eine
`package.xml`.

### 4.2 YOLO-Laufzeit kontrollieren

Der Jetson braucht eine zu seiner JetPack-Version passende NVIDIA-PyTorch-
Installation. Nicht die Laptop-PyTorch-Installation kopieren.

```bash
cat /etc/nv_tegra_release
python3 --version
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

`torch.cuda.is_available()` sollte auf dem Jetson `True` liefern. Member As
Perception-Stand erwartet Ultralytics `8.4.87`. Wenn PyTorch bereits korrekt
fuer JetPack installiert ist:

```bash
python3 -m pip install --no-deps ultralytics==8.4.87
```

`--no-deps` ist wichtig, damit pip nicht die NVIDIA-PyTorch-Version ersetzt.

Modellintegritaet:

```bash
cd ~/Desktop/ros2_ws/src/perception/models
sha256sum -c yolo11n_seg_imdl.sha256
```

### 4.3 Jetson bauen

Nicht nur `imdl_simulation` bauen. Der richtige Befehl fuer den heutigen
Sensorstand ist:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  imdl_msgs perception slam imdl_simulation
source install/setup.bash
```

## 5. Einmalige Laptop-Vorbereitung

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  imdl_msgs imdl_interfaces perception slam imdl_autonomy imdl_simulation
source install/setup.bash
```

Schnittstelle kontrollieren:

```bash
ros2 interface show imdl_msgs/msg/DetectedObjectArray
```

## 6. Netzwerk und Uhrzeit

Jetson und Laptop muessen im gleichen lokalen Netz sein. Gastnetze mit
Client-Isolation funktionieren oft nicht fuer ROS 2 DDS.

In jedem neuen Terminal auf beiden Rechnern:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Danach jeweils das lokale Workspace sourcen:

Jetson:

```bash
source ~/Desktop/ros2_ws/install/setup.bash
```

Laptop:

```bash
source /home/baran/imdl/ros2_ws/install/setup.bash
```

`ROS_DOMAIN_ID=2` ist ein Beispiel. Entscheidend ist, dass Create3, Jetson und
Laptop denselben vereinbarten Wert verwenden.

Kontrollen:

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
echo $RMW_IMPLEMENTATION
date +%s.%N
```

Die Uhren sollten moeglichst weniger als etwa 0,1 bis 0,2 Sekunden
auseinanderliegen. Bei grossen Abweichungen schlagen TF und Depth-Synchronisation
fehl. `timedatectl status` zeigt, ob die Zeitsynchronisation aktiv ist.

## 7. Physische Koordinaten messen

### 7.1 Map-Achsen festlegen

1. Markiere die Mitte der Arena als `(0, 0)`.
2. Lege eine Bodenrichtung als `+x` fest.
3. `+y` zeigt bei Blick entlang `+x` nach links.
4. Miss alle Positionen vom Mittelpunkt aus in Metern.
5. Yaw ist die physische Vorwaertsrichtung des Roboters:
   - `0` zeigt entlang `+x`,
   - `1.5708` zeigt entlang `+y`,
   - `3.1416` zeigt entlang `-x`,
   - `-1.5708` zeigt entlang `-y`.

Der visuelle 180-Grad-Offset des Meshes in RViz aendert diese Definition
nicht. Gemessen wird die echte Vorwaertsrichtung des Create3.

Der Frontpunkt ist ein Messpunkt von der Robotermitte aus gerade in der
physischen Vorwaerts-/Fahrtrichtung des Roboters. Er hat nichts mit der
Blickrichtung der messenden Person zu tun.

### 7.2 Bedeutung von A, B und C

| Task | Bedeutung |
| --- | --- |
| Task 1 | A = Roboterstart, B = Fahrziel |
| Task 2 | A = Roboterstart, B und C = zwei Fahrziele |
| Task 3 | A = Roboterstart, B = bekanntes Blockzentrum, C = Ablagezentrum |
| Task 4 | A = Roboterstart, C = Ablagezentrum; Objekt unbekannt |
| Task 5 | A = Roboterstart, C = Ablagezentrum; drei Objekte unbekannt |

Robotermessungen beziehen sich auf die Mitte von `base_link`, nicht auf die
Kamerakante. Blockmessungen beziehen sich auf das Blockzentrum.

## 8. Reale Mission erzeugen

Eine reale Mission verwendet die reine Wande-Map
`maps/arena_unknown.yaml`. Darin stehen keine Zylinder und keine Bloecke.
Diese Hindernisse kommen erst waehrend der Fahrt aus `/scan`, RGB-D und den
lokalen Nav2-Costmaps.

### Task 1

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task1 --mission-id real_task1_01 \
  --a -0.50 -0.35 0.0 \
  --b 0.40 -0.35 0.0 \
  --output /home/baran/imdl/real_task1_01.yaml --force
```

### Task 2

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task2 --mission-id real_task2_01 \
  --a -0.50 -0.35 0.0 \
  --b 0.35 -0.25 1.5708 \
  --c 0.35 0.40 3.1416 \
  --output /home/baran/imdl/real_task2_01.yaml --force
```

### Task 3

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task3 --mission-id real_task3_01 \
  --a -0.50 -0.35 0.0 \
  --b 0.20 0.10 \
  --c -0.30 0.45 \
  --target-color red \
  --output /home/baran/imdl/real_task3_01.yaml --force
```

### Task 4

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task4 --mission-id real_task4_01 \
  --a -0.50 -0.35 0.0 \
  --c -0.30 0.45 \
  --target-color red \
  --output /home/baran/imdl/real_task4_01.yaml --force
```

### Task 5

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task5 --mission-id real_task5_01 \
  --a -0.50 -0.35 0.0 \
  --c -0.30 0.45 \
  --color-order red blue yellow \
  --output /home/baran/imdl/real_task5_01.yaml --force
```

Die Zahlen sind nur Beispiele und muessen durch eure Messungen ersetzt werden.

Relative Messung von B aus dem Roboterframe bei A:

```bash
--b-relative-to-a VORWAERTS LINKS DELTA_YAW
```

Beispiel: 0,60 m vor und 0,10 m links:

```bash
--b-relative-to-a 0.60 0.10 0.0
```

## 9. Sicherer Startablauf

Fuer den ersten Test werden Stack und Mission getrennt gestartet. Dadurch
bewegt sich der Roboter nicht, waehrend Sensoren und TF kontrolliert werden.

### 9.1 Create3 kontrollieren

Auf dem Laptop:

```bash
ros2 topic hz /Team2/odom
ros2 topic echo /Team2/hazard_detection --once
ros2 topic echo /Team2/kidnap_status --once
```

Wenn `/Team2/odom` nicht fortlaufend erscheint, keine Autonomie starten.

### 9.2 Jetson: Sensoren starten

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
ros2 launch imdl_simulation real_sensors.launch.py \
  lidar_mode:=driver start_rviz:=false
```

Falls die Kameraextrinsik vom Default abweicht:

```bash
ros2 launch imdl_simulation real_sensors.launch.py \
  lidar_mode:=driver start_rviz:=false \
  camera_x:=0.19 camera_y:=0.0 camera_z:=-0.02 \
  camera_roll:=0.0 camera_pitch:=0.262 camera_yaw:=0.0
```

Diese Werte muessen am echten Aufbau gemessen bzw. kalibriert werden.

### 9.3 Laptop Terminal 1: RViz und RobotModel

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
ros2 launch imdl_simulation real_rviz.launch.py
```

RViz verwendet `map` als Fixed Frame. Der 180-Grad-Wert korrigiert nur die
Meshdarstellung und nicht Sensor-TF oder Navigation.

### 9.4 Laptop Terminal 2: Nav2 und Safety, noch ohne Mission

Task-2-Beispiel:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
ros2 launch imdl_autonomy real_autonomy.launch.py \
  mission_file:=/home/baran/imdl/real_task2_01.yaml \
  start_mission:=false \
  use_front_ultrasonic:=false
```

Dieser Launch publiziert die gemessene Pose A automatisch auf
`/initialpose`. A darf irgendwo in der Arena liegen.

### 9.5 Vor jeder Bewegung kontrollieren

```bash
ros2 topic hz /scan
ros2 topic hz /Team2/odom
ros2 topic echo /autonomy/safety_state --once
ros2 topic echo /autonomy/localization_state --once
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

Erwartet:

- `/scan` und `/Team2/odom` laufen fortlaufend.
- Safety meldet `"safe": true`.
- Localization meldet `"ready": true`.
- Alle Nav2-Lifecycle-Nodes sind `active`.
- `map -> base_link` liegt in der Naehe der gemessenen Pose A.
- Lidarlinien liegen in RViz an den echten Waenden.

Perception:

```bash
ros2 topic type /perception/detected_objects
ros2 topic hz /perception/detected_objects
ros2 topic echo /perception/detected_objects --once
ros2 topic echo /tracked_objects --once
```

Ein brauchbarer Block fuer Navigation braucht:

```text
is_block: true
map_pose_valid: true
is_stale: false
```

Ein brauchbarer Block fuer Pick/Kamera-Control braucht zusaetzlich:

```text
depth_valid: true
base_pose_valid: true
```

### 9.6 Laptop Terminal 3: Mission starten

Ohne Ultraschall immer explizit `require_ultrasonic:=false` setzen:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash
ros2 launch imdl_autonomy run_task.launch.py \
  mission_file:=/home/baran/imdl/real_task2_01.yaml \
  use_sim_time:=false \
  require_ultrasonic:=false
```

Nur eine Mission starten. Wenn `real_autonomy.launch.py` mit
`start_mission:=true` laeuft, darf `run_task.launch.py` nicht zusaetzlich
gestartet werden.

Mission beobachten:

```bash
ros2 topic echo /mission_state
ros2 topic echo /task_result
ros2 topic echo /performance_metrics
```

## 10. Testreihenfolge fuer heute

1. Sensoren und Topics ohne Bewegung pruefen.
2. TF und AMCL bei stillstehendem Roboter pruefen.
3. Task 1 mit einem nur 20 bis 30 cm entfernten B ohne Hindernis fahren.
4. Erst danach die vollstaendige Task 1 fahren.
5. Task 2 zunaechst mit einem Zylinder und grosszuegigem Abstand fahren.
6. Danach die offizielle Task-2-Anordnung testen.
7. Erst bei stabiler Navigation Perception und Tasks 3-5 untersuchen.

Zwischen Versuchen den Roboter exakt auf das neu gemessene A stellen. Wenn A
oder die YAML geaendert wurde, sowohl `real_autonomy.launch.py` als auch
`run_task.launch.py` neu starten. Der Localization Guard und der Solver muessen
dieselbe Mission-ID und dieselbe Startpose verwenden.

## 11. Betrieb ohne Ultraschall

Das ist voll unterstuetzt:

```text
real_autonomy: use_front_ultrasonic:=false
run_task:      require_ultrasonic:=false
```

Dann gilt:

- Ultraschall ist nicht im Collision Monitor aktiv.
- Der Solver wartet nicht auf `/ultrasonic/front/range`.
- Lidar und Create3-Hazards sichern die Navigation.
- Bei typisierter Perception liefert RealSense `depth_m` und `range_m`.
- Die Kamera zentriert den Block weiterhin ueber `center_u_px`.

Kein Dummy-Topic und kein gefaelschter Ultraschallwert ist erforderlich.

## 12. Betrieb ohne Greifarm

Task 1 und 2 sind davon nicht betroffen.

Fuer Task 3-5:

1. Starte Jetson-Sensoren, RViz und `real_autonomy` mit
   `start_mission:=false`.
2. Pruefe `/perception/detected_objects` und `/tracked_objects`.
3. Pruefe in RViz, ob Blockpositionen plausibel sind.
4. Verwende Nav2 nur fuer kontrollierte Navigationsdiagnosen.
5. Starte den Task-3/4/5-Solver nicht mit der Erwartung eines Gesamterfolgs.

Der fehlende Hardware-Service ist:

```text
/autonomy/manipulate_object
imdl_interfaces/srv/ManipulateObject
```

Member Cs Hardware-Adapter muss spaeter dieselbe PICK/PLACE-Schnittstelle wie
der Gazebo-Adapter implementieren. Der Gazebo-Adapter darf nicht am echten
Roboter gestartet werden.

## 13. Not-Stopp und kontrolliertes Beenden

Waehle vor dem ersten Lauf eine Person, die ausschliesslich den Roboter und
den physischen Stop beobachtet. Halte die offizielle Create3-Stopmoeglichkeit
frei erreichbar.

Softwareseitig:

1. `Ctrl-C` im Missionsterminal.
2. Falls der Roboter nicht sofort steht, auch `Ctrl-C` im
   `real_autonomy`-Terminal.
3. Als zusaetzlicher Nullbefehl:

```bash
ros2 topic pub --rate 10 /Team2/cmd_vel geometry_msgs/msg/Twist "{}"
```

Diesen Publisher nach Stillstand mit `Ctrl-C` beenden. Danach die Ursache
untersuchen und nicht direkt neu starten.

Der Mission Manager cancelt beim normalen `Ctrl-C` seine Nav2-Action. Create3
BUMP, CLIFF, STALL, WHEEL_DROP und KIDNAP werden zusaetzlich vom Safety Monitor
ueberwacht.

## 14. Daten fuer Fehleranalyse aufzeichnen

Vor einem wichtigen Lauf auf dem Laptop:

```bash
mkdir -p /home/baran/imdl/bags
cd /home/baran/imdl/bags
ros2 bag record -o task2_01 \
  /scan \
  /Team2/odom \
  /Team2/hazard_detection \
  /Team2/kidnap_status \
  /tf \
  /tf_static \
  /amcl_pose \
  /autonomy/safety_state \
  /autonomy/localization_state \
  /mission_state \
  /task_result \
  /performance_metrics \
  /perception/detected_objects \
  /tracked_objects
```

Raw RGB und Depth erzeugen grosse Bags. Nur fuer Perception-Diagnose
zusaetzlich aufnehmen:

```bash
ros2 bag record -o perception_01 \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /perception/detected_objects
```

## 15. Wo Logik und Parameter geaendert werden

Nie Dateien unter `build/` oder `install/` bearbeiten. Nur unter `src/`.

### Task-Logik

```text
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/task1.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/task2.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/task3.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/task4.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/task5.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/transport.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/solvers/search.py
```

### Navigation und Sicherheit

```text
ros2_ws/src/imdl_autonomy/config/nav2_params.yaml
ros2_ws/src/imdl_autonomy/imdl_autonomy/navigation_adapter.py
ros2_ws/src/imdl_autonomy/imdl_autonomy/safety_monitor.py
```

### Solverparameter

```text
ros2_ws/src/imdl_autonomy/config/solver_params.yaml
```

### Perception

```text
ros2_ws/src/perception/config/perception.yaml
ros2_ws/src/perception/perception/
```

### Reale Sensorstarts und Extrinsik

```text
ros2_ws/src/imdl_simulation/launch/real_sensors.launch.py
ros2_ws/src/perception/launch/camera.launch.py
ros2_ws/src/slam/launch/lidar.launch.py
```

## 16. Was nach einer Aenderung neu gestartet wird

| Aenderung | Aktion |
| --- | --- |
| Nur Missions-YAML | Laptop `real_autonomy` und Solver neu starten |
| Python-Solver | `imdl_autonomy` auf Laptop bauen, Solver neu starten |
| `solver_params.yaml` | `imdl_autonomy` bauen, betroffene Nodes neu starten |
| `nav2_params.yaml` | `imdl_autonomy` bauen, kompletten `real_autonomy`-Stack neu starten |
| Perception-Code/Config | `imdl_msgs perception` auf Jetson bauen, Sensorlaunch neu starten |
| Message-Definition | `imdl_msgs` auf beiden Rechnern bauen, alle ROS-Nodes neu starten |
| Kameraextrinsik | Sensorlaunch mit neuen Argumenten neu starten |

Laptop-Build nach Autonomieaenderung:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select imdl_autonomy
source install/setup.bash
```

Jetson-Build nach Perceptionaenderung:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select imdl_msgs perception
source install/setup.bash
```

## 17. Haeufige Fehler

### `Package ... not found`

```bash
colcon list
ls src/PAKETNAME
```

Wenn `package.xml` fehlt, ist nur ein Unterordner statt des vollstaendigen
Pakets kopiert worden.

### Jetson-Themen sind auf Laptop unsichtbar

Auf beiden Rechnern `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`,
`RMW_IMPLEMENTATION`, WLAN und Firewall kontrollieren. Danach ROS-Daemon neu
starten:

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic list --spin-time 10 --no-daemon
```

### `/perception/detected_objects` fehlt

Pruefen:

```bash
ros2 node list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
python3 -c "import torch, ultralytics; print(torch.cuda.is_available())"
```

Beim neuen Perception-Stand auch Modellpfad und Modell-Hash pruefen.

### `map_pose_valid: false`

Die Kamera sieht das Objekt, aber TF bis `map` fehlt oder ist zeitlich nicht
verfuegbar:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

Ausserdem die Uhrzeit zwischen Jetson und Laptop kontrollieren.

### `SCAN_STALE`

```bash
ros2 topic hz /scan_raw
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Seriellen Port und Berechtigungen am Jetson kontrollieren.

### `ODOM_STALE`

```bash
ros2 topic hz /Team2/odom
```

Create3-Verbindung bzw. ROS-Domain kontrollieren.

### `AMCL_POSE_MISSING` oder falsche Roboterposition

- A und A-Yaw nachmessen.
- Kontrollieren, ob Scan und Wande-Map in RViz ueberlappen.
- Sicherstellen, dass `real_autonomy` und Solver dieselbe YAML verwenden.
- Keine alte Simulation mit `use_sim_time=true` parallel laufen lassen.

### `Nav2 action server not available`

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list
```

Mission erst starten, wenn die Server `active` sind.

### `MANIPULATOR_UNAVAILABLE`

Bei Task 3-5 ohne echten Hardware-Adapter ist diese Meldung korrekt. Task 1
oder 2 verwenden oder nur Perception/Nav2 diagnostizieren.

### Solver wartet auf Ultraschall

Der Real-Solver wurde ohne den notwendigen Parameter gestartet. Neu starten
mit:

```bash
require_ultrasonic:=false
```

## 18. Mindestprotokoll pro Versuch

Notiere:

- Datum und Mission-ID,
- echte A/B/C-Messungen,
- verwendete Git-Commits auf Jetson und Laptop,
- ROS-Domain und Rechner-IP-Adressen,
- Erfolg oder Fehlergrund,
- Laufzeit,
- Kollision oder Mindestabstand,
- AMCL-Startpose und Endpose,
- Bag-Verzeichnis,
- Aenderungen an Parametern.

Erst nach einem aufgezeichneten erfolgreichen Hardwarelauf darf
`hardware_validated` fachlich als wahr betrachtet werden. Der aktuelle Solver
publiziert dieses Feld vorsichtshalber weiterhin als `false`.
