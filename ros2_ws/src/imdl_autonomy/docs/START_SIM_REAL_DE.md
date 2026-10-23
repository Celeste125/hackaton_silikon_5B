# IMDL Startanleitung: Simulation und echter Roboter

## 1. Koordinatensystem

- Der feste Ursprung des `map`-Frames ist die Mitte der 1,63 x 1,63 m Arena.
- `+x` und `+y` sind einmal fest auf dem Boden definierte Achsen.
- Yaw wird in Radiant gegen den Uhrzeigersinn von `+x` gemessen.
- A ist die absolute Startpose des Robotermittelpunkts. A darf ueberall an
  einer kollisionsfreien Stelle in der Arena liegen und muss nicht `(0, 0)`
  sein.
- B und C werden vorzugsweise ebenfalls absolut im `map`-Frame angegeben.
- Alternativ bedeutet `--b-relative-to-a vorwaerts links delta_yaw` eine
  Messung im Roboterkoordinatensystem bei A. Das Tool rechnet sie vor dem Start
  in absolute Kartenkoordinaten um.

Die negative `origin`-Angabe in `arena_unknown.yaml` ist lediglich die
Koordinate der linken unteren Bildecke der Occupancy Map. Sie bedeutet nicht,
dass die Arenamitte verschoben ist; der physische Mittelpunkt bleibt im
`map`-Frame exakt `(0, 0)`.

Die statische Nav2-Karte enthaelt nur die Arenawaende. Zylinderpositionen
werden nicht in die Mission geschrieben; globale und lokale Costmap lernen sie
waehrend der Fahrt aus `/scan`.

Benoetigte Angaben pro Task:

| Task | Trial-Argumente |
| --- | --- |
| Task 1 | `--a x y yaw --b x y yaw` |
| Task 2 | `--a x y yaw --b x y yaw --c x y yaw` |
| Task 3 | `--a x y yaw --b x y --c x y --target-color red` |
| Task 4 | `--a x y yaw --c x y --target-color red` |
| Task 5 | `--a x y yaw --c x y --color-order red blue yellow` |

Bei Task 3 ist B das bekannte Objektzentrum, nicht eine Roboterpose. Bei Task
4 und 5 fehlt B absichtlich, weil der Solver die Objekte suchen muss.

## 2. Einmal bauen

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  imdl_interfaces perception slam imdl_autonomy imdl_simulation
source install/setup.bash
```

## 3. Simulation mit vorhandener Welt

Terminal 1 erstellt die Gazebo-Welt, spawnt den Roboter an A aus der Mission
und startet Sensoren, Perception, Nav2 und RViz:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imdl_simulation autonomy.launch.py task:=task2 variant:=a
```

Nach `Gazebo ready; starting perception and Nav2.` startet Terminal 2 den
Solver:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imdl_autonomy run_task.launch.py task:=task2 variant:=a
```

`variant` waehlt nur die Gazebo-Welt und die dazugehoerige Missionsdatei. Im
Solver gibt es keine Variantenlogik.

Bei Task 3 bis 5 prueft Terminal 1 vor dem Nav2-Start zusaetzlich Gazebos
`/get_entity_state` und `/set_entity_state`. Terminal 2 wartet danach bis zu
12 Sekunden auf `/autonomy/manipulate_object`, sodass ein normales
ROS-Discovery-Rennen nicht als `MANIPULATOR_UNAVAILABLE` endet.

## 4. Simulation mit selbst gemessenen A/B/C

Zuerst eine Mission erzeugen. Dieses Beispiel startet den Roboter bewusst
nicht in der Arenamitte:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run imdl_autonomy prepare_trial \
  --task task2 --mission-id sim_task2_measured \
  --a -0.50 -0.35 0.35 \
  --b 0.40 -0.20 1.570796 \
  --c 0.35 0.45 3.141593 \
  --output /home/baran/imdl/sim_task2_measured.yaml --force
```

Dann eine vorhandene oder selbst angepasste `.world` laden. Der Launch liest A
aus der YAML und spawnt den Roboter automatisch dort:

```bash
ros2 launch imdl_simulation autonomy.launch.py \
  mission_file:=/home/baran/imdl/sim_task2_measured.yaml \
  world:=/home/baran/imdl/ros2_ws/src/imdl_simulation/worlds/arena_task2_a.world
```

In Terminal 2 wird exakt dieselbe Mission gestartet:

```bash
ros2 launch imdl_autonomy run_task.launch.py \
  mission_file:=/home/baran/imdl/sim_task2_measured.yaml
```

Zum Erzeugen der Laufzeitkarte ist kein `generate_maps` noetig. Die kanonische
`arena_unknown.yaml` bleibt absichtlich eine reine Waendekarte, damit der
Solver Hindernisse aus dem simulierten Lidar statt aus der World-Datei kennt.

Eine eigene Task-3/4/5-World muss fuer simuliertes Pick/Place diesen
World-Plugin-Eintrag enthalten:

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros><namespace>/</namespace></ros>
  <update_rate>1.0</update_rate>
</plugin>
```

Bei Cancel, Timeout oder Fehler nach einem Pick sendet der Solver `RELEASE`.
Der Simulationsadapter stellt den Block auf seine Vor-Pick-Pose zurueck, damit
der naechste Versuch ohne Neustart der World moeglich bleibt.

## 5. Echter Roboter

Vorher muessen Jetson, Laptop und Create3 dieselbe ROS-Domain verwenden und
ihre Systemuhren synchronisiert sein.

Auf Jetson und Laptop vor den ROS-Befehlen dieselben Netzwerkwerte setzen
(sofern sie nicht bereits im Shell-Profil stehen):

```bash
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Nach dem Uebertragen dieses Projektstands auf den Jetson einmal neu bauen:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select slam imdl_simulation
```

Auf dem Jetson startet ein Terminal nur Kamera, Perception, Lidar und die
Korrektur der Lidar-Zeitstempel. Es startet absichtlich weder SLAM noch AMCL,
weil diese Komponenten der Laptop-Nav2-Stack besitzt:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imdl_simulation real_sensors.launch.py \
  lidar_mode:=driver start_rviz:=false
```

Auf dem Laptop wird aus den Messungen dieselbe absolute Mission erzeugt:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run imdl_autonomy prepare_trial \
  --task task2 --mission-id real_task2_01 \
  --a -0.50 -0.35 0.35 \
  --b 0.40 -0.20 1.570796 \
  --c 0.35 0.45 3.141593 \
  --output /home/baran/imdl/real_task2_01.yaml --force
```

Laptop-Terminal 1 startet RobotModel und RViz:

```bash
ros2 launch imdl_simulation real_rviz.launch.py
```

Laptop-Terminal 2 startet Nav2, AMCL und Safety, aber noch nicht die Mission:

```bash
ros2 launch imdl_autonomy real_autonomy.launch.py \
  mission_file:=/home/baran/imdl/real_task2_01.yaml \
  start_mission:=false
```

Erst wenn `/scan`, `/Team2/odom`, TF, AMCL und die Costmaps in RViz plausibel
sind, startet Laptop-Terminal 3 den Solver:

```bash
ros2 launch imdl_autonomy run_task.launch.py \
  mission_file:=/home/baran/imdl/real_task2_01.yaml \
  use_sim_time:=false
```

Alternativ startet `real_autonomy.launch.py` mit `start_mission:=true` den
Solver nach sechs Sekunden automatisch. Die getrennte Variante ist fuer den
ersten Hardwaretest sicherer und leichter zu diagnostizieren.

### Optionaler Arduino-Ultraschall

Die Hardware-Integration braucht keinen neuen Solver. Der Arduino-, Serial-
Bridge- oder micro-ROS-Node muss diesen ROS-Vertrag publizieren:

```text
/ultrasonic/front/range
sensor_msgs/msg/Range
frame_id: ultrasonic_front_link
radiation_type: ULTRASOUND
```

Erst nach erfolgreicher Pruefung mit

```bash
ros2 topic type /ultrasonic/front/range
ros2 topic hz /ultrasonic/front/range
ros2 topic echo /ultrasonic/front/range --once
```

wird er fuer Nav2/Safety aktiviert:

```bash
ros2 launch imdl_autonomy real_autonomy.launch.py \
  mission_file:=/home/baran/imdl/real_task3_01.yaml \
  start_mission:=false \
  use_front_ultrasonic:=true
```

Der separat gestartete Solver darf den Messwert fuer Tasks 3 bis 5 dann
verbindlich verlangen:

```bash
ros2 launch imdl_autonomy run_task.launch.py \
  mission_file:=/home/baran/imdl/real_task3_01.yaml \
  use_sim_time:=false \
  require_ultrasonic:=true
```

Ohne angeschlossenen Arduino bleiben beide Optionen `false`. Ein einzelner
Ultraschallsensor misst nur Frontdistanz; die Kamera bleibt fuer Farbe und
zentrale Links/Rechts-Ausrichtung verantwortlich.

Vor dem Missionsstart helfen diese Kontrollen:

```bash
ros2 topic hz /scan
ros2 topic hz /Team2/odom
ros2 topic echo /autonomy/localization_state
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
```

Ein unbekannter Roboterstart ohne gemessenes A ist nicht derselbe Fall. Dafuer
waere globale Lokalisierung, SLAM oder ein externer Referenzmarker erforderlich.
