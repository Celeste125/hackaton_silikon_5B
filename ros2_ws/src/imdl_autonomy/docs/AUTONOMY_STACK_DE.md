# IMDL Create3: kanonischer Autonomie-Stack

Stand: 16. Juli 2026

Dieses Dokument beschreibt die einzige aktive Autonomie-Implementierung. Die
fruehere Legacy- und Suffix-Trennung wurde entfernt; alle Launches, Solver,
Konfigurationen und Tests verwenden dieselben kanonischen Module.

## 1. Was der Stack garantiert

- Die Task-Solver entscheiden nur ueber Missionszustaende.
- Kein Task-Solver publiziert direkt `cmd_vel`.
- Nav2 plant und regelt jede Fahrt.
- Der Velocity Smoother begrenzt Beschleunigungen.
- Der Collision Monitor ist der letzte Publisher zum Create3.
- Die statische Occupancy Map liefert nur die vermessenen Arenawaende.
- Zylinder gelten als unbekannt und gelangen ausschliesslich ueber Live-Lidar
  in globale Costmap, lokale Costmap und Collision Monitor.
- Unbekannte Farbblockpositionen werden nie aus einer World-Datei gelesen.
- Kamera-Detektionen werden mit zeitlich passender Depth fusioniert und per TF
  in den `map`-Frame transformiert.
- Pick/Place ist derzeit nur in Gazebo implementiert.
- Tasks 3 bis 5 melden auf Hardware ehrlich `MANIPULATOR_UNAVAILABLE`.

## 2. Architektur

```text
mission_manager.py
  -> solvers/task1.py ... task5.py
  -> navigation_adapter.py -> Nav2 Actions
  -> runtime_context.py -> Localization, Safety, Tracks, Manipulator

localization_guard.py
  -> Startpose A, Scan/Odom/AMCL/TF-Watchdog

safety_monitor.py
  -> stale Scan/Odom, BUMP, CLIFF, STALL, WHEEL_DROP, Kidnap

object_registry.py
  -> TF-sichere Detektionen, Kalman-Tracking, Mahalanobis-Gating

simulation_manipulator.py
  -> validierter Gazebo Pick/Carry/Place/Release-Service

next_best_view.py
  -> A*, Sichtstrahlen, Bayes-Belief, Viewpoint-Scoring
```

Die gemeinsame nicht-blockierende Solver-Schnittstelle steht in
`imdl_autonomy/solvers/base.py`:

```python
start()   # Mission genau einmal initialisieren
tick()    # einen kurzen, nicht blockierenden Zustands-Schritt ausfuehren
cancel()  # aktive Nav2- und Manipulator-Aktionen abbrechen
```

Ein Solver liefert immer `RUNNING`, `SUCCEEDED` oder `FAILED`. Lange
`while`-Schleifen, `time.sleep()` und eigene Radregelungen gehoeren nicht in
einen Task-Solver.

### Robotermodell

`imdl_simulation/urdf/create3_imdl.urdf.xacro` ist das reproduzierbare Overlay
des gepinnten offiziellen Create3-Modells. Es enthaelt nur die
projektspezifischen Ergaenzungen: Lidar, RGB-D-Kamera, Front-Ultraschall,
`base_footprint` und den konfigurierbaren visuellen Yaw-Offset. Simulation,
Real-Like-Simulation und Real-RViz verwenden dieselbe Datei; Controller und
Create3-Nodes bleiben aus dem offiziellen iRobot-Paket.

## 3. ROS-Vertrag

Interne Frames und Topics:

```text
map -> odom -> base_link -> lidar_link / camera_color_optical_frame
                                  \-> ultrasonic_front_link
/scan
/ultrasonic/front/range
/tracked_objects
/mission_state
/task_result
/performance_metrics
/visualization_marker_array
/autonomy/global_path
```

Simulation und Hardware werden nur an den Raendern unterschieden:

```text
Simulation/Hardware Odom:   /Team2/odom
Simulation/Hardware Befehl: /Team2/cmd_vel
Create3 Hazards:            /Team2/hazard_detection
Create3 Kidnap:             /Team2/kidnap_status
Lidar:                      /scan
Front-Ultraschall:          /ultrasonic/front/range
```

Die Simulation verwendet `use_sim_time=true`; Hardware verwendet Systemzeit.
Der reale Sensorrechner muss Kamera, Lidar und Perception bereits im selben
ROS-Domain bereitstellen. `real_autonomy.launch.py` startet absichtlich
keinen zweiten Kamera- oder Lidar-Treiber.

Die normale Vorwaertskette ist konsistent auf maximal `0.90 rad/s` und
`2.0 rad/s2` abgestimmt. Ihr RobustMPPI-Profil ist strikt vorwaertsgerichtet;
nur Task 1 darf ueber ReverseMPPI negative Translation anfordern.
ReverseMPPI bleibt bei `0.75 rad/s`. Der Collision Monitor darf Drehungen in
der Slowdown-Zone weiterhin reduzieren. Auf
`/performance_metrics` werden Minimum, Maximum und Mittelwert von linearer und
angularer Geschwindigkeit fuer Controller, Smoother, Collision Monitor und
Odometrie getrennt ausgegeben.

### Front-Ultraschall

Simulation und spaeterer Arduino verwenden absichtlich denselben Vertrag:

```text
Topic:   /ultrasonic/front/range
Typ:     sensor_msgs/msg/Range
Frame:   ultrasonic_front_link
Modus:   radiation_type = ULTRASOUND
```

Der Sensor sitzt vorne auf Blockhoehe. Er liefert nur die Entfernung im
schmalen Messkegel. Er erkennt weder Farbe noch eine eindeutige Links/Rechts-
Abweichung und ersetzt deshalb weder Kamera noch Lidar oder AMCL.

Vor einem simulierten Pick gilt:

1. Die Kamera bestaetigt Farbe und zentriert den Block horizontal.
2. Die Track-Geometrie sagt die erwartete Frontdistanz voraus.
3. Mehrere frische Ultraschallwerte muessen innerhalb der konfigurierten
   Toleranz zu dieser Erwartung passen.
4. Erst dann darf der Gazebo-Manipulator den Pick ausfuehren.

Der Collision Monitor kann den Sensor zusaetzlich als letzte kurze Front-
Stoppschicht nutzen. Auf Hardware bleibt diese Quelle deaktiviert, bis ein
Arduino- oder micro-ROS-Node gueltige `Range`-Nachrichten publiziert. Ein
einzelner Sensor eignet sich nicht fuer eine sichere zweidimensionale
Positionsbestimmung durch blindes Wackeln; kleine Drehkorrekturen stammen
deshalb weiterhin aus dem gemessenen Kamerafehler und laufen als Nav2 Spin.

## 4. Missionsdateien

Die Laufzeitlogik kennt keine Variante `a` oder `b`. Eine Variante waehlt im
Launch nur eine vorhandene World und Missionsdatei aus. Der Solver liest nur
den Inhalt der YAML-Datei.

Beispiel Task 2:

```yaml
mission_id: real_task2_trial_01
task: task2
frame_id: map
coordinate_mode: absolute
obstacle_mode: unknown
map_yaml: /path/to/imdl_autonomy/maps/arena_unknown.yaml
time_limit_sec: 180.0
poses:
  A: {x: 0.50, y: -0.50, yaw: 3.141593}
  B: {x: -0.55, y: -0.10, yaw: 1.570796}
  C: {x: -0.50, y: 0.50, yaw: 0.0}
```

Pflichtfelder:

| Task | Posen | Weitere Felder |
| --- | --- | --- |
| Task 1 | A, B | keine |
| Task 2 | A, B, C | keine |
| Task 3 | A, B, C | `target_color` |
| Task 4 | A, C | `target_color`, optional `fixed_place_yaw` |
| Task 5 | A, C | exakt drei Werte in `color_order` |

Vor dem Start prueft `mission_geometry.py`, ob Roboterziele mit dem ganzen
Footprint gegen die Arenawaende frei und innerhalb der Arena erreichbar sind.
Unbekannte Zylinder koennen hier absichtlich nicht vorgeprueft werden. Objekt-
und Ablagezentren sind keine Roboterziele; fuer sie werden freie Approach-Ringe
geprueft.

## 5. Task-Algorithmen

### Task 1

1. `NavigateToPose` plant und replanned A nach B mit SmacPlanner2D.
2. FastRPP faehrt vorwaerts; der Adapter zeichnet dabei den laengsten von
   Nav2 publizierten globalen Pfad auf.
3. Nach dem Erreichen von B wird genau dieser Pfad geometrisch umgedreht.
4. ReverseMPPI darf laut Konfiguration nur `vx <= 0` ausgeben.
5. Gemessene Odometrie muss negative Translation bestaetigen.
6. Der Create3-Backup-Limit-Override wird danach immer wiederhergestellt.
7. Nav2 Spin stellt die geforderte Orientierung alpha an A her.

### Task 2

Der Solver faehrt ausschliesslich B und C aus der Missionsdatei an. Die
statische Karte kennt nur die Waende. Der 360-Grad-Lidar markiert Zylinder in
beiden Costmaps, globale Beobachtungen werden gepuffert, und MPPI/Smac planen
mit 2 Hz neu. Ein anfangs verdeckter Zylinder wird beim ersten Sichtkontakt in
den naechsten Plan aufgenommen. Der Collision Monitor bleibt unabhaengig davon
die letzte Stoppschicht. Es gibt keine zufaelligen Ziele und keinen
handgebauten `cmd_vel`-Fallback.

### Task 3

1. B wird als bekanntes Objektzentrum behandelt.
2. Mehrere raeumlich verschiedene Kamera-Viewpoints werden aus einem Ring
   erzeugt. Jeder Kandidat muss zuerst `ComputePathToPose` gegen die aktuelle
   Lidar-Costmap bestehen; erst dann beginnt die Fahrt.
3. Drei zeitlich verschiedene Detektionen bestaetigen den Track.
4. Ein engerer Pick-Approach wird neu geplant, ebenfalls live vorgeprueft und
   mit der tatsaechlichen Roboterpose exakt auf das Objekt ausgerichtet.
5. Vor `PICK` muessen drei neue Kameraframes den Block innerhalb von `0.05 rad`
   um die horizontale optische Mitte zeigen. Korrekturen laufen als begrenzte
   Nav2-Spin-Actions und nie als rohes `cmd_vel`. Zusaetzlich muessen die
   Sensor-Zeitstempel monoton steigen; verspaetete oder umsortierte Frames
   zaehlen nicht als neue Zentrierungsnachweise.
6. Ein Front-Ultraschall auf Blockhoehe bestaetigt den Abstand gegen die
   erwartete Track-Geometrie. Er wird nie fuer Farbe oder Suchrichtung benutzt.
7. Der Gazebo-Adapter validiert unabhaengig Track, Distanz, maximal `0.10 rad`
   Blickwinkel und Zielabweichung. `/task_result` enthaelt fuer jeden Pick den
   maschinenlesbaren Nachweis `visual_centered_picks` inklusive
   Ultraschallfehler.
8. Der Block wird zu einer kollisionsfreien virtuellen Ablage auf dem Roboter
   bewegt, nach C transportiert und exakt im Zentrum platziert. Der Adapter
   bestaetigt die Entity-Pose mit maximal `1.2 cm` Positions- und `0.05 rad`
   Yaw-Abweichung.
9. Bei einem Plan- oder Approach-Fehler wird eine alternative Ringposition
   verwendet. Unmoegliche Ziele verbrauchen so keine lange Controllerfahrt.
10. Safety-Stop, Timeout, Solverfehler und Cancel loesen `RELEASE` aus. Gazebo
   setzt ein noch getragenes Objekt auf seine Vor-Pick-Pose zurueck, damit ein
   zweiter offizieller Versuch nicht im Zustand `Already carrying` beginnt.

### Task 4

Ein bereits bestaetigter Track wird sofort genutzt. Sonst beginnt eine kurze
Kameradrehung und anschliessend Next-Best-View:

```text
Score = Information und neue Sichtabdeckung
        - Fahrzeit - Drehzeit - Costmap-Risiko
```

Raycasting schneidet Sicht an der vorgegebenen Waendenkarte ab. Unbekannte
Zylinder werden nicht aus der World-Datei gelesen: Die Live-Costmap verwirft
unerreichbare Viewpoints, und eine reale Kameraokklusion erscheint als
negative Beobachtung. Diese reduziert die Zellwahrscheinlichkeit bayesianisch,
setzt sie aber nicht auf null. `fixed_place_yaw` wird beim Ablegen wirklich
benutzt.

### Task 5

Die Farbreihenfolge wird strikt eingehalten. Die Registry speichert trotzdem
alle sichtbaren Farben. Ein schon bestaetigtes spaeteres Objekt wird gecached
und spaeter direkt angefahren; der Cache ist nur eine Navigationshypothese und
reicht nie allein fuer `PICK`. Vor jedem der drei Picks sind ein frischer Track
und die gleiche Bildzentrierung wie in Task 3 Pflicht. Nach Pick wird seine
Track-ID gesperrt. Die drei Bloecke werden bei C mit den Mittelpunktshoehen
`1.75`, `5.25` und `8.75 cm` zentriert gestapelt.

## 6. Perception

Die kanonische Schnittstelle von Member A ist:

```text
Topic: /perception/detected_objects
Typ:   imdl_msgs/msg/DetectedObjectArray
```

Fuer Navigation wird `position_map_m` nur verwendet, wenn `is_block`,
`map_pose_valid` und `!is_stale` gelten. Fuer Kameraausrichtung und Pick
muessen zusaetzlich `depth_valid` und `base_pose_valid` gelten. Die vier
greifbaren Klassen sind `red_block`, `yellow_block`, `green_block` und
`blue_block`. `blue_obstacle` hat `is_block=false` und kann daher weder als
Block verfolgt noch gegriffen werden.

Der Object Registry und der Camera Observation Store abonnieren ausschliesslich
diese typisierte Schnittstelle. Die bisherigen JSON-/HSV-Topics sind nicht
Teil dieser Integration.

`depth_m` und `range_m` stammen bereits von der RealSense. Ein optionaler
Ultraschallsensor dient nur als unabhaengige Distanz- und Sicherheitspruefung,
nicht als Ersatz fuer Depth oder Farberkennung.

Wichtig fuer Depth:

- RealSense `16UC1` wird von Millimetern nach Metern konvertiert.
- Gazebo `32FC1` ist bereits in Metern und wird nicht nochmals skaliert.
- RGB und Depth werden ueber den Aufnahmezeitpunkt gepaart.
- Jeder RGB-Frame wird hoechstens einmal als 3D-Detektion publiziert.
- `NaN`, `inf`, Nullwerte und unplausible Tiefen werden verworfen.
- Bei mehreren Projektionen auf ein Pixel gewinnt die naechste Oberflaeche.

Weiss, Grau und Schwarz in RViz sind Occupancy-Werte, keine erkannten
Objektfarben. Farbklassifikation kommt ausschliesslich aus dem Kamerabild.

## 7. Startbefehle

### Simulation ohne laufende Mission

Terminal 1:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imdl_simulation autonomy.launch.py task:=task2 variant:=a
```

Der Launch startet Nav2 erst, nachdem `/scan`, RGB, Depth, `/Team2/odom` und
die TF-Ketten zu Lidar und Kamera tatsaechlich bereit sind. Die Meldung
`Gazebo ready; starting perception and Nav2.` ist die eindeutige Freigabe.

Der Standardmodus ist fuer Laptops optimiert, ohne die Missionslogik zu
aendern: Kameraaufloesung `640x480` bei 20 Hz, unveraenderte Physik und
Kollisionen, aber keine Gazebo-Schatten/Sensor-Hilfsgrafiken und eine schlanke
RViz-Ansicht. Da Gazebo RGB und Depth bereits pixelgenau aus derselben
optischen Geometrie erzeugt, konvertiert `depth_align` diese Frames direkt in
Millimeter. Nur echte, geometrisch getrennte RealSense-Frames werden weiterhin
kalibriert reprojiziert. Am schnellsten bleibt die vollstaendige Simulation,
wenn nur ein 3D-Client rendert:

```bash
ros2 launch imdl_simulation autonomy.launch.py \
  task:=task2 variant:=a gui:=false rviz:=true
```

`gui:=false` beendet nur `gzclient`; `gzserver`, Lidar, RGB-D, Nav2 und
Perception laufen weiter. Fuer eine reine Gazebo-Ansicht gilt entsprechend
`gui:=true rviz:=false`.

Auf dem Referenz-Laptop stieg der headless gemessene Echtzeitfaktor dadurch von
`0.698` auf `0.932`. Lidar stieg von `6.99` auf `9.25 Hz`, RGB von `11.73` auf
`17.74 Hz` und aligned Depth von `9.48` auf `18.12 Hz`. Die CPU-Last von
`depth_align` fiel dabei von rund `1049 %` auf rund `22 %`. Die sichtbare
Gazebo-/RViz-FPS bleibt zusaetzlich von GPU und WSL/OpenGL-Beschleunigung
abhaengig.

Die schwere Diagnoseansicht mit 30-Hz-Kamera, sichtbaren Gazebo-Sensoren,
TF-Baum und drei RViz-Bildanzeigen ist weiterhin explizit verfuegbar:

```bash
ros2 launch imdl_simulation autonomy.launch.py \
  task:=task2 variant:=a \
  camera_update_rate:=30.0 sensor_visualization:=true \
  rviz_config:=/home/baran/imdl/ros2_ws/install/imdl_simulation/share/imdl_simulation/rviz/real_create3.rviz
```

### Mission separat starten

Terminal 2:

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imdl_autonomy run_task.launch.py task:=task2 variant:=a
```

Eigene Mission statt Variantenwahl:

```bash
ros2 launch imdl_autonomy run_task.launch.py \
  mission_file:=/absolute/path/to/trial.yaml
```

### Reales Geraet

Koordinatensystem einmal physisch markieren: Ursprung in der Arenamitte,
`+x`/`+y` als feste Bodenachsen, Yaw gegen den Uhrzeigersinn von `+x`. A ist
die absolute gemessene Startpose des Robotermittelpunkts und muss nicht im
Ursprung liegen. B und C werden ebenfalls absolut gemessen. Dann wird eine
gepruefte Mission fuer Simulation oder Hardware erzeugt:

```bash
ros2 run imdl_autonomy prepare_trial \
  --task task2 --mission-id real_task2_01 \
  --a -0.50 -0.50 0.0 \
  --b 0.45 -0.20 1.570796 \
  --c 0.40 0.50 3.141593 \
  --output /home/baran/imdl/real_task2_01.yaml
```

Optional kann B oder C mit `--b-relative-to-a` beziehungsweise
`--c-relative-to-a` als `vorwaerts links delta_yaw` angegeben werden. Das Tool
transformiert den Wert einmal in den `map`-Frame; zur Laufzeit sind alle Ziele
immer absolut. Eine relative Missions-YAML wird abgelehnt.

```bash
ros2 launch imdl_autonomy real_autonomy.launch.py \
  mission_file:=/home/baran/imdl/real_task2_01.yaml
```

Die Hindernisse muessen dabei weder vermessen noch in YAML eingetragen werden.
Noetig bleiben ein laufender `/scan`, die korrekte absolute Pose A und die
bekannten Arenagrenzen. Ein komplett unbekanntes A ist ein anderer Fall und
benoetigt globale Lokalisierung oder SLAM.

Der vollstaendige Ablauf mit bestehender oder eigener Gazebo-World,
Sensor-only-Start auf dem Jetson, RViz, Nav2 und separat gestartetem Solver
steht in `docs/START_SIM_REAL_DE.md`. `prepare_real_trial` bleibt als
kompatibler Alias fuer bestehende Befehle erhalten.

## 8. Bauen und testen

Dieses Workspace benoetigt wegen des eingesetzten Symlink-Install-Pfads
`setuptools==79.0.1`. Die aktuell gepruefte Installation verwendet genau diese
Version.

```bash
cd /home/baran/imdl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  imdl_interfaces perception slam imdl_autonomy imdl_simulation
source install/setup.bash
```

Reproduzierbarer Pytest-Lauf fuer die aktuelle Maschine:

```bash
bash src/imdl_autonomy/scripts/test.sh
```

Der explizite Colcon-Befehl lautet:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
colcon test --packages-select perception imdl_autonomy \
  --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

Die beiden Umgebungswerte verhindern Konflikte zwischen ROS Humble/
`cv_bridge`, System-NumPy 1.21 und globalen User-Pytest-Plugins. Die Launches
setzen fuer die laufenden Perception-Nodes bereits `PYTHONNOUSERSITE=1`.

## 9. RViz und Diagnose

Aktiviere `MarkerArray` auf `/visualization_marker_array`. Sichtbar sind:

- A, B und C in dauerhaft verschiedenen Farben,
- aktives Ziel und globaler Pfad,
- Controllername,
- exakter Footprint, Stop- und Slowdown-Zone,
- Tracks mit Confidence/Kovarianz,
- akzeptierte und verworfene Approach-Posen,
- NBV-Kandidaten, Sichtkegel und abgesuchte Flaeche,
- Zustand, Versuch und Laufzeit.

Hilfreiche Befehle:

```bash
ros2 topic echo /mission_state
ros2 topic echo /task_result
ros2 topic echo /performance_metrics
ros2 topic echo /tracked_objects --once
ros2 topic hz /scan
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

`Nav2 action server unavailable` bedeutet, dass Terminal 1 noch nicht bereit
ist oder eine andere ROS-Domain verwendet. `MANIPULATOR_UNAVAILABLE` ist auf
Hardware fuer Tasks 3 bis 5 derzeit das korrekte Ergebnis.

## 10. Ehrlicher Validierungsstand

Bereits tatsaechlich beobachtet:

| Lauf | Ergebnis |
| --- | --- |
| Task 1a Gazebo, kanonischer Stack | erfolgreich, 28.167 s, 0 Recoveries, echte negative Odom-Geschwindigkeit bis -0.100 m/s |
| Task 1a optimiertes Laptop-Profil | erfolgreich, 17.085 s, 0 Recoveries, Vorwaerts- und Rueckwaertsfahrt |
| Task 2a Gazebo | erfolgreich, 31.858 s, B 4.4 cm, C 5.0 cm, 0 Recoveries |
| Task 2b Gazebo | erfolgreich, 52.624 s, B 2.8 cm, C 4.9 cm, 0 Recoveries |
| Task 2a, unbekannte Hindernisse | erfolgreich, 49.987 s, B 7.6 cm/0.712 rad, C 7.0 cm/0.658 rad, 0 Recoveries; statische Karte enthaelt nur Waende |
| Freie Nav2-Drehung 90 Grad | 2.227 s, Fehler ca. 0.013 rad |
| Freie Nav2-Drehung 180 Grad | 3.995 s, Fehler ca. 0.014 rad |
| Aktuelle Autonomie- und Perception-Tests | 116/116 bestanden: 104 Autonomie, 12 Perception |
| Offizielle NBV-Suchsimulation | 4a: 5, 4b: 3, 5a: 7, 5b: 13 erreichbare Viewpoints bis alle benoetigten Farben gefunden waren |
| Python-Qualitaet | kritische Flake8-Regeln `E9,F63,F7,F82`, Compileall und `git diff --check` ohne Befund; der strenge repositoryweite Docstring-/Import-/79-Zeichen-Bestand ist noch nicht bereinigt |
| Build | `imdl_interfaces`, `perception`, `imdl_autonomy` und `imdl_simulation` erfolgreich |
| Launch-Syntax | sieben kanonische Simulation-, Mission-, Real- und Nav2-Launches erfolgreich geladen |
| Gazebo-Vertrag | Scan, RGB, Depth, Odom, Sensor-TF und fuer Task 3-5 Get/SetEntityState vor Nav2-Start verifiziert |
| Robotermodell | Xacro expandiert; `check_urdf` erfolgreich; Kamera liegt unter der Lidar-Ebene und der ganze 3.5-cm-Block bleibt an Kamera- und Pick-Ring im vertikalen FOV |

Noch offen:

- Der letzte tatsaechliche Task-3a-Lauf bestaetigte RGB, Depth, TF, roten Track
  und den ersten Pick-Approach. Er scheiterte danach ehrlich aus zwei
  reproduzierten Gruenden: Farbblockmodelle waren noch `static`, und die
  `2.5 cm`-Praezisionstoleranz liess MPPI wenige Zentimeter vor dem Greifziel
  stehen. Beide Ursachen sind im aktuellen Code behoben: dynamische
  kollisionsfaehige Bloecke, `5 cm` Navigation plus separate Winkel- und
  Bildausrichtung sowie verifiziertes Set/GetEntityState-Pick/Place.
- Seit diesen Korrekturen steht ein frischer Task-3a-End-to-End-Lauf noch aus.
  Der externe Gazebo-Start war am 15. Juli 2026 nicht durch ROS, sondern durch
  das Codex-Ausfuehrungslimit bis 22. Juli 2026, 08:57 Uhr blockiert.
- Task 4 und Task 5 sind durch Unit-/State-Machine-Tests abgedeckt, aber noch
  nicht end-to-end in allen Welten gefahren.
- Der aktuelle Stack wartet bei Task 3-5 kontrolliert auf den Manipulator,
  verwirft veraltete Kameraframes und rollt abgebrochene Picks per `RELEASE`
  zurueck. Diese neuen Laufzeitpfade sind automatisiert getestet, aber noch
  nicht in einem frischen Gazebo-Prozess beobachtet.
- Alle sechs offiziellen Task-3/4/5-Fixtures bestehen eine sequenzielle
  Offline-Pruefung mit dem 20.5-cm-Footprint, walls-only Solverkarte und
  zylinderhaltiger Referenz-Costmap. Zwei zuvor physisch unloesbare
  Task-5-Zylinderpositionen wurden minimal korrigiert; Anzahl, Durchmesser und
  unbekannter Laufzeitstatus blieben unveraendert.
- Die echte Next-Best-View-Scoring-/Bayes-Schleife findet in einer
  konservativen Offline-Simulation alle offiziellen unbekannten Objekte:
  Task 4a/4b nach 5/3 Viewpoints und Task 5a/5b nach insgesamt 7/13
  erreichbaren Viewpoints.
- Die geforderten drei Wiederholungen pro zehn Welten und randomisierten
  Stresslaeufe sind noch nicht abgeschlossen.
- Task 1 und Task 2 sind noch nicht mit dem echten Create3 validiert.
- Tasks 3 bis 5 warten auf den Hardware-Manipulatoradapter von Member C.

Die ersten Task-2-Zahlen stammen aus der inhaltlich gleichen Solverarchitektur
vor der abschliessenden Dateinamen-Kanonisierung. Task 2a wurde inzwischen mit
dem finalen installierten Stack und `arena_unknown.yaml` erneut gefahren. Eine
separate Costmap-Pruefung zeigte an beiden Zylinderzentren statisch den Wert 0,
waehrend die Live-Costmap dort 99 und in der Umgebung bis 100 meldete. Task 2b
sowie Tasks 3 bis 5 muessen fuer eine vollstaendige Release-Abnahme weiterhin
durch die gesamte kanonische Matrix laufen.

Deshalb ist der Stack strukturell und testseitig deutlich staerker, aber noch
nicht ehrlich als "perfekt" oder hardwarevalidiert zu bezeichnen.

### Fortsetzungspunkt

Als Naechstes Task 3a mit einem frischen ROS-Domain-ID-/Gazebo-Prozess starten.
Im Log muessen nacheinander `WAIT_*_PLAN`, ein `CONFIRMED`-Track,
`BLOCK_CENTERED`, `PICK confirmed` und `PLACE confirmed` erscheinen.
`/task_result` muss erfolgreich sein und genau einen Eintrag unter
`visual_centered_picks` enthalten. Danach dieselbe Abnahme fuer 3b, 4a/4b und
5a/5b; Task 5 muss drei Eintraege in Missions-Farbreihenfolge liefern. Fuer die
Abnahme CycloneDDS und je Lauf eine frische Domain verwenden, damit keine alten
transient-local Ergebnisse in den Test geraten.
