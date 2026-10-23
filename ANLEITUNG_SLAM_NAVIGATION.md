# Anleitung: Karte mit SLAM erstellen und zu Koordinaten fahren

Create3 + SLAMTEC-Lidar am Jetson, Laptop mit ROS 2 Humble im Docker-Container.
Es wird nur der Lidar verwendet, keine Kamera.

## Überblick: was läuft wo?

| Rechner | Phase 1: Karte erstellen | Phase 2: Navigation |
| --- | --- | --- |
| **Jetson** (SSH) | `mapping.launch.py` (Lidar + SLAM), teleop | `lidar.launch.py` (nur Lidar) |
| **Laptop** (Container) | RViz, Karte speichern | RViz, AMCL + Nav2, Ziele senden |

Wichtig:

- Alle Rechner verwenden **`ROS_DOMAIN_ID=2`**, wie der Create3.
- `real_rviz.launch.py` muss **immer** laufen, beim Kartieren und bei der
  Navigation. Es veröffentlicht die Transformation `base_link → lidar_link`,
  einschließlich der 180°-Drehung des Lidars.
- **Nie** `mapping.launch.py` und `real_autonomy.launch.py` gleichzeitig starten.

---

## 0. Einmalige Einrichtung

### Laptop

Docker ohne sudo erlauben, danach einmal ab- und wieder anmelden:

```bash
sudo usermod -aG docker $USER
```

Image bauen und Workspace kompilieren:

```bash
cd ~/imdl-main/docker
./run.sh --build        # baut das Humble-Image und startet den Container
ws_build                # im Container: baut das Workspace nach build_humble/ und install_humble/
```

### Jetson

Domain dauerhaft setzen:

```bash
echo 'export ROS_DOMAIN_ID=2' >> ~/.bashrc
source ~/.bashrc
```

Teleop installieren, falls es fehlt:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

---

## 1. Container starten (jedes Mal)

Für **jedes** Laptop-Terminal, das ROS braucht:

```bash
cd ~/imdl-main/docker && ./run.sh
```

- Das erste Terminal startet den Container, jedes weitere öffnet eine
  zusätzliche Shell im selben Container.
- Domain, Humble und das Workspace sind automatisch geladen.
- Karten liegen im Container unter `/root/maps` und auf dem Laptop unter `~/maps`.
- Nach `apt install` im Container ist das Paket beim nächsten Start wieder weg.
  Dauerhaft: Paket ins `docker/Dockerfile` eintragen und `./run.sh --build`.

Verbindung prüfen, im Container:

```bash
ros2 topic list | grep Team2      # Create3-Topics müssen sichtbar sein
```

---

## 2. Karte erstellen (SLAM)

### 2.1 Roboter an die Startposition stellen

Die Startposition und die Blickrichtung (Bumper = vorne) beim SLAM-Start
werden der Ursprung **(0, 0, yaw 0)** der Karte. Markiere sie auf dem Boden.

### 2.2 Jetson, SSH-Terminal 1: SLAM starten

```bash
ros2 launch slam mapping.launch.py
```

### 2.3 Laptop, Container-Terminal 1: RViz

```bash
ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359
```

- `model_visual_yaw:=0.0` sorgt dafür, dass das Robotermodell richtig herum
  angezeigt wird.
- `lidar_yaw:=3.14159265359` gibt an, dass der Lidar um 180° gedreht montiert ist.
- Im Map-Display das **Color Scheme auf `raw`** stellen. Der AMD-Treiber
  zeigt sonst fehlerhafte Farben an.
- Falls RViz trotzdem falsch zeichnet:
  `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch imdl_simulation real_rviz.launch.py ...`

Kontrolle: Fixed Frame `base_link`, ein Hindernis vor den Bumper stellen.
Die Scanpunkte müssen vorne erscheinen, an der roten x-Achse.

### 2.4 Jetson, SSH-Terminal 2: Roboter fahren

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Team2/cmd_vel
```

```
   u   i   o      i = vorwärts      , = rückwärts
   j   k   l      j / l = drehen    k = Stopp
   m   ,   .      u o m . = Kurven

   q / z  alles schneller / langsamer
   w / x  nur geradeaus schneller / langsamer
   e / c  nur Drehung schneller / langsamer
```

- Zuerst etwa 6× `z` drücken, damit er langsam fährt. Besonders langsam drehen.
- Die Taste gedrückt halten. Das Terminal muss dabei aktiv sein.
- Kurz anhalten, bevor du die Richtung wechselst. Am Ende zurück zum Startpunkt.
- Steht der Roboter im Dock, zuerst:
  `ros2 action send_goal /Team2/undock irobot_create_msgs/action/Undock "{}"`

### 2.5 Laptop, Container-Terminal 2: Karte speichern

Solange SLAM noch läuft:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/meine_karte
```

Es entstehen `~/maps/meine_karte.yaml` und `~/maps/meine_karte.pgm` auf dem
Laptop. Danach `mapping.launch.py` auf dem Jetson mit Ctrl-C beenden.

Neu anfangen: `mapping.launch.py` beenden, den Roboter an die Startmarkierung
stellen und neu starten.

---

## 3. Zu Koordinaten fahren (Navigation)

Die Koordinaten beziehen sich auf die Karte: (0, 0) ist die SLAM-Startposition,
+x die damalige Blickrichtung des Roboters, +y links davon. Yaw in Radiant
gegen den Uhrzeigersinn (0 = +x, 1.5708 = +y, 3.1416 = -x).
Koordinaten eines Kartenpunkts ablesen: in der RViz-Werkzeugleiste über „+“ das
Tool „Publish Point“ hinzufügen, auf die Karte klicken und in einem
Container-Terminal `ros2 topic echo /clicked_point` ausführen.

### 3.1 Jetson: nur Lidar starten

Kein SLAM und kein `odom_tf_republisher`, das übernimmt der Laptop:

```bash
ros2 launch slam lidar.launch.py
```

### 3.2 Laptop, Container: Mission anlegen

Die Mission legt die Startpose A und die Karte fest. Steht der Roboter wieder an
der SLAM-Startmarkierung, ist A = `0 0 0`:

```bash
ros2 run imdl_autonomy prepare_trial --task task1 --mission-id test01 \
  --a 0.0 0.0 0.0 \
  --b 1.0 0.5 0.0 \
  --map-yaml /root/maps/meine_karte.yaml \
  --output /root/maps/test01.yaml --force
```

`--b x y yaw` ist das Ziel für den Task-1-Solver. Das Tool prüft, ob A und B
frei auf der Karte liegen.

### 3.3 Laptop, Container-Terminal 1: RViz

```bash
ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359
```

### 3.4 Laptop, Container-Terminal 2: AMCL + Nav2

Noch ohne Mission:

```bash
ros2 launch imdl_autonomy real_autonomy.launch.py mission_file:=/root/maps/test01.yaml start_mission:=false
```

In RViz prüfen, ob die Scanpunkte auf den Wänden der Karte liegen. Falls
nicht: mit „2D Pose Estimate“ die echte Roboterposition auf der Karte setzen.

### 3.5 Laptop, Container-Terminal 3: Ziel senden

Variante A, per Klick: in RViz das Werkzeug **„2D Goal Pose“ / „Nav2 Goal“**
wählen, auf das Ziel klicken und in die Zielrichtung ziehen.

Variante B, exakte Koordinaten:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {z: 0.0, w: 1.0}}}}"
```

Die Orientierung ergibt sich aus dem gewünschten yaw: `z = sin(yaw/2)`, `w = cos(yaw/2)`.

| yaw | z | w |
| --- | --- | --- |
| 0 | 0.0 | 1.0 |
| 1.5708 (+y) | 0.7071 | 0.7071 |
| 3.1416 (-x) | 1.0 | 0.0 |
| -1.5708 (-y) | -0.7071 | 0.7071 |

Variante C, Task-1-Solver: fährt von A nach B aus der Mission.

```bash
ros2 launch imdl_autonomy run_task.launch.py mission_file:=/root/maps/test01.yaml use_sim_time:=false
```

---

## 4. Eigener Behavior Tree (Paket `imdl_statemachine`)

Fährt einen festen Ablauf, ohne Missionsdatei und ohne Task-Solver:

```
Mission (Sequence)
 ├── Nav2Ready → GoToActionPoint → DoAction
 └── GoToRouteStart → SlowRoute → Turn
     (der erste Fehler / Timeout bricht die Mission ab)
```

| Datei | Inhalt |
| --- | --- |
| `ros2_ws/src/imdl_statemachine/config/waypoints.yaml` | Startpose, Punkte, langsames Tempo, Wendewinkel |
| `ros2_ws/src/imdl_statemachine/imdl_statemachine/behavior_tree.py` | Blätter (py_trees) und `build_tree()`. Die eigene Aktion gehört in `DoAction`. |
| `ros2_ws/src/imdl_autonomy/launch/map_navigation.launch.py` | AMCL + Nav2 mit der gespeicherten Karte, ohne Missionen |

Einmalig im Container bauen (`ws_build` enthält das Paket bereits):

```bash
ws_build
```

### Starten

Jetson, nur Lidar:

```bash
ros2 launch slam lidar.launch.py
```

Container-Terminal 1, RViz:

```bash
ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359
```

Container-Terminal 2, AMCL + Nav2 mit der Karte:

```bash
ros2 launch imdl_autonomy map_navigation.launch.py map:=/root/maps/meine_karte.yaml
```

Container-Terminal 3: Roboter an die SLAM-Startmarkierung stellen, dann:

```bash
ros2 run imdl_statemachine behavior_tree
# andere Punkte-Datei:
ros2 run imdl_statemachine behavior_tree --ros-args -p waypoints_file:=/root/maps/meine_punkte.yaml
```

- Ctrl-C bricht die laufende Fahrt ab (Nav2-Goal wird gecancelt).
- Das laufende Blatt zeigt `ros2 topic echo /behavior_tree/state`.
- Das langsame Tempo wird über `/speed_limit` gesetzt und danach wieder aufgehoben.
- Nach Änderungen an `waypoints.yaml` oder `behavior_tree.py` ist kein Build
  nötig (`--symlink-install`), nur neu starten.

---

## Not-Stopp

1. Ctrl-C im Terminal, das das Ziel gesendet hat bzw. im Solver.
2. Ctrl-C bei `real_autonomy.launch.py`.
3. Zusätzlich einen Nullbefehl senden:
   ```bash
   ros2 topic pub --rate 10 /Team2/cmd_vel geometry_msgs/msg/Twist "{}"
   ```

---

## Checks und Fehlersuche

| Problem | Prüfen / Lösung |
| --- | --- |
| Container sieht keine Topics | `echo $ROS_DOMAIN_ID` überall `2`? Danach `ros2 daemon stop`. Firewall: `sudo ufw allow from 192.168.178.0/24` |
| `Message Filter dropping message ... lidar_link` | Läuft `real_rviz.launch.py`? `ros2 run tf2_ros tf2_echo base_link lidar_link` |
| Karte verschmiert / springt | Nur ein TF-Sender pro Frame? `ros2 run tf2_ros tf2_monitor map base_link`. Langsamer fahren. |
| Roboter fährt nicht mit teleop | teleop auf dem Jetson starten. Im Dock? Erst undocken. |
| Odom fehlt | `ros2 topic hz /Team2/odom` |
| Scan fehlt | `ros2 topic hz /scan_raw` und `ros2 topic hz /scan` auf dem Jetson |
| Uhren | `ssh jetson@192.168.178.138 'date +%s.%N'; date +%s.%N` → Unterschied < 0.1 s |
| Nav2 nimmt kein Ziel | `ros2 lifecycle get /bt_navigator` muss `active` sein |
| RViz-Warnung zu QoS bei `/Team2/odom` | Odometry-Display: Reliability Policy = Best Effort |
