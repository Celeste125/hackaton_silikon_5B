# Task-Solver-Logik fuer IMDL Create3

> Architekturhinweis: Die produktive Neuimplementierung liegt im separaten
> ``-Stack. Konkrete Dateien, Startbefehle, Schnittstellen und der ehrliche
> Implementierungs- und Teststatus stehen in `docs/AUTONOMY_STACK_DE.md`.

> Du willst direkt programmieren? Beginne mit `docs/solver_coding_guide_de.md`. Dort stehen die konkreten Solver-Dateien, Klassen, Methoden sowie Build- und Startbefehle. Dieses Dokument hier erklaert den fachlichen Hintergrund.

Dieses Dokument erklaert, wie die Task-Solver professionell gedacht werden sollten. Es ist bewusst auf Deutsch und etwas grundlegend geschrieben, weil die wichtigste Idee nicht "mehr Code" ist, sondern eine saubere Denkweise: Der Roboter darf nicht blind Befehle abspielen, sondern muss staendig wissen, was er sieht, wo er ist, was sein naechster sinnvoller Schritt ist und wann er abbrechen oder neu planen muss.

## 1. Grundidee

Ein guter Task-Solver ist keine lange Liste von `cmd_vel`-Kommandos wie "fahre 2 Sekunden geradeaus, drehe 1 Sekunde". Das funktioniert nur, wenn die Welt perfekt gleich bleibt. In der echten Welt sind aber Batterie, Reibung, Boden, Sensorrauschen, kleine Kollisionen, Latenz und Startposition nie exakt gleich.

Ein robuster Solver besteht deshalb aus mehreren Schichten:

1. **Mission Manager**
   Entscheidet, welche Aufgabe gerade laeuft und welcher Zustand als naechstes kommt.

2. **Navigation Adapter**
   Kapselt Nav2 oder eine lokale Fahrstrategie. Der Task-Code soll nicht direkt wild `/cmd_vel` schreiben.

3. **Perception**
   Erkennt farbige Bloecke, schaetzt deren Pose und meldet Confidence.

4. **Object Registry**
   Speichert erkannte Objekte ueber Zeit, filtert Ausreisser und verwirft alte Daten.

5. **Safety Layer**
   Stoppt bei stale Sensoren, Hindernissen, TF-Problemen, Kollisionen oder unplausiblen Daten.

6. **Manipulation Adapter**
   Simuliert oder steuert spaeter Pick/Place. Die Task-Logik sollte nicht wissen muessen, ob Pick/Place echt oder simuliert ist.

Die wichtigste Regel:

> Die Mission sagt **was** passieren soll. Navigation, Perception und Safety entscheiden **wie** es sicher passiert.

## 2. Sensoren und was sie wirklich bedeuten

### 2.1 Lidar

Der Lidar liefert Entfernungen in einem Kreis oder Halbkreis um den Roboter. Er sagt:

- Da ist etwas in Richtung Winkel `theta`.
- Es ist ungefaehr `r` Meter entfernt.

Der Lidar sagt nicht:

- Welche Farbe ein Objekt hat.
- Ob es ein Block, eine Wand, ein Bein, ein Kabel oder ein Zylinder ist.
- Ob ein Objekt niedrig genug ist, um ignoriert zu werden.

Fuer eure Arena ist wichtig:

- Die grossen zylindrischen Hindernisse mit ca. 23 cm Durchmesser sollten vom Lidar erkannt werden.
- Waende werden vom Lidar erkannt.
- Farbige kleine Bloecke mit ca. 3.5 cm Kantenlaenge koennen je nach Hoehe, Position und Lidar-Montage unzuverlaessig sein.
- Weisse, graue und dunkle Flaechen in RViz sind eine Darstellung von Lidar-/SLAM-Daten. Sie sind keine vom Sensor erkannten Objektfarben.

Professionelle Nutzung:

- Lidar fuer **Kollisionsvermeidung**, **SLAM**, **Costmap** und **Wand-/Obstacle-Abstand**.
- Nicht fuer Farblogik.
- Nicht ungefiltert als Wahrheit behandeln.

Typische Fallen:

- Selbstteile des Roboters werden gesehen, z.B. Kamera, Halterung oder Gehaeusekante.
- Sehr nahe Messungen koennen Rauschen sein.
- Glas, schwarze Oberflaechen oder schräge Flaechen koennen fehlerhaft sein.
- Wenn der Lidar zu hoch oder zu niedrig ist, sieht er andere Dinge als erwartet.

Deshalb braucht man Filter:

- Werte unter einem `self_filter` ignorieren, z.B. alles unter 0.15-0.20 m.
- Ungueltige Werte `inf`, `nan`, `0.0` ignorieren.
- Sektoren bilden: vorne, vorne-links, vorne-rechts, links, rechts.
- Nicht wegen eines einzelnen Punktes hart reagieren, sondern wegen stabiler Hindernis-Cluster.

### 2.2 Kamera

Die Kamera liefert Farbe und optional Tiefe. Sie ist fuer farbige Bloecke viel wichtiger als der Lidar.

Sie kann:

- Rot, Gruen, Blau, Gelb usw. erkennen.
- Pixelposition im Bild bestimmen.
- Mit Depth eine 3D-Position schaetzen.

Sie kann nicht automatisch:

- Zwischen echtem Objekt und farbigem Lichtreflex unterscheiden.
- Mit schlechten HSV-Werten stabil arbeiten.
- Ein Objekt hinter einem Hindernis sehen.
- Sicher sagen, dass ein Objekt nicht existiert, nur weil es gerade nicht im Bild ist.

Professionelle Nutzung:

- Kamera findet farbige Bloecke.
- Depth gibt Entfernung.
- TF transformiert die Pose von `camera_color_optical_frame` nach `map`.
- Object Registry bestaetigt Objekte ueber mehrere Frames.

Typische Fallen:

- Schatten veraendern Value und Saturation.
- Rot kann wegen Hue-Wraparound zwei Bereiche brauchen.
- Kleine Bloecke sind nur wenige Pixel gross.
- Depth am Rand eines kleinen Blocks kann Hintergrundtiefe statt Blocktiefe liefern.

Gute Filter:

- HSV-Maske pro Farbe.
- Morphology: Opening gegen Rauschen, Closing gegen Loecher.
- Mindestflaeche und Maximalflaeche.
- Rechteck-/Kontur-Plausibilitaet.
- Depth-Fenster um Objektzentrum, z.B. Median aus 5x5 oder 7x7 Pixeln.
- Mehrere Bestaetigungen ueber Zeit.

### 2.3 Odometry

Odometry sagt, wie sich der Roboter nach eigener Schaetzung bewegt hat.

Sie ist gut fuer kurze Bewegungen, aber driftet ueber Zeit. Wenn der Roboter rutscht oder anstoesst, kann Odometry falsch werden.

Professionell:

- Odometry ist kurzzeitig nuetzlich.
- Fuer Kartenposition braucht man SLAM oder AMCL.
- Missionserfolg nie nur ueber "ich habe X Sekunden gefahren" bestimmen, sondern ueber Pose-Toleranz.

### 2.4 IMU

Die IMU misst Drehung und Beschleunigung. Sie ist nuetzlich fuer:

- Stabilere Orientierung.
- Erkennen von starken Stoessen.
- Erkennen von Kippen oder Anheben.

Aber:

- IMU alleine gibt keine gute absolute Position.
- Gyro driftet.

### 2.5 Hazard, Bumper, Cliff, Wheel Status

Create3 hat viele Safety-Topics. Fuer professionelle Task-Solver sollten diese nicht ignoriert werden.

Beispiele:

- `/Team2/hazard_detection`
- `/Team2/cliff_intensity`
- `/Team2/slip_status`
- `/Team2/wheel_status`
- `/Team2/kidnap_status`
- `/Team2/stop_status`

Diese Topics sind fuer Recovery wichtig:

- Wenn Bumper/Hazard aktiv: sofort stoppen, kurz zurueck oder seitlich drehen, dann neu planen.
- Wenn Kidnap erkannt: Mission pausieren, neu lokalisieren.
- Wenn Wheel Slip: Geschwindigkeit reduzieren.

## 3. Frames und TF

Frames sind Koordinatensysteme. Ohne saubere Frames faehrt ein Roboter fast sicher falsch.

Wichtige Frames:

- `map`: stabile Weltkarte.
- `odom`: lokale Odometry-Welt, driftet langsam.
- `base_link`: Zentrum/Koerper des Roboters.
- `lidar_link`: Lidar-Position auf dem Roboter.
- `camera_link`: Kameramodul.
- `camera_color_optical_frame`: optischer Kamera-Frame, wichtig fuer Bildpunkte.

Regel:

> Alles, was fuer Navigation langfristig wichtig ist, sollte in `map` oder `odom` transformiert werden. Alles, was aus Kamera/Lidar kommt, kommt zuerst in Sensorframes.

Beispiel Objekt:

1. Kamera erkennt roten Block bei Pixel `(u, v)`.
2. Depth liefert Entfernung `z`.
3. Pixel wird zu 3D-Punkt in `camera_color_optical_frame`.
4. TF transformiert Punkt nach `map`.
5. Object Registry speichert `red_block` mit Pose in `map`.

Gefahr:

Wenn ein Objekt eigentlich in Kamera-Koordinaten ist, aber als `map` interpretiert wird, faehrt der Roboter voellig falsch. Deshalb muss jede Detektion einen `frame_id` und Timestamp haben.

## 4. Zeit und Stale Data

Ein Roboter darf nicht mit alten Daten fahren.

Jeder Solver sollte pruefen:

- Ist `/scan` frisch? Z.B. juenger als 0.5-1.0 s.
- Ist `/odom` frisch?
- Ist TF verfuegbar?
- Ist die Objektpose frisch? Z.B. juenger als 2-5 s, je nach Aufgabe.
- Ist die Kamera aktuell?

Wenn Daten alt sind:

1. Stoppen.
2. Kurz warten.
3. Sensorstatus loggen.
4. Wenn Timeout: Mission in `FAILED_SENSOR_STALE`.

Das ist kein Luxus. Das ist Safety.

## 5. Object Registry

Die Object Registry ist ein Zwischenspeicher fuer erkannte Objekte.

Warum braucht man sie?

Kamera-Erkennung ist verrauscht. Ein Objekt kann fuer einen Frame falsch erkannt werden. Oder es verschwindet kurz, weil der Roboter wackelt.

Ein Track sollte speichern:

- Farbe oder Klasse.
- Pose in `map`.
- Confidence.
- Anzahl Bestaetigungen.
- Letzter Timestamp.
- Beobachtete Pixelgroesse oder Depth.
- Optional Kovarianz/Unsicherheit.

Ein Objekt wird erst "gueltig", wenn:

- Es mindestens `N` mal gesehen wurde, z.B. 3 Frames.
- Die Posen nah beieinander liegen, z.B. weniger als 8 cm Abstand.
- Die Depth plausibel ist.
- Die Flaeche plausibel ist.

Ein Objekt wird verworfen, wenn:

- Es zu alt ist.
- Neue Messungen zu weit weg sind.
- Es ausserhalb der Arena liegt.
- Es in einer Wand oder in einem Hindernis liegt.

Einfache professionelle Logik:

```text
fuer jede neue Detektion:
  transformiere nach map
  finde bestehenden Track gleicher Farbe in der Naehe
  wenn Track gefunden:
    pose = gewichteter Mittelwert
    confidence steigt
    timestamp aktualisieren
  sonst:
    neuen Track mit niedriger confidence erstellen

regelmaessig:
  alte Tracks entfernen
  Tracks mit genug confidence als "confirmed" markieren
```

## 6. Navigation: global und lokal

### 6.1 Global Planning

Global Planning plant einen Weg durch die Karte. Dafuer ist Nav2 geeignet.

Inputs:

- Zielpose.
- Karte.
- Costmap.
- Robot-Footprint.

Output:

- Pfad.

Gut fuer:

- Um Waende und bekannte Hindernisse herumfahren.
- Replanning, wenn etwas blockiert.

Nicht gut alleine fuer:

- Sehr enge, dynamische Situationen.
- Objekte, die nur live per Lidar auftauchen, wenn Costmap falsch konfiguriert ist.

### 6.2 Local Control

Der lokale Controller folgt dem Pfad und reagiert auf lokale Hindernisse.

Wichtige Parameter:

- Maximalgeschwindigkeit.
- Rotationsgeschwindigkeit.
- Inflation radius.
- Obstacle range.
- Raytrace range.
- Goal tolerance.
- Robot footprint.

Fuer kleine Arenen:

- Lieber langsamer und stabiler.
- Kleine Goal-Toleranzen nicht uebertreiben.
- Hindernisse brauchen genug Inflation, sonst schneidet der Roboter Kurven und kollidiert.

### 6.3 Reactive Recovery

Wenn Nav2 blockiert:

1. Stop.
2. Costmap clear.
3. Kleine Rotation, um Lidaransicht zu verbessern.
4. Alternative Approach-Pose probieren.
5. Wenn Bumper/Hazard: kurze Rueckwaertsbewegung nur kontrolliert und begrenzt.
6. Replan.

Wichtig:

Rueckwaertsfahren ist gefaehrlich, wenn man hinten keine gute Sensorik hat. Deshalb sollte Recovery bevorzugt drehen und seitlich/diagonal ausweichen, statt lange rueckwaerts zu fahren.

## 7. Hindernisse, Bloecke und Farben

### 7.1 Zylinder-Hindernisse

Die 23 cm Zylinder sind echte Hindernisse.

Fuer Navigation:

- Lidar erkennt sie.
- Costmap muss sie als obstacle aufnehmen.
- Inflation sollte groesser als Roboter-Sicherheitsabstand sein.

Strategie:

- Niemals direkt auf den Zylinder zufahren.
- Wenn vorne blockiert: Richtung mit groesserem Freiraum waehlen.
- Wenn beide Seiten eng: Stop, neu planen, eventuell Rueckzug nur kurz.

### 7.2 Kleine farbige Bloecke

Die 3.5 cm Bloecke haben Collision, sind aber fuer Lidar evtl. zu klein oder auf falscher Hoehe.

Fuer Task-Logik:

- Bloecke sind Zielobjekte, keine normalen Hindernisse.
- Kamera erkennt Farbe.
- Depth schaetzt Pose.
- Navigation faehrt nicht in den Block hinein, sondern zu einer Approach-Pose davor.

### 7.3 Weiss, Grau und Dunkel in der Lidar-/SLAM-Anzeige

Diese Farben beziehen sich auf die Karten- und Lidar-Darstellung in RViz, nicht auf eine Farberkennung der Kamera und nicht auf die reale Farbe eines Objekts.

Bei einer typischen ROS-`OccupancyGrid`-Karte bedeutet die Darstellung:

- Weiss: als frei beobachteter Bereich (`occupancy` nahe `0`).
- Dunkel oder Schwarz: als belegt erkannter Bereich (`occupancy` nahe `100`), zum Beispiel Wand oder Hindernis.
- Grau: unbekannter beziehungsweise noch nicht beobachteter Bereich (`occupancy = -1`).

Je nach gewaehltem RViz-Color-Scheme kann die exakte Helligkeit anders aussehen. Entscheidend sind die numerischen Occupancy-Werte, nicht die Bildschirmfarbe.

Die einzelnen Punkte eines `LaserScan` sind aktuelle Entfernungsmessungen. RViz kann sie zum Beispiel rot, weiss oder nach Intensitaet einfaerben. Auch diese Farbe ist nur Visualisierung. Ein normaler 2D-Lidar liefert daraus keine zuverlaessige Aussage wie "dieses Objekt ist weiss" oder "dieses Objekt ist dunkel".

Fuer den Task Solver folgt daraus:

- Navigation verwendet freie, belegte und unbekannte Zellen der Costmap.
- Aktuelle Scan-Punkte aktualisieren die lokale Obstacle-Layer.
- Die reale Farbe eines Task-Blocks kommt ausschliesslich aus der Kamera-Pipeline.
- Ein dunkler Kartenbereich ist ein Navigationshindernis, aber keine erkannte Farbklasse.

## 8. Standard-Zustandsmaschine fuer alle Tasks

Ein guter Solver laeuft als State Machine.

Empfohlene States:

```text
IDLE
PRECHECK
LOCALIZE
PLAN
NAVIGATE
SEARCH
APPROACH_OBJECT
ALIGN
PICK
TRANSPORT
PLACE
VERIFY
RECOVERY
DONE
FAILED
```

### PRECHECK

Pruefen:

- Gibt es `/scan`?
- Gibt es `/odom`?
- Gibt es TF `odom -> base_link`?
- Gibt es Kamera-Topics, falls Aufgabe Objekte braucht?
- Ist die Arena/Map geladen?
- Ist Startpose plausibel?

### LOCALIZE

Wenn SLAM/AMCL:

- Warten, bis `map -> odom` existiert.
- Roboter ggf. langsam drehen, um Lidar mehr Sicht zu geben.

### PLAN

Ziel bestimmen:

- Direktziel.
- Viewpoint.
- Approach-Pose.
- Drop-Zone.

### NAVIGATE

Nav2-Ziel senden.

Erfolg:

- Action success.
- Pose innerhalb Toleranz.
- Kein Safety-Fehler.

Fehler:

- Timeout.
- Blockiert.
- Costmap leer/falsch.
- TF fehlt.

### SEARCH

Wenn Objekt unbekannt:

- Viewpoints erzeugen.
- Naechsten Viewpoint waehlen.
- Kamera drehen/fahren.
- Registry pruefen.

### APPROACH_OBJECT

Nicht zum Objektzentrum fahren. Stattdessen:

- Approach-Pose in Abstand `d` vor Objekt berechnen.
- Roboter schaut zum Objekt.
- Abstand z.B. 15-25 cm, je nach Manipulator/Kamera.

Formel:

```text
Richtung = norm(robot_pose - object_pose)
approach = object_pose + Richtung * approach_distance
yaw = angle_to_object
```

### ALIGN

Feinausrichtung:

- Objekt im Bildzentrum halten.
- Kleine Winkelkorrekturen.
- Depth pruefen.
- Nicht schnell fahren.

### PICK / PLACE

Fuer Milestone-Simulation:

- Pick kann simuliert sein.
- Wichtig ist eine klare Schnittstelle:

```text
pick(object_id) -> success/failure
place(zone_id) -> success/failure
```

Spaeter kann dieselbe Schnittstelle echte Manipulation nutzen.

### RECOVERY

Recovery darf nicht chaotisch sein.

Saubere Reihenfolge:

1. Stop.
2. Loggen: warum Recovery?
3. Wenn Sensor stale: warten.
4. Wenn Obstacle vorne: kleine Rotation zur freieren Seite.
5. Wenn Nav2 blockiert: Costmap clear + Replan.
6. Wenn Objekt verloren: Search neu starten.
7. Maximal `N` Versuche, dann FAILED.

## 9. Task 1: Punktnavigation

Ziel:

Der Roboter faehrt autonom zu vorgegebenen Punkten.

Logik:

1. PRECHECK.
2. Zielpose aus Task-Datei laden.
3. Nav2 goal senden.
4. Warten bis success oder timeout.
5. Ergebnis pruefen.

Wichtig:

- Keine festen Fahrzeiten.
- Ziel muss innerhalb Arena liegen.
- Toleranz z.B. 5-10 cm.
- Bei Fehler: replan, nicht blind weiter.

Pseudocode:

```text
for goal in task1_goals:
  result = navigate(goal)
  if result failed:
    recover()
    retry goal
  if retries exhausted:
    fail
done
```

## 10. Task 2: Navigation mit Hindernissen

Ziel:

Zu drei Punkten fahren und Hindernisse vermeiden.

Wichtig:

- Hindernisse koennen an verschiedenen Orten sein.
- Der Solver darf nicht auf feste freie Wege optimiert sein.

Strategie:

1. Lidar und Costmap aktiv.
2. Ziele in sinnvoller Reihenfolge waehlen.
3. Nav2 plant global.
4. Local controller vermeidet live Hindernisse.
5. Recovery, wenn blockiert.

Wenn Hindernis direkt vorne:

- Nicht lange rueckwaerts.
- Stop.
- Freien Sektor links/rechts bestimmen.
- Zur freieren Seite langsam drehen.
- Replan.

Gute lokale Logik:

```text
front = min scan -25 bis +25 grad
left = min scan +30 bis +90 grad
right = min scan -90 bis -30 grad

if front < safety_distance:
  stop
  if left > right:
    rotate_left_slowly
  else:
    rotate_right_slowly
  replan
```

Aber:

Das sollte nur Recovery sein. Normalerweise soll Nav2 fahren.

## 11. Task 3: Bekanntes Objekt aufnehmen/transportieren

Ziel:

Objektpose ist bekannt oder schnell erkennbar. Roboter faehrt hin, richtet sich aus, simuliert Pick, faehrt zur Drop-Zone, simuliert Place.

Logik:

1. Objektpose aus Task-Datei oder Registry.
2. Approach-Pose berechnen.
3. Nav2 zu Approach-Pose.
4. Kamera bestaetigt Objekt.
5. ALIGN.
6. PICK.
7. Nav2 zur Drop-Zone.
8. PLACE.
9. VERIFY.

Wichtig:

- Nicht auf Objektzentrum navigieren.
- Nicht picken, wenn Kamera das Objekt nicht bestaetigt.
- Wenn Objekt nicht mehr da: Search oder fail mit klarem Grund.

## 12. Task 4: Unbekanntes Objekt suchen

Ziel:

Objektposition ist unbekannt. Roboter muss suchen.

Professionelle Strategie:

Active Visual Search.

Idee:

Der Roboter waehlt Viewpoints so, dass er mit wenig Fahrzeit moeglichst viel neue Sicht bekommt.

Ein Viewpoint ist ein Ort + Blickrichtung.

Score:

```text
score = erwartete_sichtbarkeit - fahrkosten - risiko
```

Gute Viewpoints:

- Freie Position.
- Nicht zu nah an Wand/Hindernis.
- Kamera schaut in noch nicht gesehene Bereiche.
- Fahrweg kurz.

Einfache robuste Version:

1. Erzeuge 6-10 Viewpoints in der Arena.
2. Entferne Viewpoints in Hindernissen.
3. Sortiere nach:
   - Entfernung vom Roboter.
   - Sichtabdeckung.
   - Sicherheitsabstand.
4. Fahre zum besten Viewpoint.
5. Drehe langsam Kamera/Roboter.
6. Wenn Objekt erkannt und bestaetigt: Task 3 Logik.
7. Sonst naechster Viewpoint.

## 13. Task 5: Mehrere Objekte

Ziel:

Mehrere Objekte in Reihenfolge oder effizienter Reihenfolge bearbeiten.

Wenn Reihenfolge vorgegeben:

- Reihenfolge strikt einhalten.

Wenn nicht vorgegeben:

- Naechstes erreichbares Objekt waehlen.
- Nicht nur Luftlinie, sondern Pfadkosten nutzen.

Logik:

```text
remaining = required_objects
while remaining:
  update registry
  if next object unknown:
    search object
  approach object
  pick
  navigate drop
  place
  remove object from remaining
done
```

Wichtig:

- Jeder Pick/Place-Schritt braucht Retry.
- Registry darf bereits gefundene Objekte cachen.
- Wenn ein Objekt verloren geht: nicht alles resetten, sondern nur dieses Objekt neu suchen.

## 14. Safety-Regeln

Mindestregeln:

- Wenn `/scan` stale: stop.
- Wenn `/odom` stale: stop.
- Wenn TF fehlt: stop.
- Wenn front obstacle sehr nah: stop.
- Wenn bumper/hazard: stop + recovery.
- Wenn Mission zu lange dauert: fail statt endlos fahren.
- Wenn Pose ausserhalb Arena: stop.
- Wenn Objektpose ausserhalb Arena: Detektion verwerfen.

Empfohlene zentrale Safety-Parameter:

```yaml
max_linear_speed: 0.12
max_angular_speed: 0.6
front_stop_distance: 0.22
side_stop_distance: 0.12
scan_timeout_sec: 0.75
odom_timeout_sec: 1.0
object_timeout_sec: 5.0
max_retries_per_goal: 3
```

## 15. Logging und Debugging

Gute Logs beantworten:

- Was ist mein aktueller State?
- Was ist mein Ziel?
- Warum habe ich gestoppt?
- Welche Sensoren sind frisch?
- Welche Hindernisse sehe ich?
- Welche Objektpose glaube ich?
- Welche Recovery wurde gewaehlt?

Gute RViz Marker:

- Aktuelles Ziel.
- Naechste Approach-Pose.
- Geplanter Pfad.
- Erkannte Objekte mit Confidence.
- Search-Viewpoints.
- Safety-Zonen vorne/links/rechts.
- Mission State als Text.

Beispiel Log:

```text
[MISSION] state=NAVIGATE goal=B pose=(0.42, -0.31, 1.57)
[SAFETY] front_clearance=0.18m below 0.22m -> STOP
[RECOVERY] rotate_left because left=0.51m right=0.24m
[OBJECT] red confirmed pose=(0.33, 0.12) confidence=0.82 age=0.4s
```

## 16. Simulation vs echte Hardware

Simulation ist nuetzlich, aber nicht Wahrheit.

Simulation kann:

- Welt laden.
- Lidar realistisch gegen Gazebo-Geometrie raycasten.
- Kamera/Depth simulieren.
- Kollisionen testen.
- Solver-Logik schnell testen.

Simulation kann schlechter:

- Lichtbedingungen.
- Sensorrauschen.
- Rutschender Boden.
- USB-Latenz.
- Falsche Halterungen.
- Echte Reflexionen.

Deshalb:

1. Erst Logik in Real-like Simulation testen.
2. Dann mit echtem Lidar/Kamera stationaer testen.
3. Dann langsam fahrend testen.
4. Dann komplette Mission.

## 17. Was man vermeiden sollte

Nicht professionell:

- Lange feste `cmd_vel`-Sequenzen.
- Navigation und Perception und Pick/Place alles in einer Monster-Funktion.
- Objektpositionen ohne TF annehmen.
- Auf Kamera-Einzelbild vertrauen.
- Hindernisvermeidung nur fuer eine bekannte Welt hardcoden.
- Endlos-Recovery ohne klaren Abbruch.
- Keine Sensor-Freshness pruefen.
- Nur visuell in RViz vertrauen, ohne Topic-Hz und Logs zu pruefen.

Professionell:

- State Machine.
- Nav2 als Hauptnavigation.
- Object Registry.
- TF konsequent.
- Safety Layer.
- Klare Adapter fuer Navigation und Manipulation.
- Parameter in YAML.
- Tests fuer Geometrie und Recovery-Entscheidungen.

## 18. Empfohlene Implementierungsstruktur

```text
imdl_autonomy/
  mission_manager.py
  navigation_adapter.py
  perception_client.py
  object_registry.py
  manipulation_adapter.py
  safety_monitor.py
  task_solvers/
    task1_solver.py
    task2_solver.py
    task3_solver.py
    task4_solver.py
    task5_solver.py
```

### Mission Manager

Startet Task, verwaltet State Machine, published `/mission_state` und `/task_result`.

### Navigation Adapter

Funktionen:

```text
navigate_to(pose)
cancel()
clear_costmap()
spin(angle)
backup(distance)
```

### Object Registry

Funktionen:

```text
update(detections)
get_confirmed(color)
get_nearest(colors)
remove(object_id)
```

### Safety Monitor

Funktionen:

```text
is_safe_to_move()
front_clearance()
sensor_fresh()
stop_reason()
```

### Manipulation Adapter

Funktionen:

```text
pick(object_id)
place(zone)
has_object()
```

## 19. Minimaler, aber guter Solver-Ablauf

```text
start mission
precheck sensors
while mission not done:
  update safety
  if unsafe:
    stop and recover

  update object registry

  state = current state
  if state == NAVIGATE:
    send nav goal
    wait non-blocking for result

  if state == SEARCH:
    choose next viewpoint
    navigate viewpoint
    scan with camera

  if state == APPROACH:
    compute approach pose
    navigate approach
    align with camera

  if state == PICK:
    call manipulation adapter

  if state == PLACE:
    call manipulation adapter

  publish markers and logs
end
```

## 20. Prioritaeten fuer euren Milestone

Wenn die Zeit knapp ist:

1. Task 1 stabil mit Nav2 oder sehr sauberer lokaler Navigation.
2. Task 2 mit echter Lidar-Costmap und kontrollierter Recovery.
3. Task 3 mit Approach-Pose und simuliertem Pick/Place.
4. Task 4 mit einfacher Viewpoint-Suche.
5. Task 5 mit Registry und Reihenfolge.

Ehrlich fuer eine Demo:

- Lieber eine klare State Machine zeigen als eine zufaellige Bewegung, die einmal funktioniert.
- Lieber gute Marker und Logs zeigen als behaupten, dass alles autonom perfekt ist.
- Lieber Sensor-Freshness und Safety zeigen als schnell aber gefaehrlich fahren.

## 21. Kurzfassung

Der Roboter muss immer diese Fragen beantworten:

1. Wo bin ich?
2. Was sehe ich?
3. Sind meine Sensordaten frisch?
4. Was ist mein Ziel?
5. Ist der direkte Weg sicher?
6. Wenn nicht: welche Recovery ist sinnvoll?
7. Habe ich mein Objekt wirklich bestaetigt?
8. Ist die Mission fertig oder muss ich neu planen?

Wenn der Solver diese Fragen sauber beantwortet, wird er deutlich robuster als eine Liste von Bewegungsbefehlen.
