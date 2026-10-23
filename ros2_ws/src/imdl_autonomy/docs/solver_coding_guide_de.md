# IMDL Solver praktisch programmieren

Dieses Dokument zeigt, an welchen Dateien du arbeitest und wie ein Solver in
diesem Repository wirklich implementiert wird. Die Architektur ist eindeutig:
Es gibt nur den kanonischen Stack ohne parallele Legacy- oder Suffix-Version.

## 1. Die wichtigen Dateien

```text
imdl_autonomy/
  mission_manager.py       startet und tickt genau einen Solver
  navigation_adapter.py    einzige Solver-Schnittstelle zu Nav2
  runtime_context.py       Lokalisierung, Safety, Tracks, Manipulator
  next_best_view.py        Suche und Sichtbarkeitsbewertung
  solvers/
    base.py                gemeinsame Solver-Schnittstelle
    registry.py            task1 ... task5 registrieren
    task1.py ... task5.py  eigentliche Task-Entscheidungen
    search.py              wiederverwendbare aktive Suche
    transport.py           Approach, Pick, Transport, Place
config/
  missions/*.yaml          A/B/C, Karte, Farben und Zeitlimit
  nav2_params.yaml         Planner, Controller und Safety-Geometrie
  solver_params.yaml       missionsweite Zeitlimits und Abstaende
```

Wenn die Reihenfolge einer Aufgabe falsch ist, bearbeitest du
`solvers/taskN.py`. Wenn ein korrektes Ziel schlecht angefahren wird, pruefst
du `navigation_adapter.py`, Behavior Tree und `nav2_params.yaml`. Sensorfilter
gehoeren in Perception, Object Registry oder Safety Monitor, nicht in den
Task-Solver.

## 2. Solver-Vertrag

Jeder Solver erbt von `BaseSolver` und implementiert drei kurze Methoden:

```python
from .base import BaseSolver, SolverStatus


class Task6Solver(BaseSolver):
    def start(self) -> None:
        self.state = 'STARTING'

    def tick(self):
        # Nur einen Zustands-Schritt ausfuehren und sofort zurueckkehren.
        return self.snapshot(SolverStatus.RUNNING)

    def cancel(self) -> None:
        self.context.navigation.cancel()
```

`tick()` laeuft alle 100 ms. Deshalb sind dort verboten:

- `while`-Schleifen, die auf ROS-Daten warten,
- `time.sleep()`,
- eigene Publisher fuer `Twist` oder `cmd_vel`,
- direkte Gazebo-Serviceaufrufe,
- Lesen von Objektpositionen aus einer World-Datei.

Eine Aktion wird in einem Zustand gestartet und in spaeteren Ticks beobachtet.
Task 2 zeigt das kleinste vollstaendige Muster:

```python
if self.state == 'START_B':
    accepted = self.context.navigation.navigate_to(
        self.context.mission.poses['B'],
        controller='RobustMPPI',
        timeout_sec=self.context.navigation_timeout_sec,
    )
    if not accepted:
        return self.fail('NAVIGATION_REJECTED')
    self.state = 'WAIT_B'
    return self.running()

if self.state == 'WAIT_B':
    nav = self.context.navigation.tick()
    if not nav.terminal:
        return self.running()
    if nav.state is NavigationState.SUCCEEDED:
        self.context.navigation.reset()
        self.state = 'START_C'
        return self.running()
    return self.fail(nav.message)
```

Nutze die vorhandenen Hilfsmethoden der echten Solver, statt dieses Beispiel
wortwoertlich zu duplizieren. Entscheidend ist das Muster `START_* -> WAIT_*`.

## 3. Welche Daten der Solver bekommt

`self.context` stellt die freigegebenen Schnittstellen bereit:

| Zugriff | Bedeutung |
| --- | --- |
| `mission.poses['A'/'B'/'C']` | vorgegebene Missionskoordinaten |
| `current_pose()` | aktuelle lokalisierte Pose im `map`-Frame |
| `navigation` | Nav2 ComputePath, Navigate, FollowPath und Spin |
| `objects` | bestaetigte, TF-transformierte Objekttracks |
| `safety` | zentraler Sensor- und Hazard-Zustand |
| `manipulator` | asynchroner Pick/Place-Service |
| `view_planner` | Next-Best-View und Bayes-Suchkarte |
| `visualizer` | Ziele, Kandidaten, Tracks und Status in RViz |

Der Solver liest nicht direkt `/scan`. Lidarwerte werden von Nav2-Costmaps,
Collision Monitor, Localization Guard und Safety Monitor verarbeitet. Weiss,
Grau und Schwarz in einer Occupancy Map sind Kartenwerte, keine Objektfarben:

- weiss: freie Zelle,
- grau: unbekannte Zelle,
- dunkel/schwarz: belegte Zelle.

Farbige Bloecke kommen aus Kamera plus Depth und erscheinen als
`TrackedObject` im `map`-Frame.

## 4. Missionsdateien statt harter Koordinaten

Der Solver darf keine Variante `a` oder `b` abfragen. Beide Launch-Werte
waehlen nur World und YAML. Ein Task-2-Versuch sieht so aus:

```yaml
mission_id: task2_trial_01
task: task2
frame_id: map
coordinate_mode: absolute
obstacle_mode: unknown
map_yaml: ../../maps/arena_unknown.yaml
time_limit_sec: 180.0
poses:
  A: {x: 0.52, y: -0.52, yaw: 3.141593}
  B: {x: -0.50, y: -0.15, yaw: 1.570796}
  C: {x: -0.50, y: 0.50, yaw: 0.0}
```

`mission_spec.py` validiert Pflichtfelder und `mission_geometry.py` prueft, ob
Roboterziele mit dem vollstaendigen Footprint erreichbar sind.

## 5. Einen weiteren Task registrieren

1. Lege `solvers/task6.py` mit `Task6Solver` an.
2. Importiere die Klasse in `solvers/registry.py` und ergaenze `SOLVERS`.
3. Erweitere die erlaubten Tasks in `mission_spec.py` samt Pflichtfeldern.
4. Lege mindestens eine Missions-YAML an.
5. Schreibe State-Machine-Tests mit Mock-Navigation fuer Success, Abort,
   Timeout, Cancel und Retry.
6. Fuehre Build, Unit-Tests und anschliessend Gazebo-Laufzeitpruefungen aus.

## 6. Starten und beobachten

Terminal 1 startet Umgebung, Sensorik, Nav2 und Support-Nodes, aber keine
Mission:

```bash
cd /home/baran/imdl/ros2_ws
source install/setup.bash
ros2 launch imdl_simulation autonomy.launch.py task:=task2 variant:=a
```

Terminal 2 startet nur den Solver:

```bash
cd /home/baran/imdl/ros2_ws
source install/setup.bash
ros2 launch imdl_autonomy run_task.launch.py task:=task2 variant:=a
```

Wichtige Diagnose-Topics:

```bash
ros2 topic echo /mission_state
ros2 topic echo /task_result
ros2 topic echo /performance_metrics
ros2 topic echo /tracked_objects --once
ros2 topic hz /scan
```

Die komplette Architektur, Task-Strategien und der ehrliche Validierungsstand
stehen in `docs/AUTONOMY_STACK_DE.md`.
