# Buffer Feed: autonomes Nachschieben und Laden für Federpuffer

`buffer_feed` steuert eine Vorfördereinheit **ohne den Host im Zeitpfad**. Alles
Zeitkritische (Sensor lesen, Schritte erzeugen, Rampen, Stoppen) macht die MCU
selbst. Die Schritte werden in den Step-Strom des vorhandenen Steppers
eingemischt und laufen deshalb parallel zu normalen Bewegungen desselben
Steppers (Extruder-synchron, Retract, ...).

```
Sensor 1 -> Vorförderer -> Sensor 2 -> Sensor 3 -> Puffer (low / high)
(entry_pin)               (exit_pin)   (gate_pin)   (trigger_pin / stop_pin)
```

## Warum nicht `gcode_button` + `MANUAL_STEPPER`?

Jede Bewegung über G-Code (auch `MANUAL_STEPPER ... SYNC=0`) wird auf der
`print_time` am Ende der Lookahead-Queue des Toolheads eingeplant. Je nach
Druck sind das bis zu 1-2 s Vorlauf. Dazu kommt, dass der Button-Befehl erst
zwischen zwei Druckbefehlen abgearbeitet wird. Bei `buffer_feed` entscheidet
die MCU selbst. Reaktionszeit mit den Standardwerten: ca. 1,5-2,5 ms,
unabhängig von Druck, Bahnplanung und CAN-Last.

## Funktionen

### 1. Nachschieben (Puffer-Sensoren, optional Sensor 3)

* Die MCU fragt den **Trigger-Sensor** (Puffer fast leer) zyklisch ab. Ist er
  entprellt aktiv, werden `distance` mm nachgeschoben. Das Schrittintervall
  wird linear von `start_velocity` auf `velocity` verkleinert (Dauer
  `(velocity-start_velocity)/accel`).
* Der **Stop-Sensor** (Puffer voll) wird vor jedem Schritt geprüft. Er darf
  **überfahren** werden: Beim Auslösen bremst der Stepper mit der Rampe `decel`
  ab (Dauer `(velocity-start_velocity)/decel`) und bleibt dann stehen. Die
  Überfahrstrecke ist also die Bremsstrecke. Ohne Rampe (`accel: 0`) stoppt er
  sofort.
* Der **Gate-Sensor** (`gate_pin`, Sensor 3) ist optional. Ist er konfiguriert,
  wird nur nachgeschoben, solange er ausgelöst ist (Filament ist am Puffer
  angekommen).
* Läuft gerade eine Bewegung des Hosts in Gegenrichtung (Retract), wartet das
  Nachschieben, bis sie beendet ist. Der Dir-Pin wird nie unter einer
  laufenden Bewegung umgeschaltet und vor der nächsten Host-Bewegung
  automatisch wieder passend gesetzt.
* Bleibt der Trigger-Sensor nach `max_runs` Läufen in Folge aktiv (Stau,
  Sensor defekt), stoppt das Nachschieben, es erscheint eine Fehlermeldung und
  `fault_gcode` wird ausgeführt.
* Nach einem Stop über den Stop-Sensor wird erst wieder gestartet, wenn der
  Trigger-Sensor zwischendurch losgelassen wurde.

### 2. Laden (Sensor 1 -> Sensor 2), optional

Konfiguriert man `entry_pin` und `exit_pin`:

1. Löst Sensor 1 aus, läuft der Vorförderer sofort an (Rampe `load_*`) und
   fährt, bis Sensor 2 auslöst.
2. Löst Sensor 2 nicht innerhalb von `load_timeout` aus: Abbruch,
   Fehlermeldung und `load_fault_gcode`.
3. Nach dem Auslösen von Sensor 2 wird `load_clear_distance` gefahren:
   * positiv: weiter in Förderrichtung, die Strecke gilt ab dem Auslösen und
     enthält die Bremsrampe (Wert also größer als die Bremsstrecke wählen),
   * negativ: anhalten und dann diese Strecke zurückfahren,
   * 0: mit Rampe anhalten.
4. Ein neuer Ladevorgang startet erst, wenn Sensor 1 zwischendurch
   losgelassen wurde. Ist Sensor 2 schon bei Sensor-1-Auslösung aktiv, wird
   nichts bewegt.

Das Beladen bis der Puffer voll ist (Taster 1) und das Entladen (Taster 2) sind
bewusst **nicht** in der MCU, sondern nativ in Klipper (Macros) umzusetzen. Dafür
gibt es `BUFFER_FEED_MOVE` (siehe unten) und `config/sample-buffer-feed.cfg`.

## Voraussetzungen

* Firmware-Update der MCU, auf der der Stepper und die Sensoren hängen
  (`make menuconfig`: "Support autonomous filament buffer feeding" ist
  standardmäßig aktiv, außer auf AVR). Host-Klipper und MCU müssen
  zusammenpassen.
* Der Stepper muss "step on both edges" verwenden. Das ist bei
  TMC-Treibern mit Step/Dir der Standard, solange kein großer
  `step_pulse_duration` gesetzt ist. Sonst bricht Klipper beim Start mit einer
  Fehlermeldung ab.
* Stepper und alle Sensoren müssen an derselben MCU hängen.
* Die Sensor-Pins dürfen zusätzlich von `gcode_button` o.ä. benutzt werden.
  Die alte Logik (Extrusionsfaktor anpassen) sollte aber entfernt werden.

## Konfiguration

Siehe `config/sample-buffer-feed.cfg`.

| Option | Default | Bedeutung |
|---|---|---|
| `stepper` | | Config-Sektion des Steppers (`manual_stepper x` oder `extruder_stepper x`) |
| `trigger_pin` | | Sensor "Puffer fast leer" |
| `stop_pin` | | Sensor "Puffer voll" (optional) |
| `gate_pin` | | Sensor 3: Nachschieben nur wenn aktiv (optional) |
| `distance` | | Strecke pro Auslösung (mm) |
| `velocity` | | Fördergeschwindigkeit (mm/s), max. 25000 Schritte/s |
| `accel` | 0 | Beschleunigung (mm/s²), 0 = sofort volle Geschwindigkeit |
| `decel` | `accel` | Bremsrampe, v.a. beim Überfahren des Stop-Sensors (mm/s²) |
| `start_velocity` | 2.0 | Start-/Endgeschwindigkeit der Rampen (mm/s) |
| `poll_interval` | 0.0005 | Abfrageintervall in der MCU (s) |
| `trigger_debounce` | 0.002 | Entprellzeit des Trigger-Sensors (s) |
| `stop_samples` | 2 | Aufeinanderfolgende Abtastungen (je eine pro Schritt) bis der Stop-Sensor gilt |
| `max_runs` | 3 | Läufe bei dauerhaft aktivem Trigger bis zum Fehler (0 = aus) |
| `fill_runs` | 30 | wie `max_runs`, aber für das erste Füllen des leeren Puffers nach Enable, Laden oder manuellem Move (0 = aus) |
| `fault_gcode` | | G-Code bei Nachschub-Fehler |
| `entry_pin` | | Sensor 1: startet das Laden (mit `exit_pin` zusammen) |
| `exit_pin` | | Sensor 2: beendet das Laden |
| `load_velocity` | `velocity` | Ladegeschwindigkeit (mm/s) |
| `load_accel` | `accel` | Beschleunigung beim Laden (mm/s²) |
| `load_decel` | `load_accel` | Bremsrampe beim Laden (mm/s²) |
| `load_start_velocity` | `start_velocity` | Start-/Endgeschwindigkeit beim Laden (mm/s) |
| `load_timeout` | 10 | Zeit (s) bis Sensor 2 auslösen muss, max. 60 |
| `load_clear_distance` | 0 | Strecke nach Sensor 2 (mm); negativ = zurück |
| `entry_debounce` | 0.002 | Entprellzeit von Sensor 1 (s) |
| `exit_samples` | 2 | Abtastungen (je eine pro Schritt) bis Sensor 2 gilt |
| `load_fault_gcode` | | G-Code bei Lade-Timeout |
| `enable` | True | Beim Start automatisch aktivieren |
| `enable_stepper` | True | Treiber beim Aktivieren automatisch einschalten |

Die Rampen sind linear im Schrittintervall und damit eine Annäherung an
konstante Beschleunigung. Die Rampendauer entspricht der Formel oben.

## G-Code-Befehle

* `SET_BUFFER_FEED BUFFER=<name> [ENABLE=0|1] [DISTANCE=] [VELOCITY=]
  [ACCEL=] [DECEL=] [LOAD_VELOCITY=] [LOAD_ACCEL=] [LOAD_DECEL=]
  [LOAD_TIMEOUT=] [LOAD_CLEAR_DISTANCE=]` aktiviert/deaktiviert die
  Automatik (Nachschieben **und** Laden über Sensor 1) und ändert Parameter.
  `ENABLE=1` schaltet auch den Treiber ein, falls `enable_stepper` gesetzt ist.
* `BUFFER_FEED_MOVE BUFFER=<name> [DISTANCE=] [VELOCITY=] [ACCEL=] [DECEL=]
  [STOP=] [STOP_ON=TRIGGER|RELEASE] [WAIT=0|1] [WAIT_TIMEOUT=]` startet eine
  einzelne Bewegung. Der Befehl kehrt sofort zurück (außer mit `WAIT=1`), die
  MCU führt die Bewegung selbstständig aus. Läuft noch eine vorherige
  Bewegung, wird der Befehl mit einer Fehlermeldung abgelehnt.
  * `WAIT=1`: wartet, bis die MCU das Ende der Bewegung meldet (nötig, wenn
    danach ein weiterer Move oder `ENABLE=1` folgt). Wird die Bewegung
    abgebrochen oder dauert sie länger als `WAIT_TIMEOUT` (Default: aus
    Strecke/Geschwindigkeit berechnet), bricht das Makro mit Fehler ab.
  * `DISTANCE`: maximale Strecke. **Negative Werte fahren rückwärts**
    (Entladen). Default: `distance` der Config (vorwärts).
  * `STOP`: Sensor, der die Bewegung beendet (mit Bremsrampe, wie beim
    Nachschieben): `none`, `full` (Puffer voll, `stop_pin`), `low` (Puffer fast
    leer, `trigger_pin`), `entry` (Sensor 1), `exit` (Sensor 2), `gate`
    (Sensor 3). Default: `full` bei Vorwärtsfahrt, wenn ein `stop_pin`
    konfiguriert ist, sonst `none`. Auch `sensor1`/`sensor2`/`sensor3` und
    `buffer_full`/`buffer_low` werden akzeptiert (`0` = none, `1` = full).
  * `STOP_ON`: `TRIGGER` (Default) stoppt, wenn der Sensor auslöst,
    `RELEASE`, wenn er wieder loslässt (z.B. Filament verlässt Sensor 1).
  * Ist der Sensor schon im Stop-Zustand, hält die Bewegung nach wenigen
    Schritten wieder an. Ohne Stop-Sensor läuft sie die volle `DISTANCE`.
  * Beispiele: Laden bis der Puffer voll ist
    `BUFFER_FEED_MOVE BUFFER=feeder1 DISTANCE=600 STOP=full`, Entladen bis
    Sensor 1 frei ist
    `BUFFER_FEED_MOVE BUFFER=feeder1 DISTANCE=-600 STOP=entry STOP_ON=RELEASE`.
* `BUFFER_FEED_LOAD BUFFER=<name> [WAIT=1] [WAIT_TIMEOUT=]` startet den
  Ladevorgang (Sensor 1 -> 2) manuell, z.B. zum Testen. Mit `WAIT=1` führt
  ein Timeout (`load_timeout`) zu einem Makro-Fehler.
* `BUFFER_FEED_WAIT BUFFER=<name> [TIMEOUT=]` wartet auf das Ende einer
  laufenden Bewegung bzw. eines Ladevorgangs.
* `BUFFER_FEED_ABORT BUFFER=<name>` bricht einen laufenden Nachschub oder
  Ladevorgang ab (sofort, ohne Rampe).
* `QUERY_BUFFER_FEED BUFFER=<name>` zeigt Status und den Zustand aller
  Sensoren. Praktisch zum Prüfen der Verdrahtung und der Pin-Polarität.

Status (Moonraker/Macros): `printer["buffer_feed <name>"]` mit `enabled`,
`last_result` (`done`, `stop_sensor`, `aborted`, `fault`, `busy`,
`load_timeout`, `loaded`), `last_steps`, `fault_count`, `load_fault_count`,
`distance`, `velocity`, `accel`, `decel`.

## Wichtige Hinweise

* **Entladen:** Vorher mit `SET_BUFFER_FEED ... ENABLE=0` die Automatik
  abschalten (vor dem `BUFFER_FEED_MOVE`, denn `ENABLE=0` bricht eine laufende
  Bewegung ab). Sonst löst beim Zurückziehen der Puffer-Trigger aus und es wird
  wieder nach vorne geschoben. Manuelle Bewegungen laufen auch bei
  abgeschalteter Automatik. `BUFFER_FEED_MOVE` kehrt sofort zurück: Wer danach
  einen weiteren Move oder `ENABLE=1` ausführen will, nutzt `WAIT=1` (sonst
  meldet die MCU "busy" und der Befehl wird abgelehnt). Beispiel Entladen mit
  Nachfahrstrecke:

      SET_BUFFER_FEED BUFFER=feeder1 ENABLE=0
      BUFFER_FEED_MOVE BUFFER=feeder1 DISTANCE=-2000 STOP=exit STOP_ON=RELEASE WAIT=1
      BUFFER_FEED_MOVE BUFFER=feeder1 DISTANCE=-400 STOP=none WAIT=1
      SET_BUFFER_FEED BUFFER=feeder1 ENABLE=1

  Liegt beim `ENABLE=1` noch Filament an Sensor 1, startet kein Laden; erst
  nach erneutem Einführen (Sensor 1 frei, dann ausgelöst).
* **Abbrechen:** `ENABLE=0` und `BUFFER_FEED_ABORT` stoppen eine laufende
  Bewegung sofort (ohne Rampe). Ein direkt folgender `BUFFER_FEED_MOVE` wird
  angenommen, auch wenn gerade ein automatischer Nachschub lief. Jeder vom
  Host gestartete Move trägt eine Kennung (`tag`); Meldungen anderer Läufe
  (Automatik, abgebrochener Vorgänger) beenden ein `WAIT=1` nicht vorzeitig.
* **Motor aus:** Bei `M84` / `motor_off` / idle_timeout wird die Automatik
  abgeschaltet, damit bei stromlosem Treiber keine falschen Fehler entstehen.
  Sie muss danach wieder mit `SET_BUFFER_FEED ... ENABLE=1` eingeschaltet
  werden (z.B. in `PRINT_START`).
* **Positionsbuchführung:** Die zusätzlichen Schritte sind für den Host
  unsichtbar. Die Host-Position des Steppers bleibt exakt auf die
  Host-Bewegungen bezogen.
* **Auslegung:** Die Fördergeschwindigkeit muss deutlich über dem maximalen
  Verbrauch des Extruders liegen. Wird der Puffer nicht nachgefüllt, greift
  `max_runs`.
* Schrittimpulse von Host-Bewegung und Zusatzschritten werden von der MCU
  nacheinander erzeugt (nie gleichzeitig). Ob der Mindestabstand für den
  verwendeten Treiber immer reicht, ist auf der Hardware zu prüfen.

## Test

`scripts/buffer_feed_sim/run.sh` baut den echten Code aus `src/stepper.c` und
`src/buffer_feed.c` auf dem PC und testet ihn gegen einen simulierten
Scheduler, GPIO und Federpuffer (beide Stepper-Pfade): Rampen, exakte
Schrittzahlen, Regelkreis mit Verbrauch, Parallelbetrieb mit Host-Extrusion,
Retract, Positionsbuchführung, Stop-Sensor mit Bremsrampe, Gate, Laden
(Latenz, Freifahren vor/zurück, Timeout, Wiederanlauf), Abbruch, Fehler und
Re-Enable. Auf echter Hardware ersetzt das den Praxistest nicht.
