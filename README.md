# Torque Tool Network

Three-part proof-of-concept system:

1. `target_monitor_wifi/target_monitor_wifi.ino`
   - Flash one copy to every individual tool ESP32-C3.
   - Change `TOOL_ID` so every physical tool has a unique ID.
   - The tool performs tare, HX711 sampling, MAD filtering, relay pulse control,
     and target cutoff locally.
   - ESP-NOW is used for commands and telemetry.

2. `tool_network_controller/tool_network_controller.ino`
   - Flash this to the ESP32-C3 that remains connected to the laptop by USB.
   - It discovers tool ESP32-C3 units over ESP-NOW.
   - It routes commands to individual tools and forwards their status to the
     laptop over USB Serial.
   - Current table size: 20 tools.

3. `laptop_controller/laptop_controller.py`
   - Runs on the laptop.
   - Uses `pyserial` to communicate with the Tool Network Controller.
   - Includes a command-line interface for initial testing.
   - The `ToolNetworkController` Python class is intended to be imported by a
     future graphical UI.

## Architecture

    Laptop UI / Python
           |
        USB Serial
           |
    Tool Network Controller ESP32-C3
           |
         ESP-NOW
       /    |    \
    TOOL_01 TOOL_02 TOOL_03 ...

Critical load-cell and relay control stays on each individual tool. The laptop
does not need to maintain a real-time wireless control loop.

## Arduino dependencies

Tool firmware:
- ESP32 Arduino core
- HX711 library
- WiFi / ESP-NOW supplied by the ESP32 Arduino core

Controller firmware:
- ESP32 Arduino core
- WiFi / ESP-NOW supplied by the ESP32 Arduino core

Use compatible ESP32 Arduino core versions on both types of ESP32-C3.

## Configure each tool

Before flashing each individual tool, edit:

    const char TOOL_ID[] = "TOOL_01";

Use a unique value such as `TOOL_01`, `TOOL_02`, etc.

Also set the real load-cell calibration factor:

    const double N_PER_COUNT = 0.205;

## Laptop setup

Python 3.10+ recommended.

Install:

    pip install -r requirements.txt

Run:

Windows:

    python laptop_controller.py --port COM5

Linux:

    python laptop_controller.py --port /dev/ttyACM0

macOS:

    python laptop_controller.py --port /dev/cu.usbmodemXXXX

If `--port` is omitted, the program attempts to select or prompt for an
available serial port.

## Laptop commands

    list
    target TOOL_01 100
    stop TOOL_01
    tare TOOL_01
    reset TOOL_01
    loosen TOOL_01
    target-all 100
    stop-all
    tare-all
    reset-all
    loosen-all
    status
    quit

Example:

    target TOOL_03 125

sends a 125 N target to TOOL_03. TOOL_03 then performs the sampling/filtering
and relay control itself.

## USB Serial protocol

The controller emits machine-readable lines such as:

    STATUS,TOOL_01,MONITORING,100.0000,421,398,1
    STATUS,TOOL_02,READY,0.0000,35,0,0

STATUS fields: tool ID, state, target (N), raw counts, stop value (the
MAD-filtered value the tool compares with the target: the window maximum by
default, see `TARGET_DETECTION` in the tool firmware), relay on. Keep the
target field: the dashboard uses it to tell a status from its latest TARGET
apart from an older one, and the parser reads fields by position. (Older
controller firmware also printed a 1 s average before relay on; the laptop
still accepts that.)

STOP: relays off, target cleared, back to IDLE. The tare baseline and the
readings are kept, so a tight bolt keeps showing its load and a TARGET
straight after a STOP re-tightens it against the same zero, with no new tare.
RESET: the same, but it also clears the tare, so the tool needs a TARE first.
Only the on-tool safety stops leave a tool in STOPPED; a TARGET from STOPPED
works too. TARGET is refused by a tool that was never tared, was RESET, or
whose last tare failed.

    EVENT,DISCOVERED,TOOL_03,AA:BB:CC:DD:EE:FF

Every command gets a `SENT` line from the controller and then a reply from
the tool itself (`<state>` is the tool's state after the command):

    SENT,TOOL_01,TARGET,100.0000          queued for the radio
    ACK,TOOL_01,TARGET,MONITORING,100.0000 tool accepted it
    NACK,TOOL_01,TARGET,IDLE,100.0000     tool refused it (TARGET before TARE)
    ACK,TOOL_01,TARE,IDLE,0.0000          tare started
    DONE,TOOL_01,TARE,READY,0.0000        tare finished (or TARE_FAILED)
    ERROR,NO_RESPONSE,TOOL_01,STOP        no ACK/NACK within 1 s

This deliberately simple protocol makes it straightforward for a future GUI
to consume.

The laptop sends:

    TARGET,TOOL_01,100
    STOP,TOOL_01
    TARE,TOOL_01
    RESET,TOOL_01
    LOOSEN,TOOL_01
    TARGET_ALL,100
    STOP_ALL
    TARE_ALL
    RESET_ALL
    LOOSEN_ALL
    LIST

## Dashboard (tablet-style UI)

`laptop_controller/dashboard.py` serves a browser UI styled after the rugged
tablet mock-ups: flange selection, live bolt diagram and per-bolt bars during
tightening, and a completed screen with a saved report.

    pip install -r requirements.txt
    python dashboard.py --port COM5      # real controller
    python dashboard.py --sim 12         # no hardware, 12 simulated tools

**Click-to-run scripts.** `laptop_controller/run_scenarios/` has a Windows
`.bat` file per scenario (first-time setup, three simulations, real tools at
bench scale or on the bench rig flange, and the text controller). Double-click
one to run it. The COM port and bench target are set once in `settings.bat`;
the folder's `README.txt` lists what each script does.

It opens http://localhost:8000. A run:

1. Uses only bolts whose tool is online (bolt 1 = TOOL_01, and so on, unless
   the profile lists `tools`).
2. Tares every tool, so start with the tools fitted but the bolts unloaded.
3. Sends the full target to every tool at once, in a single run with no
   passes, and waits for TARGET_REACHED. A bolt that misses the timeout (120 s
   by default, `--timeout`) is stopped and flagged as insufficient preload.
4. Keeps watching bolts that are at target, because tightening a bolt can
   loosen its neighbours. A bolt that reads below target (by any amount) for
   three status lines in a row loses its tick and is sent its target again.
   After 5 re-tightens it is flagged as insufficient ("kept loosening"). The
   run finishes only once every bolt is at target together for 3 s. This needs
   the current tool firmware, which keeps measuring after TARGET_REACHED.
5. Stops only that tool and flags its bolt as over-tightened if it reads above
   the profile's `max_n` (the maximum preload). The other bolts carry on.
   Every STOP the dashboard sends is repeated (up to 5 times, 1 s apart) until
   the tool reports IDLE; if it never does, the log shows
   `ERROR,STOP_NOT_CONFIRMED,<tool>`.
6. Saves an Excel report (`.xlsx`) in `laptop_controller/reports/`: the run
   summary with each bolt's final load and re-tighten count at the top, then
   the load log below it (every status line from every tool during the run,
   about 4 per second per tool: time, bolt, state, stop value N (what the
   tool compares with the target), raw N, relay), and a chart of the stop
   value against time with one line per bolt
   plus the target (minimum) and maximum. Written by `xlsx_report.py` with
   the Python standard library only, so nothing extra to install.

The gear icon opens a tool panel with live readings, Tare / Stop / Loosen
all, and a Loosen button per tool (see Loosening below). Loosen is refused
while a tightening run is in progress.

**Tools per bolt.** A flange's `bolt_count` is the most tools it can take.
Before starting, tap any bolt to attach a tool, move one, or leave the bolt
empty (Auto fills free bolts with online tools in order). Tools can be
staggered: after a run, "Next Tool Group" keeps the finished bolts and
auto-attaches the tools to the remaining ones. The report lists the group
that did each bolt.

**Flanges and bolts.** `flange_profiles.json` holds every API 6A flange
from Tables D.1-D.7 / E.1-E.7 (6B and 6BX, including large bore) plus two
bench rigs: pressure class and socket size in metric and imperial, bolt
count, and the stud size (`bolt`, e.g. `1-1/8-8`). It holds no targets.
`bolt_profiles.json` holds Tables H.1 (SI) and H.2 (USC) per stud size. After
picking a flange the operator picks the allowable bolt stress (275 / 360 /
327.5 MPa, or 40 / 52.5 / 47.5 ksi, as the table has them for that stud) and
the friction factor f (0.07 or 0.13). The dashboard reads the target tension
and make-up torque from the table as the minimum, and sends the tension to the
tools in N. The maximum preload and torque are `max_over_target_pct` above the
minimum (25 %, so max = min + 0.25 × min); the setup screen shows all four.
`flange_overrides` caps the stress at 275 MPa / 40 ksi for the H.3.3 flanges.
The lookup is in `bolt_calc.py`, where equation-based targets can be added.
A bench rig uses a bolt with a fixed `custom` target (50 N) for the
proof-of-concept load cell (62.5 N max). The table values were typed in from the API 6A
pages; check any value against the standard before real use.

**Bench scale (real tool, small load).** `python dashboard.py --port COM5
--bench-target 30` runs any flange setup on the proof-of-concept load cell:
the tools are sent 30 N (any value; 10 to 50 N suits the current cell) instead
of the full table tension, and the over-limit stop scales the same way. The
UI and reports scale each reading back up, so pressing the cell to 15 N shows
half the flange's target tension and the torque that implies. A yellow note
on the setup screen shows the load-cell target; the tool panel and the
report's `load_cell_N` column keep the raw load-cell values.

**Units.** The Metric / Imperial button in the header switches every force
(N or kN / lbf), torque (N·m / lbf·ft), pressure class, socket and stud size,
and which stress table is offered (H.1 or H.2). The live torque during a run
scales the table torque with the measured tension (T ∝ F) and is marked ≈
except at the exact table value. Reports list targets and results in both kN
and lbf. The text CLI takes `--units metric|imperial` (targets typed in N or
lbf) and `--n-per-count` to show measured forces instead of raw counts.

**Flange icons.** Each flange's diagram comes from its `icon` (`6BX`, `6B`,
`bench` or `generic`; default follows `type`). Bolt positions are drawn from
`bolt_count`. New icon styles go in `FLANGE_ICONS` in `static/index.html`.

**Ideal simulation.** `python simulate_ideal.py` starts the dashboard with 32
simulated tools (`--tools N` to change) that all stream live readings and
always reach target, each at its own pace.

**Other simulations.** `--sim N` takes `--sim-stall P` (chance a tool stalls
below target, default 0.03), `--sim-overshoot P` (chance it overshoots the
max when it reaches target) and `--sim-loosen F` (how much tightening one bolt
loosens those already at target; 0.3 makes re-tightening easy to see).

`--n-per-count` must match `N_PER_COUNT` in the tool firmware (default 0.205).
The controller firmware tracks at most 20 tools (the ESP-NOW peer limit), so
flanges with more than 20 bolts need staggered tool groups on hardware.

## Future UI integration

Import instead of replacing the communications layer:

    from laptop_controller import ToolNetworkController

Then create a controller and attach callbacks:

    controller = ToolNetworkController("COM5")
    controller.on_status = my_status_handler
    controller.connect()
    controller.set_target("TOOL_01", 100.0)

A GUI should use the callback/status model rather than directly parsing serial
data throughout its widgets.

## On-tool safety

The tool firmware does not rely on the laptop to stop:

- Relays are forced off (pins latched HIGH before becoming outputs) as the
  first thing at boot. The pins float for the moment between power-on and
  that line; a pull-up resistor on each relay input covers that gap in hardware.
- The tool stops itself (relays off, TARGET_REACHED) as soon as the filtered
  value reaches the target, which is the minimum preload. Checking starts
  after 3 readings (`MIN_DETECT_READINGS`) rather than a full 10-reading
  window, so it no longer runs blind for ~1 s at the start of each run or
  re-tighten. A TARGET for a bolt already at or above it never switches the
  relays on.
- During a run, three MAD-filtered values in a row above 1.25 × target
  (`MAX_OVER_TARGET`, which must match `max_over_target_pct`) switch the
  relays off and put the tool in STOPPED, reporting the over-force reading.
- A run that lasts longer than `MAX_RUN_MS` (180 s) stops itself.
- A LOOSEN run stops itself after 15 s.

## Loosening

`LOOSEN,<tool>` (or `LOOSEN_ALL`) runs a tool in reverse. The tool stops by
itself after 15 s (`LOOSEN_TIMEOUT_MS`) and replies `DONE,<tool>,LOOSEN,...`;
STOP, RESET or TARE end it earlier. It works in any state, tared or not: a
tared tool goes back to READY afterwards, an untared one back to IDLE. While
loosening, STATUS shows the state `LOOSENING`.

Wiring: there is no direction relay yet (`REVERSE_RELAY_PIN = -1`), so set
the wrench to reverse BY HAND before pressing Loosen; LOOSEN only switches the
two drive relays. There is no check that the force is falling, so a wrench
left in forward will tighten until the 15 s timeout or Stop. If a direction
relay is fitted later, set `REVERSE_RELAY_PIN` to its GPIO: it is then
switched (active-LOW, ON = reverse) before the drive relays, only with the
drive relays off (500 ms spin-down).

## Important current limitations

- ESP-NOW packets are currently unencrypted.
- This is an early protocol and does not yet include command sequence numbers,
  so two quick commands of the same kind to one tool can't be told apart.
- `SENT` only means the ESP-NOW send was queued; the tool's own `ACK` / `NACK`
  line is the confirmation. A lost reply shows as `ERROR,NO_RESPONSE`, even if
  the tool did act on the command.
- Tool firmware should be bench-tested with relays/load hardware in a safe
  setup before controlling the real mechanism.
