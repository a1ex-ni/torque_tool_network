# Torque Tool Network

Proof-of-concept system that tightens every bolt on a flange at once. Each
tool unit reads a load cell under its bolt and switches its wrench off at the
target preload; a laptop dashboard runs the sequence and saves a report.

    Browser (static/index.html)
          |  WebSocket / REST
    dashboard.py  (laptop)
          |  USB Serial, 115200 baud
    tool_network_controller.ino  (ESP32-C3 on the laptop)
          |  ESP-NOW broadcast
    target_monitor_wifi.ino  (one ESP32-C3 per tool: TOOL_01, TOOL_02, ...)

Load-cell sampling, target cutoff and the safety stops all run on the tool, so
a dropped radio link cannot leave a wrench running.

## Folder layout

| Path | What it is |
|------|------------|
| `target_monitor_wifi/target_monitor_wifi.ino` | Tool firmware (HX711 + 2 drive relays) |
| `tool_network_controller/tool_network_controller.ino` | Gateway firmware (ESP-NOW to USB Serial) |
| `laptop_controller/dashboard.py` | Web dashboard and tightening sequence |
| `laptop_controller/laptop_controller.py` | Serial API (`ToolNetworkController`) and text CLI |
| `laptop_controller/simulated_controller.py` | Fake controller and tools for running without hardware |
| `laptop_controller/simulate_ideal.py` | Simulation where every bolt reaches target |
| `laptop_controller/bolt_calc.py` | Target tension and torque lookup (API 6A Tables H.1 / H.2) |
| `laptop_controller/xlsx_report.py` | Excel report writer (standard library only) |
| `laptop_controller/flange_profiles.json` | 71 API 6A flanges (6B and 6BX) plus 2 bench rigs |
| `laptop_controller/bolt_profiles.json` | Stud ratings, stress options, friction factors |
| `laptop_controller/static/index.html` | Tablet-style UI |
| `laptop_controller/run_scenarios/` | Double-click `.bat` launchers for Windows |
| `laptop_controller/reports/` | Saved run reports |

## Setup

**Arduino:** ESP32 Arduino core (WiFi and ESP-NOW come with it) on both
boards, plus the HX711 library for the tool. Use the same core version on both.

**Each tool:** set a unique `TOOL_ID` (`"TOOL_01"`, `"TOOL_02"`, ...) before
flashing. Pins: HX711 DOUT 2 / SCK 3, relays on GPIO 1 and 0 (active-LOW).

**Calibration:** `N_PER_COUNT = 0.19975` N per count, worked out from the
datasheets (CF-816 load cell 0.69936 mV/V at 300 kN, HX711 gain 128). It must
match `DEFAULT_N_PER_COUNT` in `dashboard.py` and the `--n-per-count` default.

**Laptop:** Python 3.10+, then in `laptop_controller/`:

    pip install -r requirements.txt    # pyserial, fastapi, uvicorn

Only one program can hold the COM port: close the Arduino Serial Monitor first.

## Running

    python dashboard.py --port COM5               # real tools
    python dashboard.py --port COM5 --bench-target 1500
    python dashboard.py --sim 12                  # 12 simulated tools
    python simulate_ideal.py                      # 32 simulated tools, all reach target
    python laptop_controller.py --port COM5       # text CLI, no dashboard

The dashboard opens http://localhost:8000. If `--port` is left out, the
program picks the only ESP32/USB-looking port or asks.

Dashboard options: `--timeout S` (time a bolt may take, default 120 s; capped
at 30 s in simulation), `--bench-target N`, `--n-per-count`, `--baud`,
`--http-port`, `--no-browser`. Simulation options (with `--sim N`):
`--sim-stall P` (chance a bolt stalls below target, default 0.03),
`--sim-overshoot P` (chance it overshoots the max), `--sim-loosen F` (how much
tightening one bolt loosens the others, e.g. 0.3).

`run_scenarios/` wraps these as `.bat` files (setup, three simulations, bench
scale, bench rig, text CLI metric and imperial). Set the COM port and bench
target once in `settings.bat`.

## Using the dashboard

1. **Select a flange.** Filter by type and pressure class, or search.
2. **Set up.** Pick the allowable bolt stress and friction factor. The screen
   shows the stud, minimum preload (target) and torque, and maximum preload
   and torque. Tap a bolt to attach, move or clear a tool (Auto fills free
   bolts with online tools in order). A flange's `bolt_count` is the most tools
   it can take.
3. **Start Tightening.** The run then:
   - uses only bolts with an online tool;
   - tares every tool (fit the tools with the bolts unloaded);
   - sends the full target to every tool at once, with no passes;
   - marks a bolt reached after 2 status lines in a row at TARGET_REACHED and
     at or above target;
   - re-sends the target to a reached bolt that reads below target for 3
     status lines in a row (tightening one bolt loosens its neighbours), up to
     5 times before flagging it "insufficient";
   - stops that tool and flags its bolt "over-tightened" after 2 status lines
     above the maximum;
   - flags a bolt "insufficient" if it misses the timeout;
   - finishes once every bolt is at target together for 3 s.
4. **Completed.** Shows each bolt's result. Save Report downloads the Excel
   file. Next Tool Group keeps the finished bolts and attaches the tools to
   the rest, so a flange with more bolts than tools is done in groups.

Every STOP the dashboard sends is repeated (up to 5 times, 1 s apart) until
the tool reports IDLE or STOPPED, otherwise the log shows
`ERROR,STOP_NOT_CONFIRMED,<tool>`. Cancel sends STOP_ALL, then a confirmed
STOP to each bolt still tightening.

The gear icon opens the tool panel: live readings per tool, Tare All, Stop
All, Loosen All and a Loosen button per tool. Loosening is refused while a
run is in progress.

**Units.** The Metric / Imperial button switches forces (N, kN / lbf),
torques (N·m / lbf·ft), pressure class, socket and stud size, and which stress
table is offered (H.1 or H.2). Live torque scales the table torque with the
measured tension (T ∝ F) and is marked ≈ except at the table value.

**Bench scale.** `--bench-target N` sends every API 6A table target to the tools
scaled down to N newtons, with the maximum scaled the same way. The UI and
report scale readings back up to full size; a yellow note shows the
load-cell target, and the tool panel and report keep the load-cell values.

## Targets

`flange_profiles.json` holds geometry only: `id`, `display_name`,
`bolt_count` and `bolt` (stud size, e.g. `1-1/8-8`) are required; `type`,
`icon`, pressure classes, socket sizes and `tools` (tool per bolt) are
optional. `bolt_profiles.json` holds API 6A Tables H.1 (275 / 360 / 327.5 MPa)
and H.2 (40 / 52.5 / 47.5 ksi) per stud, with friction factors 0.07 and 0.13.

`bolt_calc.py` reads the target tension and make-up torque from the table for
the chosen stress and friction factor. The maximum is `max_over_target_pct`
(25 %) above the target. `flange_overrides` caps four H.3.3 flanges at
275 MPa / 40 ksi. The bench rigs use the `BENCH` stud with a fixed 50 N
target and 62.5 N maximum. Table values were typed in by hand: check them
against the standard before real use.

## Tool firmware behaviour

States: `IDLE`, `TARE_FAILED`, `READY`, `MONITORING`, `TARGET_REACHED`,
`STOPPED`, `LOOSENING`.

- **Boot:** relays forced off first, then `IDLE`. No auto-tare.
- **TARE:** tares until the filtered mean is within 1500 counts of zero and
  the scaled MAD is under 500 counts, over a 10-reading window, or gives up
  after 20 s (`READY` or `TARE_FAILED`). Discovery pings continue so the tool
  stays online.
- **TARGET:** refused (NACK) unless the tool holds a good tare. Relays switch
  on (continuously; pulsing is coded but disabled) and the tool goes to
  `MONITORING`. A bolt already at target stays off and goes straight to
  `TARGET_REACHED`.
- **Target detection:** each reading goes into a 10-reading window; the MAD
  filter (modified z ≤ 3.5, MAD floor 2 counts) removes spikes and the
  maximum of the remaining readings is compared with the target
  (`TARGET_DETECTION = DETECT_MAXIMUM`, the least lag). Checking starts after
  3 readings. The reading used is the absolute value, since compression can
  read negative.
- **After TARGET_REACHED** the tool keeps measuring, so the dashboard can see
  a bolt relax.
- **STOP:** relays off, target cleared, `IDLE`, tare kept (a TARGET works
  straight away). **RESET:** the same but the tare is cleared.
- **LOOSEN:** runs the drive relays for 5 s (`LOOSEN_TIMEOUT_MS`), then
  replies DONE and returns to `READY` (or its previous state if untared).
  STOP, RESET or TARE end it early. There is no direction relay yet
  (`REVERSE_RELAY_PIN = -1`), so set the wrench to reverse by hand, and
  nothing checks that the force is falling.
- **Safety stops (`STOPPED`):** 3 filtered readings in a row above
  1.25 × target (`MAX_OVER_TARGET`), or a run longer than 180 s
  (`MAX_RUN_MS`).
- **Status** is broadcast every 250 ms, discovery every 1 s.

**Serial debug.** With the tool on USB, the Serial Monitor accepts `TARE`,
`STOP`, `RESET`, `TARGET,<N>`, `LOOSEN`, `STATUS`, `RAW` (5 s of raw HX711
readings and the sample rate) and `HELP`. Commands go through the same code as
radio commands.

## Controller firmware behaviour

Tracks up to 20 tools, found from their discovery broadcasts. A tool is
reported offline after 5 s without a packet, plus 60 s while it is taring. A
command with no reply within 3 s gives `ERROR,NO_RESPONSE`.

## Serial protocol

Laptop to controller (any case):

    LIST
    TARGET,<tool>,<N>      TARGET_ALL,<N>
    TARE,<tool>            TARE_ALL
    STOP,<tool>            STOP_ALL
    RESET,<tool>           RESET_ALL
    LOOSEN,<tool>          LOOSEN_ALL

Controller to laptop:

    STATUS,<tool>,<state>,<target_N>,<raw_counts>,<stop_value_counts>,<relay_on>
    SENT,<tool>,<cmd>[,<N>]                       command handed to the radio
    COMMAND ACCEPTED,<tool>,<cmd>,<state>,<N>     tool accepted it
    COMMAND REJECTED,<tool>,<cmd>,<state>,<N>     tool refused it (e.g. TARGET before TARE)
    DONE,<tool>,<cmd>,<state>,<N>                 TARE or LOOSEN finished
    TOOL,<tool>,ONLINE|OFFLINE,<mac>              reply to LIST
    EVENT,DISCOVERED|ONLINE|OFFLINE,<tool>[,<mac>]
    ERROR,NO_RESPONSE,<tool>,<cmd>

`stop_value` is the filtered value the tool compares with the target. The
target field lets the dashboard ignore status from an older TARGET. Other
errors: `UNKNOWN_TOOL`, `SEND_FAILED`, `NO_TOOLS`, `BAD_TARGET_VALUE`,
`BAD_COMMAND`, `BAD_TARGET_COMMAND`, `UNKNOWN_COMMAND`, `TOOL_TABLE_FULL`,
`RX_QUEUE_FULL`.

The `WirelessPacket` struct must stay identical in both `.ino` files.

## Text CLI

`laptop_controller.py` takes `list`, `target <tool> <force>`, `stop`, `tare`,
`reset`, `loosen <tool>`, the `-all` versions, `status`, `help` and `quit`.
`--units imperial` shows and takes forces in lbf.

To reuse the serial layer elsewhere:

    from laptop_controller import ToolNetworkController
    c = ToolNetworkController("COM5")
    c.on_status = handler      # called with a ToolStatus per STATUS line
    c.connect()
    c.set_target("TOOL_01", 100.0)

## Reports

Each run (completed or cancelled) saves
`reports/<YYYYMMDD_HHMMSS>_<flange id>.xlsx`:

- **Run** sheet: summary (flange, result, stud, setting, target and maximum
  in kN and lbf, table tension and torque), one row per bolt (group, status,
  final load in kN, lbf and load-cell N, re-tightens, note), then every
  status line logged during the run, grouped by bolt.
- **Chart 1:** stop value (load-cell N) against time, one line per bolt, with
  the target and maximum.
- **Chart 2 / Scaled load** sheet: full-size kN resampled to a constant step
  (0.1 s, widened for long runs), each bolt holding its last reported value.

## Limitations

- ESP-NOW packets are unencrypted and have no sequence numbers.
- `SENT` only means the send was queued; a lost reply shows as
  `ERROR,NO_RESPONSE` even if the tool acted on the command.
- The load cell's accuracy has not been tested against a reference load.
- Bench-test the tool firmware with the relays and load hardware in a safe
  setup before using it on a real flange.
