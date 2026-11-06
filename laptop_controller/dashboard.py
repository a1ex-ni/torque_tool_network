"""
dashboard.py: web dashboard and tightening sequence for the torque tool network.

    Browser <-WebSocket/REST-> this server <-USB serial-> controller ESP32 <-ESP-NOW-> tools

Run:
    python dashboard.py --port COM5          # real controller
    python dashboard.py --sim 12             # no hardware, 12 simulated tools

Then open http://localhost:8000 (opened automatically unless --no-browser).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
import webbrowser
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from bolt_calc import N_PER_LBF, bolt_setup, load_bolt_profiles, resolve
from xlsx_report import write_report
from laptop_controller import ToolNetworkController, ToolStatus, choose_port, open_port

HERE = Path(__file__).resolve().parent
PROFILES_PATH = HERE / "flange_profiles.json"
REPORTS_DIR = HERE / "reports"

DEFAULT_N_PER_COUNT = 0.19975   # must match N_PER_COUNT in target_monitor_wifi.ino
TOOL_OFFLINE_S = 3.0
TARE_TIMEOUT_S = 75.0
DEFAULT_TIMEOUT_S = 120.0

# Counts are consecutive status lines (one every 250 ms)
LOOSE_READINGS = 3           # below target -> re-tighten
MAX_RETIGHTENS = 5           # then insufficient
HOLD_IN_SPEC_S = 3.0         # all bolts at target this long to finish
REACHED_READINGS = 2         # TARGET_REACHED -> reached
OVER_READINGS = 2            # over max -> stop
STOP_ATTEMPTS = 5
STOP_RETRY_S = 1.0


# ---------------------------------------------------------------- state ----

@dataclass
class ToolView:
    tool_id: str
    state: str = "UNKNOWN"
    target_n: float = 0.0
    raw_n: float = 0.0
    min_n: float = 0.0
    avg_n: float = 0.0
    relay_on: bool = False
    mac: str = ""
    last_seen: float = 0.0
    updated: float = 0.0     # last STATUS time
    taring: bool = False
    radio_offline: bool = False  # from EVENT,OFFLINE

    def online(self) -> bool:
        # Silent while taring: trust the controller's EVENT,OFFLINE
        if self.taring:
            return not self.radio_offline
        return time.time() - self.last_seen <= TOOL_OFFLINE_S

    def to_json(self) -> dict:
        return {
            "tool_id": self.tool_id,
            "state": self.state,
            "target_n": round(self.target_n, 2),
            "raw_n": round(self.raw_n, 2),
            "min_n": round(self.min_n, 2),
            "avg_n": round(self.avg_n, 2),
            "relay_on": self.relay_on,
            "online": self.online(),
            "mac": self.mac,
        }


@dataclass
class BoltRun:
    bolt: int
    tool_id: str
    status: str = "pending"   # pending | no_tool | tare_failed | tightening | reached | insufficient | over_max | stopped
    final_n: Optional[float] = None
    note: str = ""
    group: int = 1
    carried: bool = False    # result from an earlier group
    retightens: int = 0
    sent: float = 0.0        # last TARGET time
    seen: float = 0.0        # last status counted
    low: int = 0             # consecutive readings below target
    high: int = 0            # ... over max
    hits: int = 0            # ... reached

    def to_json(self) -> dict:
        return {
            "bolt": self.bolt,
            "group": self.group,
            "carried": self.carried,
            "tool_id": self.tool_id,
            "status": self.status,
            "final_n": None if self.final_n is None else round(self.final_n, 1),
            "retightens": self.retightens,
            "note": self.note,
        }


@dataclass
class SequenceRun:
    profile: dict
    bolts: List[BoltRun]
    phase: str = "starting"   # starting | taring | tightening | complete | cancelled | aborted
    message: str = ""
    started_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))
    report_file: str = ""
    group: int = 1
    tare_failed: int = 0
    t0: float = field(default_factory=time.time)
    # Report log: (t, bolt, tool_id, state, filtered N, avg N, raw N, relay on)
    log: List[tuple] = field(default_factory=list)

    def to_json(self) -> dict:
        return {
            "profile_id": self.profile["id"],
            "group": self.group,
            "phase": self.phase,
            "target_n": self.profile["target_n"],
            "setup": self.profile.get("setup"),
            "message": self.message,
            "bolts": [b.to_json() for b in self.bolts],
            "report_file": self.report_file,
        }


class Dashboard:
    def __init__(self, controller: ToolNetworkController, n_per_count: float, timeout_s: float,
                 bench_target_n: Optional[float] = None):
        self.controller = controller
        self.n_per_count = n_per_count
        self.timeout_s = timeout_s
        # Bench scale: table targets scaled down to this many N
        self.bench_target_n = bench_target_n
        self.tools: Dict[str, ToolView] = {}
        self.sockets: List[WebSocket] = []
        self.run: Optional[SequenceRun] = None
        self.task: Optional[asyncio.Task] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.log: List[str] = []

    # ---- controller callbacks (serial thread) ----

    def attach(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.controller.on_status = lambda t: loop.call_soon_threadsafe(self._on_status, _copy(t))
        self.controller.on_event = lambda line: loop.call_soon_threadsafe(self._on_event, line)

    def _on_status(self, t: ToolStatus) -> None:
        view = self.tools.get(t.tool_id) or ToolView(t.tool_id)
        view.mac = t.mac or view.mac
        view.last_seen = t.last_seen
        view.radio_offline = False
        view.taring = False
        if t.last_status > view.updated:
            view.state = t.state
            view.target_n = t.target_n
            view.raw_n = t.raw_counts * self.n_per_count
            view.min_n = t.filtered_counts * self.n_per_count
            view.avg_n = t.filtered_avg_counts * self.n_per_count
            view.relay_on = t.relay_on
            view.updated = t.last_status
            self._record(view)
        self.tools[t.tool_id] = view
        self._check_limit(view)
        self._broadcast({"type": "tool", "data": view.to_json()})

    def _on_event(self, line: str) -> None:
        self._track_tare(line)
        if line.startswith(("STATUS,", "SENT,", "ACK,")):
            return
        self.log = (self.log + [line])[-50:]
        self._broadcast({"type": "log", "data": line})

    def _track_tare(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) < 3:
            return
        if parts[0] == "EVENT" and parts[1] in ("ONLINE", "OFFLINE"):
            view = self.tools.get(parts[2])
            if view:
                view.radio_offline = parts[1] == "OFFLINE"
                self._broadcast({"type": "tool", "data": view.to_json()})
            return
        if parts[2] != "TARE" or parts[1] not in self.tools:
            return
        view = self.tools[parts[1]]
        if parts[0] in ("ACK", "COMMAND ACCEPTED"):
            view.taring = True
        elif parts[0] == "DONE":
            view.taring = False
            view.last_seen = time.time()

    # ---- report log ----

    def _record(self, view: ToolView) -> None:
        run = self.run
        if not run or run.phase not in ("taring", "tightening"):
            return
        b = next((b for b in run.bolts if b.tool_id == view.tool_id and not b.carried), None)
        if not b or b.status == "no_tool":
            return
        run.log.append((view.updated - run.t0, b.bolt, view.tool_id, view.state,
                        view.min_n, view.avg_n, view.raw_n, view.relay_on))

    # ---- over-tightening ----

    def _check_limit(self, view: ToolView) -> None:
        run = self.run
        if not run or run.phase not in ("taring", "tightening"):
            return
        b = next((b for b in run.bolts if b.tool_id == view.tool_id and not b.carried), None)
        if not b or b.status not in ("tightening", "reached"):
            return
        live = max(view.avg_n, view.min_n)
        b.high = b.high + 1 if live > run.profile["max_n"] else 0
        if b.high >= OVER_READINGS:
            self.stop_tool(view.tool_id)
            b.status = "over_max"
            b.final_n = live
            b.note = f"Over-tightened: {live:.1f} N, max {run.profile['max_n']:.1f} N"
            self.push_run()

    # ---- STOP, retried until confirmed ----

    def stop_tool(self, tool_id: str) -> None:
        asyncio.ensure_future(self._stop_until_confirmed(tool_id))

    async def _stop_until_confirmed(self, tool_id: str) -> None:
        # Give up if a new run has started
        run = self.run
        for _ in range(STOP_ATTEMPTS):
            if self.run is not run:
                return
            sent = time.time()
            self.controller.stop(tool_id)
            while time.time() - sent < STOP_RETRY_S:
                await asyncio.sleep(0.1)
                tool = self.tools.get(tool_id)
                if tool and tool.updated > sent and tool.state in ("IDLE", "STOPPED"):
                    return
        line = f"ERROR,STOP_NOT_CONFIRMED,{tool_id}"
        self.log = (self.log + [line])[-50:]
        self._broadcast({"type": "log", "data": line})

    # ---- broadcasting ----

    def _broadcast(self, msg: dict) -> None:
        for ws in list(self.sockets):
            asyncio.ensure_future(self._send(ws, msg))

    async def _send(self, ws: WebSocket, msg: dict) -> None:
        try:
            await ws.send_json(msg)
        except Exception:
            if ws in self.sockets:
                self.sockets.remove(ws)

    def push_run(self) -> None:
        if self.run:
            self._broadcast({"type": "run", "data": self.run.to_json()})

    # ---- sequence ----

    def start(self, profile: dict, assignment: List[Optional[str]], continue_job: bool = False) -> SequenceRun:
        # assignment[i]: tool on bolt i+1 or None. continue_job keeps earlier groups' results.
        prev = self.run if continue_job and self.run and self.run.profile["id"] == profile["id"] else None
        group = prev.group + 1 if prev else 1
        bolts = []
        for i, tid in enumerate(assignment):
            old = prev.bolts[i] if prev else None
            if not tid and old and old.status not in ("no_tool", "pending"):
                bolts.append(BoltRun(old.bolt, old.tool_id, old.status, old.final_n, old.note, old.group, True,
                                     old.retightens))
            else:
                bolts.append(BoltRun(bolt=i + 1, tool_id=tid or "", group=group))
        self.run = SequenceRun(profile=profile, bolts=bolts, group=group, phase="taring")
        self.task = asyncio.ensure_future(self._sequence(self.run))
        return self.run

    def cancel(self) -> None:
        self.controller.stop_all()
        if self.task and not self.task.done():
            self.task.cancel()
        if self.run and self.run.phase not in ("complete", "aborted", "cancelled"):
            self.run.phase = "cancelled"
            self.run.message = "Cancelled. All tools stopped and back to idle."
            for b in self.run.bolts:
                if b.status in ("pending", "tightening"):
                    b.status = "stopped"
                    self.stop_tool(b.tool_id)
            self._finish(self.run)

    async def _sequence(self, run: SequenceRun) -> None:
        profile = run.profile
        try:
            # 1. Skip bolts without an online tool
            for b in run.bolts:
                tool = self.tools.get(b.tool_id)
                if b.carried:
                    continue
                if not b.tool_id:
                    b.status = "no_tool"
                    b.note = "No tool attached"
                elif not tool or not tool.online():
                    b.status = "no_tool"
                    b.note = "Tool offline"
            active = [b for b in run.bolts if b.status == "pending"]
            if not active:
                run.phase = "aborted"
                run.message = "No bolts have an online tool attached."
                self.push_run()
                return

            # 2. Tare every tool (bolts unloaded)
            run.phase = "taring"
            run.message = self._tare_message(active, set())
            self.push_run()
            for b in active:
                self.controller.tare(b.tool_id)
            tared: set = set()
            await asyncio.gather(*(self._wait_tared(run, b, active, tared) for b in active))
            failed = [b for b in active if b.status == "tare_failed"]
            active = [b for b in active if b.status == "pending"]
            if not active:
                run.phase = "aborted"
                run.message = "Tare failed on every tool. Check the load cells and that the bolts are unloaded."
                self.push_run()
                return
            run.tare_failed = len(failed)

            # 3. Full target on every bolt at once
            run.phase = "tightening"
            target_n = profile["target_n"]
            for b in active:
                b.status = "tightening"
            self.push_run()
            sent = time.time()
            for b in active:
                b.sent = sent
                self.controller.set_target(b.tool_id, target_n)
            await self._monitor(run, active, target_n)

            run.phase = "complete"
            done = sum(b.status == "reached" for b in run.bolts)
            untouched = sum(b.status == "no_tool" for b in run.bolts)
            over = sum(b.status == "over_max" for b in run.bolts)
            tare_bad = sum(b.status == "tare_failed" for b in run.bolts)
            problems = len(run.bolts) - done - untouched - over - tare_bad
            if done == len(run.bolts):
                run.message = "All bolts reached target."
            else:
                parts = [f"{done} of {len(run.bolts)} bolts at target"]
                if over:
                    parts.append(f"{over} over-tightened")
                if tare_bad:
                    parts.append(f"{tare_bad} tare failed")
                if problems:
                    parts.append(f"{problems} need attention")
                if untouched:
                    parts.append(f"{untouched} still need a tool")
                run.message = ", ".join(parts) + "."
            self._finish(run)
        except asyncio.CancelledError:
            pass

    @staticmethod
    def _tare_message(active: List[BoltRun], tared: set) -> str:
        failed = [b for b in active if b.status == "tare_failed"]
        msg = f"{len(tared)} of {len(active)} tared"
        if failed:
            msg += " · tare failed on " + ", ".join(f"bolt {b.bolt} ({b.tool_id})" for b in failed)
        return msg

    async def _wait_tared(self, run: SequenceRun, b: BoltRun, active: List[BoltRun], tared: set) -> None:
        start = time.time()
        # Skip status already in flight
        await asyncio.sleep(0.5)
        fresh_after = time.time()
        timed_out = True
        while time.time() - start < TARE_TIMEOUT_S:
            tool = self.tools.get(b.tool_id)
            if tool and tool.updated > fresh_after:
                if tool.state == "READY":
                    tared.add(b.bolt)
                    run.message = self._tare_message(active, tared)
                    self.push_run()
                    return
                if tool.state == "TARE_FAILED":
                    timed_out = False
                    break
            await asyncio.sleep(0.2)
        b.status = "tare_failed"
        b.note = ("Tare failed: no reply from the tool" if timed_out
                  else "Tare failed: reading not stable or not near zero (is the bolt unloaded?)")
        line = f"ERROR,TARE_FAILED,{b.tool_id}"
        self.log = (self.log + [line])[-50:]
        self._broadcast({"type": "log", "data": line})
        run.message = self._tare_message(active, tared)
        self.push_run()

    async def _monitor(self, run: SequenceRun, active: List[BoltRun], target_n: float) -> None:
        """Watch every bolt until all are at target together, re-tightening any that loosen."""
        tol = max(0.05, 1e-5 * target_n)   # float32 echo
        in_spec_since: Optional[float] = None
        last_push = 0.0
        while True:
            now = time.time()
            changed = False
            for b in active:
                if b.status not in ("tightening", "reached"):
                    continue
                tool = self.tools.get(b.tool_id)
                # Only trust status for this bolt's latest TARGET
                fresh = bool(tool) and tool.updated > b.sent and abs(tool.target_n - target_n) < tol
                new = fresh and tool.updated > b.seen
                if new:
                    b.seen = tool.updated
                # STOP clears the target, so check IDLE separately
                stopped_idle = bool(tool) and tool.updated > b.sent and tool.state == "IDLE"
                if stopped_idle or (fresh and tool.state == "STOPPED"):
                    b.status = "stopped"
                    b.final_n = max(tool.avg_n, tool.min_n)
                    b.note = "Stopped"
                    changed = True
                else:
                    at_target = new and tool.state == "TARGET_REACHED" and tool.min_n >= target_n
                    below = new and tool.state == "TARGET_REACHED" and tool.min_n < target_n
                    if new and tool.state == "TARGET_REACHED":
                        b.final_n = tool.min_n
                    # Stopped but below target: re-tighten
                    if new:
                        b.low = b.low + 1 if below else 0
                    if b.low >= LOOSE_READINGS:
                        b.low = 0
                        b.hits = 0
                        if b.retightens >= MAX_RETIGHTENS:
                            self.stop_tool(b.tool_id)
                            b.status = "insufficient"
                            b.note = f"Kept loosening ({b.final_n:.0f} N after {b.retightens} re-tightens)"
                        else:
                            b.retightens += 1
                            b.status = "tightening"
                            b.sent = time.time()
                            self.controller.set_target(b.tool_id, target_n)
                        changed = True
                    elif b.status == "tightening":
                        if new:
                            b.hits = b.hits + 1 if at_target else 0
                        if b.hits >= REACHED_READINGS:
                            b.hits = 0
                            b.status = "reached"
                            changed = True
                        elif now - b.sent > self.timeout_s:
                            self.stop_tool(b.tool_id)
                            b.status = "insufficient"
                            b.final_n = max(tool.avg_n, tool.min_n) if tool else 0.0
                            b.note = f"Timed out at {b.final_n:.0f} N of {target_n:.0f} N"
                            changed = True

            watched = [b for b in active if b.status in ("tightening", "reached")]
            if watched and all(b.status == "reached" for b in watched):
                in_spec_since = in_spec_since or now
            else:
                in_spec_since = None
            at = sum(b.status == "reached" for b in active)
            redo = sum(b.status == "tightening" and b.retightens > 0 for b in active)
            msg = f"{at} of {len(active)} at target"
            if run.tare_failed:
                msg += f" · {run.tare_failed} skipped (tare failed)"
            if redo:
                msg += f" · re-tightening {redo} that loosened"
            if in_spec_since:
                msg += " · checking they hold"
            if msg != run.message:
                run.message = msg
                changed = True
            if not watched or (in_spec_since and now - in_spec_since >= HOLD_IN_SPEC_S):
                return
            if changed or now - last_push > 0.5:
                self.push_run()
                last_push = now
            await asyncio.sleep(0.1)

    def _finish(self, run: SequenceRun) -> None:
        try:
            run.report_file = save_report(run)
        except OSError as exc:
            run.message += f" Report not saved: {exc}"
        self.push_run()


def _copy(t: ToolStatus) -> ToolStatus:
    return ToolStatus(**vars(t))


def _summary_rows(run: SequenceRun) -> List[list]:
    """Report summary block."""
    rows = []
    rows.append(["flange", run.profile["display_name"]])
    rows.append(["started", run.started_at])
    rows.append(["result", run.phase, run.message])
    setup = run.profile.get("setup") or {}
    rows.append(["stud", setup.get("bolt", ""), "bolt_count", run.profile["bolt_count"]])
    rows.append(["setting", setup.get("label", "")])
    scale = setup.get("scale", 1.0)
    full = lambda n: n / scale   # to full-size tension
    if scale != 1.0:
        rows.append(["bench_scale", float(f"{scale:.6g}"), "load_cell_target_N", round(run.profile["target_n"], 2),
                     "note", "kN / lbf columns are scaled up from the load cell"])
    rows.append(["target_kN", round(full(run.profile["target_n"]) / 1000, 3),
                 "target_lbf", round(full(run.profile["target_n"]) / N_PER_LBF)])
    rows.append(["max_kN", round(full(run.profile["max_n"]) / 1000, 3),
                 "max_lbf", round(full(run.profile["max_n"]) / N_PER_LBF)])
    if setup.get("method") == "table":
        over = setup["max_n"] / setup["target_n"]
        rows.append(["target_tension", setup["tension_native"], setup["tension_unit"],
                     "min_torque", setup["torque_native"], setup["torque_unit"]])
        rows.append(["max_tension", float(f"{setup['tension_native'] * over:.6g}"), setup["tension_unit"],
                     "max_torque", float(f"{setup['torque_native'] * over:.6g}"), setup["torque_unit"]])
    rows.append([])
    rows.append(["bolt", "tool_id", "group", "status", "final_kN", "final_lbf", "load_cell_N", "retightens", "note"])
    for b in run.bolts:
        none = b.final_n is None
        rows.append([b.bolt, b.tool_id, b.group, b.status,
                     "" if none else round(full(b.final_n) / 1000, 3),
                     "" if none else round(full(b.final_n) / N_PER_LBF),
                     "" if none else round(b.final_n, 2), b.retightens, b.note])
    return rows


# Scaled-chart time step in s; None = auto (0.1 s, widened for long runs)
REPORT_STEP_S = None

LOG_HEADER = ["time_s", "bolt", "tool_id", "state", "stop_value_N", "raw_N", "relay_on", "stop_value_kN_full_size"]


def _log_rows(run: SequenceRun) -> List[list]:
    """Load log grouped by bolt, one chart series each."""
    scale = (run.profile.get("setup") or {}).get("scale", 1.0)
    rows = []
    for t, bolt, tool_id, state, filt, avg, raw, relay in sorted(run.log, key=lambda r: (r[1], r[0])):
        rows.append([round(t, 2), bolt, tool_id, state, round(filt, 2), round(raw, 2),
                     1 if relay else 0, round(filt / scale / 1000, 3)])
    return rows


def save_report(run: SequenceRun) -> str:
    REPORTS_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = REPORTS_DIR / f"{stamp}_{run.profile['id']}.xlsx"
    scale = (run.profile.get("setup") or {}).get("scale", 1.0)
    write_report(path, _summary_rows(run), LOG_HEADER, _log_rows(run),
                 f"Load vs time: {run.profile['display_name']}",
                 run.profile["target_n"], run.profile["max_n"],
                 scaled_col=LOG_HEADER.index("stop_value_kN_full_size"),
                 scaled_target=round(run.profile["target_n"] / scale / 1000, 3),
                 scaled_max=round(run.profile["max_n"] / scale / 1000, 3),
                 step_s=REPORT_STEP_S)
    return path.name


# ------------------------------------------------------------------ app ----

PROFILE_DEFAULTS = {"type": "Custom", "pressure_class_metric": "N/A", "pressure_class_imperial": "N/A",
                    "socket_size_metric": "N/A", "socket_size_imperial": "N/A"}


def default_assignment(profile: dict) -> List[Optional[str]]:
    return profile.get("tools") or [f"TOOL_{i:02d}" for i in range(1, profile["bolt_count"] + 1)]


def load_profiles() -> List[dict]:
    # Targets come from bolt_profiles.json
    with PROFILES_PATH.open(encoding="utf-8") as f:
        profiles = json.load(f)["profiles"]
    bp = load_bolt_profiles()
    return [{**PROFILE_DEFAULTS, **p, "bolt_setup": bolt_setup(p, bp)} for p in profiles]


def create_app(dash: Dashboard, mode_label: str) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        dash.attach(asyncio.get_running_loop())
        dash.controller.connect()
        poller = asyncio.ensure_future(_poll_list())
        yield
        poller.cancel()
        dash.controller.close()

    app = FastAPI(lifespan=lifespan)

    async def _poll_list() -> None:
        # Refresh online flags
        while True:
            await asyncio.sleep(1.0)
            for view in dash.tools.values():
                dash._broadcast({"type": "tool", "data": view.to_json()})

    @app.get("/api/info")
    async def info():
        return {"mode": mode_label, "n_per_count": dash.n_per_count, "bench_target_n": dash.bench_target_n}

    @app.get("/api/profiles")
    async def profiles():
        return load_profiles()

    @app.get("/api/tools")
    async def tools():
        return [v.to_json() for v in sorted(dash.tools.values(), key=lambda v: v.tool_id)]

    @app.post("/api/start/{profile_id}")
    async def start(profile_id: str, request: Request):
        if dash.run and dash.run.phase in ("starting", "taring", "tightening"):
            return JSONResponse({"error": "A run is already in progress."}, status_code=409)
        profile = next((p for p in load_profiles() if p["id"] == profile_id), None)
        if not profile:
            return JSONResponse({"error": "Unknown flange."}, status_code=404)
        try:
            body = await request.json()
        except ValueError:
            body = {}
        try:
            setup = resolve(profile, load_bolt_profiles(), body.get("stress_id"), body.get("friction"))
        except ValueError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        # target_n/max_n: load-cell N; setup: full size
        scale = 1.0
        if dash.bench_target_n and setup["method"] == "table":
            scale = dash.bench_target_n / setup["target_n"]
        setup["scale"] = scale
        profile = {**profile, "setup": setup, "target_n": round(setup["target_n"] * scale, 2),
                   "max_n": round(setup["max_n"] * scale, 2)}
        assignment = body.get("assignment") or default_assignment(profile)
        assignment = (list(assignment) + [None] * profile["bolt_count"])[: profile["bolt_count"]]
        fitted = [t for t in assignment if t]
        if not fitted:
            return JSONResponse({"error": "Attach a tool to at least one bolt."}, status_code=400)
        if len(set(fitted)) != len(fitted):
            return JSONResponse({"error": "A tool is attached to more than one bolt."}, status_code=400)
        return dash.start(profile, assignment, bool(body.get("continue_job"))).to_json()

    @app.post("/api/cancel")
    async def cancel():
        dash.cancel()
        return {"status": "stopped"}

    def run_active() -> bool:
        return bool(dash.run and dash.run.phase in ("starting", "taring", "tightening"))

    @app.post("/api/tools/{action}")
    async def tools_action(action: str):
        actions = {"tare_all": dash.controller.tare_all, "stop_all": dash.controller.stop_all,
                   "reset_all": dash.controller.reset_all, "list": dash.controller.list_tools,
                   "loosen_all": dash.controller.loosen_all}
        if action not in actions:
            return JSONResponse({"error": "Unknown action."}, status_code=404)
        if action == "loosen_all" and run_active():
            return JSONResponse({"error": "Stop the tightening run before loosening."}, status_code=409)
        actions[action]()
        return {"status": "sent"}

    @app.post("/api/tool/{tool_id}/loosen")
    async def tool_loosen(tool_id: str):
        if run_active():
            return JSONResponse({"error": "Stop the tightening run before loosening."}, status_code=409)
        if tool_id not in dash.tools:
            return JSONResponse({"error": "Unknown tool."}, status_code=404)
        dash.controller.loosen(tool_id)
        return {"status": "sent"}

    @app.get("/api/reports/{name}")
    async def report(name: str):
        path = (REPORTS_DIR / name).resolve()
        if path.parent != REPORTS_DIR.resolve() or not path.exists():
            return JSONResponse({"error": "Not found."}, status_code=404)
        return FileResponse(path, filename=name)

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        dash.sockets.append(ws)
        for view in dash.tools.values():
            await ws.send_json({"type": "tool", "data": view.to_json()})
        if dash.run:
            await ws.send_json({"type": "run", "data": dash.run.to_json()})
        try:
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            if ws in dash.sockets:
                dash.sockets.remove(ws)

    app.mount("/", StaticFiles(directory=HERE / "static", html=True), name="static")
    return app


def main() -> int:
    parser = argparse.ArgumentParser(description="Torque tool network dashboard")
    parser.add_argument("--port", help="Controller serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sim", type=int, metavar="N", help="Run with N simulated tools instead of hardware")
    parser.add_argument("--n-per-count", type=float, default=DEFAULT_N_PER_COUNT,
                        help="Load-cell calibration, must match the tool firmware (default 0.19975)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="Seconds a bolt may take to reach target (default 120)")
    parser.add_argument("--bench-target", type=float, metavar="N",
                        help="Bench scale: run every flange target at N newtons on the load cell "
                             "(e.g. 30) and show readings scaled up to full-size tension and torque")
    parser.add_argument("--sim-loosen", type=float, default=0.0, metavar="F",
                        help="With --sim: how much tightening a bolt loosens the bolts already at target "
                             "(0 = never, 0.3 = noticeably, so re-tightening can be seen)")
    parser.add_argument("--sim-stall", type=float, metavar="P",
                        help="With --sim: chance (0-1) that a simulated bolt stalls below target (default 0.03)")
    parser.add_argument("--sim-overshoot", type=float, default=0.0, metavar="P",
                        help="With --sim: chance (0-1) that a simulated bolt overshoots past the max when it reaches target")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if args.bench_target is not None and args.bench_target <= 0:
        parser.error("--bench-target must be greater than 0")

    if args.sim:
        from simulated_controller import SimulatedController
        sim_opts = {"loosen": args.sim_loosen, "rate_spread": 0.35, "overshoot_chance": args.sim_overshoot}
        if args.sim_stall is not None:
            sim_opts["stall_chance"] = args.sim_stall
        controller = SimulatedController(args.sim, args.n_per_count, **sim_opts)
        mode = f"Simulated ({args.sim} tools)"
        timeout = min(args.timeout, 30.0)
    else:
        try:
            port = choose_port(args.port)
            open_port(port, args.baud).close()   # fail early
        except RuntimeError as exc:
            print(f"Error: {exc}")
            return 1
        controller = ToolNetworkController(port, args.baud)
        mode = port
        timeout = args.timeout

    if args.bench_target:
        mode += f" · bench {args.bench_target:g} N"
    serve(controller, mode, args.n_per_count, timeout, args.http_port, not args.no_browser, args.bench_target)
    return 0


def serve(controller: ToolNetworkController, mode: str, n_per_count: float, timeout_s: float,
          http_port: int = 8000, open_browser: bool = True, bench_target_n: Optional[float] = None) -> None:
    dash = Dashboard(controller, n_per_count, timeout_s, bench_target_n)
    app = create_app(dash, mode)
    url = f"http://localhost:{http_port}"
    print(f"Dashboard on {url} ({mode}). Ctrl+C to stop.")
    if open_browser:
        webbrowser.open(url)
    uvicorn.run(app, host="127.0.0.1", port=http_port, log_level="warning")


if __name__ == "__main__":
    raise SystemExit(main())
