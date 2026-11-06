"""
simulated_controller.py: fake controller and tools for running the dashboard without hardware.

Emits the real serial protocol through ToolNetworkController._handle_line and
mirrors the tool firmware's states. Options: stall_chance, loosen (neighbour
loosening), overshoot_chance, rate_spread.
"""

from __future__ import annotations

import random
import threading
import time
from dataclasses import dataclass
from typing import Dict

from laptop_controller import ToolNetworkController

TICK_S = 0.25              # firmware STATUS_INTERVAL_MS
TARE_S = 1.5
RAMP_GAIN = 0.12           # fraction of error closed per tick
# Step and noise scale with the target
MAX_STEP_FRAC = 0.05
MIN_MAX_STEP_N = 6.0
NOISE_FRAC = 0.002
MIN_NOISE_N = 0.3
STALL_CHANCE = 0.03
STALL_CEILING_FRAC = (0.75, 0.92)
OVERSHOOT_FRAC = (1.28, 1.4)   # past the 25 % max
LOOSEN_S = 15.0
LOOSEN_DECAY = 0.08        # preload lost per tick


@dataclass
class _SimTool:
    tool_id: str
    mac: str
    state: str = "IDLE"
    target_n: float = 0.0
    force_n: float = 0.0
    min_n: float = 0.0
    relay_on: bool = False
    control_target_n: float = 0.0
    gain: float = RAMP_GAIN
    busy_until: float = 0.0
    pending_state: str = ""
    last_step_n: float = 0.0
    loosen_until: float = 0.0
    after_loosen: str = "IDLE"
    tared: bool = False        # firmware tareValid


class SimulatedController(ToolNetworkController):
    """ToolNetworkController with fake tools."""

    def __init__(self, tool_count: int = 12, n_per_count: float = 0.19975,
                 stall_chance: float = STALL_CHANCE, rate_spread: float = 0.0, loosen: float = 0.0,
                 overshoot_chance: float = 0.0):
        super().__init__(port="SIMULATED")
        self.n_per_count = n_per_count
        self.stall_chance = stall_chance
        self.neighbour_loosen = loosen   # self.loosen is the command
        self.overshoot_chance = overshoot_chance
        self._sims: Dict[str, _SimTool] = {}
        self._lock = threading.Lock()
        for i in range(1, tool_count + 1):
            tool_id = f"TOOL_{i:02d}"
            mac = "SI:MU:LA:TE:%02X:%02X" % (i // 256, i % 256)
            gain = RAMP_GAIN * random.uniform(1 - rate_spread, 1 + rate_spread)
            self._sims[tool_id] = _SimTool(tool_id, mac, gain=gain)

    # ---- Transport overrides ----

    def connect(self) -> None:
        self._running = True
        self._handle_line("EVENT,CONTROLLER_READY")
        for sim in self._sims.values():
            self._handle_line(f"EVENT,DISCOVERED,{sim.tool_id},{sim.mac}")
        self._reader = threading.Thread(target=self._sim_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._running = False
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=1.0)

    def _send(self, command: str) -> None:
        parts = command.strip().split(",")
        kind = parts[0]
        with self._lock:
            if kind == "LIST":
                for sim in self._sims.values():
                    self._handle_line(f"TOOL,{sim.tool_id},ONLINE,{sim.mac}")
                return
            if kind in ("STOP_ALL", "TARE_ALL", "RESET_ALL", "LOOSEN_ALL"):
                for tool_id in self._sims:
                    self._apply(kind.split("_")[0], tool_id, None)
                return
            if kind == "TARGET_ALL" and len(parts) == 2:
                for tool_id in self._sims:
                    self._apply("TARGET", tool_id, float(parts[1]))
                return
            if len(parts) < 2 or parts[1] not in self._sims:
                self._handle_line(f"ERROR,UNKNOWN_TOOL,{parts[1] if len(parts) > 1 else ''}")
                return
            value = float(parts[2]) if len(parts) > 2 else None
            self._apply(kind, parts[1], value)

    # ---- Simulated tool behaviour ----

    def _apply(self, kind: str, tool_id: str, value) -> None:
        sim = self._sims[tool_id]
        sent = f"SENT,{tool_id},{kind}" + (f",{value:.4f}" if kind == "TARGET" else "")
        self._handle_line(sent)
        now = time.time()

        if kind in ("STOP", "RESET", "TARE") and sim.state == "LOOSENING":
            # End a reverse run
            sim.state = sim.after_loosen
            sim.relay_on = False
            sim.loosen_until = 0.0

        if kind in ("STOP", "RESET"):
            # STOP keeps the tare, RESET clears it
            self._reply("ACK", sim, kind)
            sim.relay_on = False
            sim.state = "IDLE"
            sim.target_n = 0.0
            if kind == "RESET":
                sim.tared = False
                sim.min_n = 0.0
            sim.busy_until = 0.0
            sim.pending_state = ""
        elif kind == "LOOSEN":
            if sim.state != "LOOSENING":
                sim.after_loosen = "READY" if sim.tared else sim.state
            sim.relay_on = True
            sim.state = "LOOSENING"
            sim.loosen_until = now + LOOSEN_S
            self._reply("ACK", sim, kind)
        elif kind == "TARE":
            sim.relay_on = False
            sim.busy_until = now + TARE_S
            sim.pending_state = "READY"
            self._reply("ACK", sim, kind)
        elif kind == "TARGET" and value is not None:
            if not sim.tared or sim.busy_until > now:
                self._reply("NACK", sim, kind, value)  # not tared
                return
            if sim.state not in ("MONITORING", "LOOSENING") and sim.min_n >= value:
                # Already at target: relays stay off
                sim.target_n = value
                sim.relay_on = False
                sim.state = "TARGET_REACHED"
                self._reply("ACK", sim, kind)
                return
            # Re-tightens never stall
            again = sim.state == "TARGET_REACHED"
            sim.target_n = value
            stall = not again and random.random() < self.stall_chance
            sim.control_target_n = value * (random.uniform(*STALL_CEILING_FRAC) if stall else 1.02)
            sim.min_n = 0.0
            sim.relay_on = True
            sim.loosen_until = 0.0
            sim.state = "MONITORING"
            self._reply("ACK", sim, kind)

    def _reply(self, kind: str, sim: _SimTool, command: str, value=None) -> None:
        target = sim.target_n if value is None else value
        self._handle_line(f"{kind},{sim.tool_id},{command},{sim.state},{target:.4f}")

    def _sim_loop(self) -> None:
        while self._running:
            now = time.time()
            with self._lock:
                for sim in self._sims.values():
                    self._step(sim, now)
                self._loosen_neighbours()
                for sim in self._sims.values():
                    if sim.busy_until <= now:  # silent mid-tare
                        self._emit_status(sim)
            time.sleep(TICK_S)

    def _loosen_neighbours(self) -> None:
        # Reached bolts lose a share of the others' force gain
        if self.neighbour_loosen <= 0:
            return
        loaded = [s for s in self._sims.values() if s.state in ("MONITORING", "TARGET_REACHED")]
        if len(loaded) < 2:
            return
        total = sum(s.last_step_n for s in loaded)
        for sim in loaded:
            if sim.state != "TARGET_REACHED":
                continue
            others = (total - sim.last_step_n) / (len(loaded) - 1)
            sim.force_n = max(0.0, sim.force_n - self.neighbour_loosen * others * random.uniform(0.5, 1.5))
            sim.min_n = sim.force_n - abs(random.uniform(0, self._noise(sim)))

    def _step(self, sim: _SimTool, now: float) -> None:
        if sim.pending_state and now >= sim.busy_until:
            if sim.pending_state == "READY":
                sim.force_n = 0.0
                sim.min_n = 0.0
                sim.tared = True
            sim.state = sim.pending_state
            if sim.pending_state == "READY":
                self._reply("DONE", sim, "TARE")
            sim.pending_state = ""

        if sim.state == "LOOSENING":
            sim.last_step_n = 0.0
            sim.force_n = max(0.0, sim.force_n * (1 - LOOSEN_DECAY) - MIN_NOISE_N)
            sim.min_n = sim.force_n
            if now >= sim.loosen_until:
                sim.relay_on = False
                sim.state = sim.after_loosen
                sim.loosen_until = 0.0
                self._reply("DONE", sim, "LOOSEN")
            return

        if sim.state == "MONITORING":
            error = sim.control_target_n - sim.force_n
            max_step = max(MIN_MAX_STEP_N, MAX_STEP_FRAC * sim.target_n)
            step = max(-max_step, min(max_step, error * sim.gain))
            sim.last_step_n = max(step, 0.0)
            sim.force_n = max(0.0, sim.force_n + sim.last_step_n)
            sim.min_n = sim.force_n - abs(random.uniform(0, self._noise(sim)))
            if sim.min_n >= sim.target_n:
                sim.relay_on = False
                sim.state = "TARGET_REACHED"
                if random.random() < self.overshoot_chance:
                    sim.force_n = sim.target_n * random.uniform(*OVERSHOOT_FRAC)
                    sim.min_n = sim.force_n
        else:
            sim.last_step_n = 0.0
            if sim.state == "TARGET_REACHED":
                sim.min_n = sim.force_n - abs(random.uniform(0, self._noise(sim)))

    @staticmethod
    def _noise(sim: _SimTool) -> float:
        return max(MIN_NOISE_N, NOISE_FRAC * sim.target_n)

    def _emit_status(self, sim: _SimTool) -> None:
        if not sim.tared:
            raw = mn = 0
        else:
            noise = self._noise(sim)
            noisy = sim.force_n + random.uniform(-noise, noise)
            raw = round(noisy / self.n_per_count)
            mn = round(sim.min_n / self.n_per_count)
        self._handle_line(
            f"STATUS,{sim.tool_id},{sim.state},{sim.target_n:.4f},{raw},{mn},{1 if sim.relay_on else 0}"
        )
