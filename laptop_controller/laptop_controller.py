"""
Serial API for the controller ESP32 plus a text CLI.

    python laptop_controller.py [--port COM4]
"""

from __future__ import annotations

import argparse
import glob
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import serial
import serial.tools.list_ports


@dataclass
class ToolStatus:
    tool_id: str
    state: str = "UNKNOWN"
    target_n: float = 0.0
    raw_counts: int = 0
    filtered_counts: int = 0
    filtered_avg_counts: int = 0
    relay_on: bool = False
    online: bool = True
    mac: str = ""
    last_seen: float = field(default_factory=time.time)
    last_status: float = 0.0  # last STATUS time


class ToolNetworkController:
    """Serial API used by the CLI and the dashboard."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.tools: Dict[str, ToolStatus] = {}
        self.events: "queue.Queue[str]" = queue.Queue()
        self.on_status: Optional[Callable[[ToolStatus], None]] = None
        self.on_event: Optional[Callable[[str], None]] = None
        self._running = False
        self._reader: Optional[threading.Thread] = None
        self._write_lock = threading.Lock()

    def connect(self) -> None:
        self.serial = open_port(self.port, self.baudrate)

        # Opening the port can reset the board
        time.sleep(2.0)
        self.serial.reset_input_buffer()

        self._running = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self.list_tools()

    def close(self) -> None:
        self._running = False
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()

    def _send(self, command: str) -> None:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Controller is not connected.")

        with self._write_lock:
            self.serial.write((command.strip() + "\n").encode("utf-8"))
            self.serial.flush()


    def list_tools(self) -> None:
        self._send("LIST")

    def set_target(self, tool_id: str, target_n: float) -> None:
        if target_n <= 0:
            raise ValueError("Target must be greater than 0 N.")
        self._send(f"TARGET,{tool_id},{target_n:.4f}")

    def stop(self, tool_id: str) -> None:
        self._send(f"STOP,{tool_id}")

    def tare(self, tool_id: str) -> None:
        self._send(f"TARE,{tool_id}")

    def reset(self, tool_id: str) -> None:
        self._send(f"RESET,{tool_id}")

    def loosen(self, tool_id: str) -> None:
        """Reverse run; the tool stops itself."""
        self._send(f"LOOSEN,{tool_id}")

    def set_target_all(self, target_n: float) -> None:
        if target_n <= 0:
            raise ValueError("Target must be greater than 0 N.")
        self._send(f"TARGET_ALL,{target_n:.4f}")

    def stop_all(self) -> None:
        self._send("STOP_ALL")

    def tare_all(self) -> None:
        self._send("TARE_ALL")

    def reset_all(self) -> None:
        self._send("RESET_ALL")

    def loosen_all(self) -> None:
        self._send("LOOSEN_ALL")


    def _reader_loop(self) -> None:
        while self._running and self.serial and self.serial.is_open:
            try:
                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self._handle_line(line)
            except (serial.SerialException, OSError) as exc:
                self._emit_event(f"SERIAL_ERROR,{exc}")
                self._running = False
                break

    def _handle_line(self, line: str) -> None:
        parts = line.split(",")
        kind = parts[0]

        # STATUS,<id>,<state>,<target_N>,<raw>,<stop_value>[,<filtered_avg>],<relay>
        if kind == "STATUS" and len(parts) in (7, 8):
            try:
                tool = self.tools.get(parts[1], ToolStatus(tool_id=parts[1]))
                tool.state = parts[2]
                tool.target_n = float(parts[3])
                tool.raw_counts = int(parts[4])
                tool.filtered_counts = int(parts[5])
                tool.filtered_avg_counts = int(parts[6]) if len(parts) == 8 else tool.filtered_counts
                tool.relay_on = parts[-1] == "1"
                tool.online = True
                tool.last_seen = tool.last_status = time.time()
                self.tools[tool.tool_id] = tool

                if self.on_status:
                    self.on_status(tool)
            except ValueError:
                self._emit_event(f"PARSE_ERROR,{line}")
            return

        if kind == "TOOL" and len(parts) >= 4:
            tool = self.tools.get(parts[1], ToolStatus(tool_id=parts[1]))
            tool.online = parts[2] == "ONLINE"
            tool.mac = parts[3]
            tool.last_seen = time.time()
            self.tools[tool.tool_id] = tool
            if self.on_status:
                self.on_status(tool)
            return

        if kind == "EVENT" and len(parts) >= 2:
            # EVENT,DISCOVERED,<tool>,<mac>
            if parts[1] == "DISCOVERED" and len(parts) >= 4:
                tool = self.tools.get(parts[2], ToolStatus(tool_id=parts[2]))
                tool.mac = parts[3]
                tool.online = True
                tool.last_seen = time.time()
                self.tools[tool.tool_id] = tool
                if self.on_status:
                    self.on_status(tool)

        self._emit_event(line)

    def _emit_event(self, line: str) -> None:
        self.events.put(line)
        if self.on_event:
            self.on_event(line)


def available_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def open_port(port: str, baudrate: int = 115200) -> serial.Serial:
    """Open the port with readable errors."""
    try:
        return serial.Serial(port, baudrate, timeout=0.2, write_timeout=1.0)
    except serial.SerialException as exc:
        if "Access is denied" in str(exc) or "PermissionError" in str(exc) or "busy" in str(exc).lower():
            raise RuntimeError(
                f"{port} is in use by another program. Close the Arduino Serial Monitor / Plotter "
                f"and any other dashboard or laptop_controller window, then try again.") from exc
        raise RuntimeError(f"Could not open {port}: {exc}") from exc


def choose_port(requested: Optional[str]) -> str:
    ports = list(serial.tools.list_ports.comports())
    if requested:
        if any(p.device.upper() == requested.upper() for p in ports):
            return requested
        print(f"{requested} was not found. Is the controller plugged in, and is that its port?")

    if not ports:
        raise RuntimeError("No serial ports found. Plug in the controller ESP32-C3 by USB.")

    # Prefer ESP32/USB serial ports
    keywords = ("ESP", "USB", "JTAG", "UART", "CP210", "CH340")
    preferred = [
        p for p in ports
        if any(k.lower() in (p.description or "").lower() for k in keywords)
    ]

    if len(preferred) == 1:
        return preferred[0].device
    if len(ports) == 1:
        return ports[0].device

    print("Available serial ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")

    while True:
        try:
            selection = int(input("Select controller port: "))
            if 1 <= selection <= len(ports):
                return ports[selection - 1].device
        except ValueError:
            pass
        print("Invalid selection.")


N_PER_LBF = 4.4482216
DISPLAY = {"imperial": False, "n_per_count": 0.19975}


def fmt_force(newtons: float) -> str:
    if DISPLAY["imperial"]:
        return f"{newtons / N_PER_LBF:,.1f} lbf"
    return f"{newtons / 1000:,.2f} kN" if abs(newtons) >= 10000 else f"{newtons:.1f} N"


def to_newtons(value: float) -> float:
    """Prompt value in current units -> N."""
    return value * N_PER_LBF if DISPLAY["imperial"] else value


def print_status(tool: ToolStatus) -> None:
    k = DISPLAY["n_per_count"]
    print(
        f"[{tool.tool_id}] {tool.state:<14} "
        f"target={fmt_force(tool.target_n)}  "
        f"measured={fmt_force(tool.filtered_counts * k)} ({tool.filtered_counts} counts)  "
        f"raw={fmt_force(tool.raw_counts * k)}  "
        f"relay={'ON' if tool.relay_on else 'OFF'}"
    )


def print_help() -> None:
    print(
        """
Commands:
  list
  target <TOOL_ID> <force>        (N, or lbf with --units imperial)
  stop <TOOL_ID>
  tare <TOOL_ID>
  reset <TOOL_ID>
  loosen <TOOL_ID>                (reverse; stops by itself after 15 s)
  target-all <force>
  stop-all
  tare-all
  reset-all
  loosen-all
  status
  help
  quit
""".strip()
    )


def run_cli(controller: ToolNetworkController) -> None:
    controller.on_status = print_status
    controller.on_event = lambda line: print(f"[controller] {line}")

    print_help()

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if not line:
            continue

        parts = line.split()
        command = parts[0].lower()

        try:
            if command in ("quit", "exit"):
                return
            elif command == "help":
                print_help()
            elif command == "list":
                controller.list_tools()
            elif command == "status":
                if not controller.tools:
                    print("No tools discovered yet.")
                for tool in controller.tools.values():
                    print_status(tool)
            elif command == "target" and len(parts) == 3:
                controller.set_target(parts[1], to_newtons(float(parts[2])))
            elif command == "stop" and len(parts) == 2:
                controller.stop(parts[1])
            elif command == "tare" and len(parts) == 2:
                controller.tare(parts[1])
            elif command == "reset" and len(parts) == 2:
                controller.reset(parts[1])
            elif command == "loosen" and len(parts) == 2:
                controller.loosen(parts[1])
            elif command == "target-all" and len(parts) == 2:
                controller.set_target_all(to_newtons(float(parts[1])))
            elif command == "stop-all":
                controller.stop_all()
            elif command == "tare-all":
                controller.tare_all()
            elif command == "reset-all":
                controller.reset_all()
            elif command == "loosen-all":
                controller.loosen_all()
            else:
                print("Unknown/invalid command. Type 'help'.")
        except (ValueError, RuntimeError, serial.SerialException) as exc:
            print(f"Error: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Torque tool network laptop controller")
    parser.add_argument("--port", help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--units", choices=["metric", "imperial"], default="metric",
                        help="Show and type forces in N/kN (metric) or lbf (imperial)")
    parser.add_argument("--n-per-count", type=float, default=0.19975,
                        help="Load-cell calibration, must match N_PER_COUNT in the tool firmware")
    args = parser.parse_args()
    DISPLAY["imperial"] = args.units == "imperial"
    DISPLAY["n_per_count"] = args.n_per_count

    try:
        port = choose_port(args.port)
        print(f"Connecting to Tool Network Controller on {port}...")
        controller = ToolNetworkController(port, args.baud)
        controller.connect()
    except (RuntimeError, serial.SerialException) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        return 1

    try:
        run_cli(controller)
    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
