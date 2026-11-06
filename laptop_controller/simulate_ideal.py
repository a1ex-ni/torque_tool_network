"""Simulated run where every bolt reaches target. Default 32 tools (largest flange)."""

import argparse

from dashboard import DEFAULT_N_PER_COUNT, serve
from simulated_controller import SimulatedController


def main() -> int:
    parser = argparse.ArgumentParser(description="Ideal-case simulation of the torque tool network")
    parser.add_argument("--tools", type=int, default=32, help="Number of simulated tools (default 32)")
    parser.add_argument("--spread", type=float, default=0.35,
                        help="How much tool ramp rates differ, 0 = lockstep (default 0.35)")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    controller = SimulatedController(
        args.tools, DEFAULT_N_PER_COUNT, stall_chance=0.0, rate_spread=args.spread
    )
    serve(controller, f"Ideal sim ({args.tools} tools)", DEFAULT_N_PER_COUNT, 30.0,
          args.http_port, not args.no_browser)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
