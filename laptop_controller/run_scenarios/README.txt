Double-click a script to run it. Each opens a window; close the window or
press Ctrl+C to stop. The dashboard opens in your browser at http://localhost:8000.

  0_first_time_setup          install the Python packages (once per laptop)
  1_simulation_ideal          no hardware, 32 tools, every bolt reaches target
  2_simulation_with_stalls    no hardware, 12 tools, some stall or over-tighten
  3_simulation_loosening      no hardware, 12 tools, bolts loosen and get re-tightened
  4_real_tools_bench_scale    real tools, any flange scaled to BENCH_TARGET N
  5_real_tools_bench_rig      real tools, Bench Rig flange at 50 N, no scaling
  6_text_controller           real tools, typed commands instead of the dashboard
  7_text_controller_imperial  as 6, readings in lbf

Change the COM port or bench target once, in settings.bat.
Only one program can use the COM port at a time: close the Arduino Serial
Monitor (and any other dashboard window) before running 4 to 7.
