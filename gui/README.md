# Actuator Characterisation GUI

A modular Qt6 desktop application for driving EtherCAT actuators through
characterisation trajectories with jitter-free 1&nbsp;kHz logging. It reuses the
exact same control core as the console tool (`actuator-test-spline`), so both
front-ends behave identically on the hardware.

## Build

Qt6 Widgets ships in the pixi environment (`qt6-main`). The GUI is built by
default via the `BUILD_GUI` CMake option:

```bash
pixi run build          # builds actuator-test-spline and actuator-test-gui
```

To build without the GUI:

```bash
cmake -S . -B build -DBUILD_GUI=OFF && cmake --build build
```

## Run

The application needs the same Linux capabilities as the console tool
(`cap_net_raw,cap_net_admin,cap_sys_nice`). A convenience task applies them and
launches the app:

```bash
pixi run run-gui
```

Or manually:

```bash
pixi run gui-capabilities          # one-time setcap on the built binary
./build/gui/actuator-test-gui [config.toml]
```

The optional positional argument is the device-config TOML (defaults to
`../config/gene-000.toml`). On launch a short wizard explains the workflow and
collects the config path, then the main window connects to the bus.

## Workflow

1. **Connection** dock &mdash; pick the config, **Connect**, then tick the joints
   you want to act on.
2. **Limits** dock &mdash; **Start capture** and backdrive the joint(s) to record
   the travel envelope, or type explicit min/max and **Apply**.
3. **Jog / Home** dock &mdash; press-and-hold jog, **Go to centre**, or **Home**.
4. **Trajectory** dock &mdash; choose a waveform, toggle CSV logging, **Play** /
   **Stop**.
5. **Plots** (centre) &mdash; live reference vs. actual and tracking error for the
   selected joint. **Telemetry** and **Log** docks show per-joint state.

## Trajectories

All parametric waveforms are generated about the captured mid-point with an
amplitude of `traj_safety_factor &times; half-range`:

| Mode | Use |
| --- | --- |
| Sinusoid | Fixed-frequency baseline. |
| Chirp (linear) | Frequency sweep `f0&rarr;f1&rarr;f0`, C1-continuous. |
| Chirp (log) | Exponential frequency sweep for wide-band ID. |
| Triangle | Constant-velocity sweep (friction / range). |
| Step | Square wave for step-response characterisation. |
| Multisine | Schroeder-phased harmonics for one-shot FRF. |

Tuning lives in the `[actuator_test]` section of the device config, e.g.:

```toml
[actuator_test]
traj_freq_hz = 0.5
chirp_f0_hz = 0.1
chirp_f1_hz = 5.0
chirp_sweep_seconds = 20.0
triangle_cycle_seconds = 4.0
step_cycle_seconds = 4.0
multisine_base_hz = 0.1
multisine_harmonics = 10
```

## Architecture

```
gui/
  core/
    telemetry.hpp          # control-thread -> GUI snapshot types
    commands.hpp           # GUI -> control-thread command variant
    controller_worker.*    # owns the bus; runs the 1 kHz loop on its own thread
  widgets/
    connection_panel.*     # connect/disconnect + joint selection
    jog_panel.*            # press-and-hold jog + homing
    limits_panel.*         # backdrive capture + explicit limits
    trajectory_panel.*     # waveform picker + play/stop
    plot_panel.*           # custom-painted reference/actual/error strip charts
  wizard/
    setup_wizard.*         # onboarding + config selection
  main_window.*            # docks everything together; polls the worker
  main.cpp                 # Qt entry point (config -> profile -> capabilities)
```

The `ControllerWorker` is the only owner of EtherCAT state. The GUI thread never
blocks on the bus: it `post()`s commands and polls `snapshot()` / `drainEvents()`
on a 60&nbsp;Hz timer. The control loop runs on a dedicated real-time thread
(SCHED_FIFO when capabilities allow), keeping the logged 1&nbsp;kHz cadence free
of GUI-induced jitter.
