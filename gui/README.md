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

## Locked-rotor DC supply acquisition

The optional ITECH IT-M3902C-80-40 reader uses internal ELOG sampling rather
than polling voltage/current measurements. Enable it in the locked-rotor dialog
and set the IPv4 address, sample period (default 0.01 s / 100 Hz) and finite
duration (default 30 s). The duration is independent of the sweep: choose enough
time to cover it. Completion of the sweep, a fault, or STOP aborts any remaining
ELOG acquisition and downloads the partial buffer.

Each session uses a persistent TCP socket on port 30000, enters remote mode,
reconfigures ELOG in ASCII with voltage/current enabled and peaks disabled, and
uses a manual trigger only after observing `wait trigger`. State queries do not
determine the sample cadence. No output, setpoint, operating-mode or protection
commands are sent. Network interfaces and routes are not changed.

Current and power plots appear after the complete LF-terminated response is
downloaded. Their time axis is ELOG-relative, independent of the drive plots and
their scrollbar. All downloaded points are retained in these plots; power is
`V[n] * I[n]`, preserving the sign. No alignment with drive telemetry is assumed.

ELOG data is automatically saved to the application's local data directory
(the full path is displayed in the supply status). **Export Samples CSV**
exports the drive CSV and an adjacent `<name>-supply.csv` containing:

```text
elog_time_s,supply_voltage_v,supply_current_a,supply_power_w,interrupted
```

There is one row per instrument sample, with `elog_time_s = n * period`.
`interrupted` is 1 for a buffer fetched after abort, otherwise 0. Supply readings
are no longer repeated in drive telemetry rows. Missing/malformed data, timeout,
unexpected states and sample-count discrepancies are reported explicitly.
A failed/incomplete download is not exported as a valid dataset. After a socket
failure, starting a new sweep establishes a fresh connection; acquisition is
never silently restarted or stitched across reconnects. Instrument buffer
capacity is not assumed unlimited, and downloads have a 30 s total timeout and
a 128 MiB software response limit. On application shutdown, unfinished
acquisition/download is cancelled; only already saved data is persistent.

The socket/state-machine regression tests use a simulated instrument and do
not contact the bench:

```bash
pixi run python tests/test_power_supply.py
```

## No-load speed test

**Tools > No-load Speed Test...** holds one DUT joint at a single target speed
in PVT. Set the signed target in rpm, speed ramp, symmetric raw torsiometer
torque band, speed tolerance and continuous in-band dwell. Manually control the
Yaskawa outside this application; this tool sends it no commands. External DAQ
is mandatory: Start is disabled without fresh valid samples. Close the other
test dialogs before starting; motion and supply acquisition are exclusive with
the locked-rotor and damped-current tools.

Speed hold uses `OP_PVT`, zero torque feedforward and the **PVT KP / PVT KD**
values entered in the dialog. They are prefilled from the selected joint's
configuration, use raw drive register units (not SI units), and apply only to
this test: the TOML and the joint's configured gains are not changed. Both gains
must be non-negative and at least one must be positive. Stop before editing
and start again to try different gains; no application restart is needed.

With `KP>0`, the controller integrates the ramped speed into a moving position
reference, beginning at the measured starting position. There is no final
position to enter. `KP` corrects accumulated tracking lag and `KD` corrects
velocity error. With `KP=0`, the position command follows measured position and
only velocity feedback is used, without accumulating a position reference.
Actual speed can differ under load, so the dialog still verifies measured speed
tolerance. Unsupported drivers, another operating mode or invalid parameters
are rejected by the worker, and worker errors cancel the dialog's motion state.
Opening this dialog starts neither DUT motion nor PSU acquisition.

Plots on the right of the settings show, over a rolling 20 s window: speed
(reference, drive-reported actual and torsiometer speed, in rpm) and raw
torsiometer torque with the `±band` lines. Live traces use the dialog's GUI
clock for display only. When an acquisition ends, the downloaded DC supply
power `V x I` is plotted against ELOG-relative time with its mean and sample
count; it is marked VALID/INVALID like the dataset.
The live raw torque readout also shows a rolling arithmetic mean of the last
100 distinct valid samples, updated approximately every 0.5 s (2 Hz).
During warm-up it averages the available samples and displays the count out of
100. Samples are polled by the GUI, not the full-rate DAQ stream; at the nominal
40 Hz GUI polling rate the full window spans about 2.5 s.
The average resets on stale/invalid data or a reader change. It is display-only:
no-load qualification, plots and exported sensor samples still use raw torque.

**Continuous rotation - no travel limits** is unchecked by default. Checking it
requires an explicit confirmation at Start that unlimited travel is safe; leave
it unchecked for bounded travel. This does not bypass drive protections or the
finite position-PDO range: with `KP>0`, the test explicitly stops before the
reference or measured position reaches signed 32-bit rollover, keeping at least
one revolution (or one fast control tick) clear of the boundary. Firmware
position-error behavior across rollover is not assumed. Use `KP=0` for operation
without an accumulated position-reference range limit.
Settings are frozen while the DUT rotates or
the supply download is pending. Both measured DUT speed and external encoder
speed must be within the configured tolerance, and raw external torque must
remain inside `[-band, +band]` throughout dwell and finite capture.

The PSU host, instrument sample period and common finite acquisition duration
are required. After dwell, ELOG starts automatically. Sensor capture begins when
the GUI observes instrument acquisition; socket/start-observation latency is
not treated as clock synchronization. The configured finite duration controls
instrument ELOG; sensor polling continues until the supply reader finishes,
including download latency, rather than ending early on a separate GUI timer.
This wider sensor interval is documented in CSV metadata; the files are not
claimed to have identical start/end instants. Acquisition/download errors, interrupted
or partial buffers and stale/invalid DAQ are invalid results, never successful
exports. Stale telemetry, worker errors, faults, Stop, disconnect and closing
the dialog cancel an unfinished capture and release the chosen joint.

Successful acquisition **keeps the DUT rotating until Stop**. It does not
automatically repeat: use **Rearm acquisition** after download completes to
require a new dwell and collect another finite dataset. Dialog Stop releases
only its chosen joint; the main window's emergency stop/Esc still stops all
joints. Neither action controls the external Yaskawa.

Each finished dataset is automatically saved atomically with `QSaveFile` in
`/home/preddi/Documents/test-actuator-analysis/no_load/<session>/`, where the
session identifier includes a UTC timestamp and unique suffix. The destination
is independent of the launching user's home (including when run with sudo).
Each acquisition has its own directory containing separate `no-load-*-sensor.csv` and
`no-load-*-supply.csv` files; paths and validity are displayed. Invalid partial
datasets are saved with `no_load_valid=0` for diagnosis. Valid data can be copied
with **Export CSV copies...**. Metadata includes the temporary KP/KD values,
target rpm, band, dwell, speed
tolerance, acquisition duration, supply period, unlimited-travel selection and
validity. Sensor rows retain their own `daq_time_s`, raw/filtered torque and
speed; supply rows retain `elog_time_s`, voltage and current. The sensor CSV is
explicitly **polled latest DAQ samples, not full-rate acquisition**; duplicate
DAQ timestamps are omitted. Clocks are independent: synchronize only in
post-processing. This tool performs no power calculations.

Hardware-free offscreen dialog regressions cover dwell/reset, freshness,
duplicate suppression, capture excursions, speed tolerance, manual rearm,
temporary gain forwarding, joint gain selection and gain metadata.
They simulate sample publication and override acquisition startup; no DAQ,
EtherCAT bus or instrument is opened. With the Ninja GUI build configured:

```bash
pixi run python tests/test_no_load_gui.py
pixi run python tests/test_speed_hold.py
```

The speed-hold core tests cover gain validation, PDO representation bounds,
positive/negative ramps, position integration across the ramp/hold transition,
position rollover guards, indefinite velocity-only holding and bounded travel
limits, including measured velocity during reversals.
Conflicting motion is blocked until speed hold is
released; release applies only to the matching speed-hold activity and does not
stop a different activity.

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
