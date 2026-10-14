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

## Damped current settle and torsiometer TARE

**Tools -> Damped Current Settle...** commands a decaying sinusoidal current on
one joint (amplitude, decay time, frequency, duration) to shake out friction
preload, then holds 0 A for the settle time and measures the mean and standard deviation of the
external raw torque. **TARE** subtracts that mean from every later torsiometer
reading in the application (main plot, locked-rotor and no-load dialogs).
Offsets accumulate across TARE operations and reset only when the application
restarts.

## No-load speed test

**Tools > No-load Speed Test...** holds one DUT joint at a single target speed
in PVT, or records independently while the DUT is controlled elsewhere
(e.g. current commanded from Jog while the Yaskawa holds speed). **Start speed**
is optional and is not needed to acquire. Set the signed target in rpm, speed ramp, symmetric raw torsiometer
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
Actual speed can differ under load. The advisory band uses only torsiometer
torque and speed, not drive-reported speed. Unsupported drivers, another
operating mode or invalid parameters
are rejected by the worker, and worker errors cancel the dialog's motion state.
Opening this dialog starts neither DUT motion nor PSU acquisition.

Plots on the right of the settings show, over a rolling 20 s window: speed
(reference, drive-reported actual and torsiometer speed, in rpm) and raw
torsiometer torque with the `±band` lines. Live traces use the dialog's GUI
clock for display only. When an acquisition ends, the downloaded DC supply
power `V x I` is plotted against ELOG-relative time with its mean and sample
count, without classifying the dataset as VALID or INVALID.
The live raw torque readout also shows a rolling arithmetic mean of the last
100 distinct valid samples, updated approximately every 0.5 s (2 Hz).
During warm-up it averages the available samples and displays the count out of
100. Samples are polled by the GUI, not the full-rate DAQ stream; at the nominal
40 Hz GUI polling rate the full window spans about 2.5 s.
The average resets on stale/invalid data or a reader change. It is display-only:
the advisory band, plots and exported sensor samples still use raw torque.
"Raw" means unfiltered: any TARE applied from the damped current settle dialog
is already subtracted, and its cumulative offset is written to the metadata as
`torsiometer_tare_offset_nm`.

**Continuous rotation - no travel limits** is unchecked by default. Checking it
requires an explicit confirmation at Start that unlimited travel is safe; leave
it unchecked for bounded travel. This does not bypass drive protections or the
finite position-PDO range: with `KP>0`, the test explicitly stops before the
reference or measured position reaches signed 32-bit rollover, keeping at least
one revolution (or one fast control tick) clear of the boundary. Firmware
position-error behavior across rollover is not assumed. Use `KP=0` for operation
without an accumulated position-reference range limit.
Settings are frozen while the DUT rotates or
the supply download is pending. The live band indicator compares raw torsiometer
torque with `[-band, +band]` and torsiometer speed with the target and configured
tolerance. It shows continuous time in band and whether the configured dwell
has been reached, including during acquisition. Drive speed is plotted but is
not part of this check.

The PSU host, instrument sample period and common finite acquisition duration
are required. **Start speed** starts motion and monitors the no-load conditions,
but never starts acquisition automatically. **Start acquisition** is enabled
with a selected connected, non-faulted DUT, fresh DAQ/controller data,
100 Hz DAQ/drive sources, valid PSU settings and no conflicting test or
unfinished acquisition/download. No speed hold, KP/KD, dwell, non-zero speed
or in-band condition is required. Press it to start instrument ELOG. Leaving the torque
or speed band resets only the advisory timer: it does not block starting,
interrupt acquisition or classify data. Freshness is checked again at the
click. Each dataset requires a manual start; after completion, Start acquisition
is available again without Rearm. DAQ and drive buffers start at the manual
acquisition request, before instrument ELOG startup, and stop when the supply
reader finishes (or capture is cancelled). The reader retains whether ELOG was
triggered, so a short acquisition cannot be mistaken for a startup failure
between GUI polls. Socket/start-observation latency is
not treated as clock synchronization. The configured finite duration controls
instrument ELOG; buffered sensor capture continues until the supply reader finishes,
including download latency, rather than ending early on a separate GUI timer.
This wider sensor interval is documented in CSV metadata; the files are not
claimed to have identical start/end instants. Acquisition/download errors and
interrupted or partial buffers are reported explicitly. Available partial
samples can be saved and exported, with interruption and result metadata,
without a no-load validity flag. Stale/invalid DAQ, stale telemetry, worker
errors, faults, Stop, disconnect and closing
the dialog cancel an unfinished capture. The dialog releases the chosen joint
only if it owns speed hold. Capture-only operation does not issue DUT motion
commands; the red Emergency stop requests the worker's immediate Stop even in
capture-only mode. It does not stop the external Yaskawa.

In capture-only mode, Jog current controls remain available while settings
and other test tools are locked during acquisition/download. The selected
rpm is an advisory external-speed target, not a command to the DUT. CSV metadata
identifies external motion control and marks dialog KP/KD as not applied.

**Manual DUT current** provides a signed ampere field with **- / +** buttons,
a selectable adjustment step (minimum and default: 1 permille of the selected
device's rated current) and a current ramp duration
(default 1 s). Each change immediately requests the existing current ramp;
typed values are committed on Enter or focus loss, not on every keystroke.
Targets and steps are rounded to integer permille levels, displayed in amperes
with six decimal places (e.g. a 13.4 A rating gives a 0.0134 A minimum step).
The worker rounds current commands to the nearest integer permille, including
during ramps, instead of truncating toward zero.
The field is bounded by the drive's rated current and the readout distinguishes
the command currently sent from the drive-reported current. Adjustment requires
a selectable MyActuator, captured travel limits, fresh telemetry/DAQ, idle or
current control and no speed hold or conflicting test. Worker protections and
the existing current command's KP=KD=0 behavior are unchanged.

Current adjustments remain available during independent acquisition and do
not cancel it. **Soft stop** in current mode requests a ramp to 0 A and cancels
unfinished acquisition; this does not stop rotation imposed by the load.
The red **Emergency stop** sends the immediate worker Stop, even outside
acquisition. Closing this dialog cancels capture but does not release current
control commanded through this panel or Jog: use Emergency stop to release
the DUT and coordinate the Yaskawa externally.

While running, **Change speed while running -> Apply speed** moves the DUT to a
new target (including 0 rpm) with the configured speed ramp, e.g. to limit supply power.
KP/KD and the travel mode stay as started. A change restarts the dwell; during
an acquisition it aborts it (partial dataset saved with reason "Speed changed
during acquisition") while the DUT keeps rotating, and **Rearm acquisition** is
optional before a new dataset. The metadata `target_rpm` is the speed target
at acquisition, not necessarily the measured speed.

After acquisition and ELOG download, **DUT speed is unchanged** and active PVT
control continues. To decelerate manually, set **0 rpm** and press **Apply speed**,
or use **Soft stop**. Both use the configured speed ramp and preserve the
integrated position reference, KP/KD and travel protections. Acknowledgement
of the zero target is checked; it is not proof that the shaft has physically stopped. Monitor
actual speed and coordinate the external Yaskawa, which this dialog does not
control. Acquisition metadata describes the measurement interval,
not the subsequent deceleration.

To run another acquisition, press **Start acquisition** again after download
completes. **Rearm acquisition** is optional and resets the advisory dwell
while using speed hold; it remains disabled at a zero target or while a speed
change awaits acknowledgement. Start acquisition itself can record at zero speed.
Dialog Stop releases
only its chosen joint; the main window's emergency stop/Esc still stops all
joints. Neither action controls the external Yaskawa.

**Soft stop** manually commands the same ramp to 0 rpm and keeps PVT control
active. It disarms the dwell or aborts an unfinished acquisition, saving that
partial dataset with a "Soft stop requested" reason. Existing completed data
is preserved. Apply a non-zero speed and Rearm to resume testing. The button is
disabled at a zero target or until the worker acknowledges a pending speed
command. **Stop** remains available independently; Soft stop is not an
emergency stop and does not command the external Yaskawa.

Soft stop occupies the top-right button position. The full-width bottom
**Emergency stop** button is red and uses the previous immediate Stop action
(release the DUT and cancel acquisition without a ramp). It is a software
stop, not a certified hardware emergency stop, and does not stop the Yaskawa.

Each finished dataset is automatically saved atomically with `QSaveFile` in
`$ACTUATOR_TEST_ANALYSIS_DIR/no_load/<session>/` (default
`~/Documents/test-actuator-analysis`, using the invoking user's home when run
with sudo), where the session identifier includes a UTC timestamp and unique
suffix.
Each acquisition has its own directory containing separate `no-load-*-sensor.csv`,
`no-load-*-drive.csv` and `no-load-*-supply.csv` files; paths and acquisition
outcomes are displayed.
Partial datasets are saved for diagnosis. Available samples can be copied
with **Export CSV copies...**. Metadata includes the temporary KP/KD values,
target rpm, band, dwell, speed
tolerance, acquisition duration, supply period, unlimited-travel selection and
interruption/result information (no `no_load_valid` field).

DAQ and drive acquisition each preserve **every source update at 100 Hz**,
independently of GUI repaint/polling, using bounded preallocated producer buffers.
Overflow cancels capture with an explicit error and dropped-sample counts in
metadata; it never silently overwrites samples. Acquisition is disabled if
either source's configured publication period is not 10 ms.

- Sensor CSV: `daq_time_s`, raw/filtered torque, speed, `sample_index` and
  `delivery_time_s`. DAQ time is the cumulative analog sample count divided by
  the hardware sampling rate, at each block end. At 50 kHz, 500 samples produce
  one 10 ms update. Raw torque is the last analog value in the block, filtered
  torque processes all analog samples, and speed uses the existing 50 ms
  sliding encoder window and 2 Hz filter. This is **not** a raw 50 kHz recording.
  Delivery time is elapsed host steady-clock time since reader startup and
  exposes delayed block delivery without changing the hardware time axis.
- Drive CSV: `drive_time_s`, actual `drive_speed_rpm`, `cycle_index` and
  `host_time_s`. Drive time follows the EtherCAT cycle count; at 1 kHz control,
  every tenth cycle is saved. Host time is the steady-clock timestamp at
  publication: inspect successive differences to assess scheduling jitter.
  Intervals longer than 15 ms are reported as a timing warning, with their count
  and the maximum host interval in metadata. They do not classify no-load data.
  This is nominal 100 Hz, **not a hard real-time guarantee** of the PC.
- Supply CSV: instrument ELOG time, voltage and current, unchanged.

The drive values are no longer paired with GUI-polled DAQ rows: their separate
CSV preserves the independent stream. Clocks and capture boundaries are not
synchronized; align only in post-processing. GUI plots, band indication and the
100-sample readout average still use latest-sample polling for display.

Hardware-free offscreen dialog regressions cover dwell/reset, freshness,
duplicate suppression, capture excursions, speed tolerance, manual rearm,
temporary gain forwarding, joint gain selection, gain metadata, buffering
across missed GUI polls, independent drive CSV output, overflow reporting and
the exact metadata/column layouts of all three CSV sources.
The same offscreen probe also checks locked-rotor return/opposite-sign sweeps,
repetition, settings locking during motion/download, and dwell timing after
current ramps. These checks issue no hardware commands.
They also cover damped-settle control locking, distinct-sample offset
measurement, tare and restart. The no-load dialog uses one capture phase;
instrument startup/download states and completion belong to the PSU reader,
without duplicate sensor-completion flags or GUI acquisition timestamps.
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

The branch's hardware-free regressions are registered with CTest when
`BUILD_TESTING=ON` (the offscreen dialog tests also require `BUILD_GUI=ON`).
Run the configured build and registered tests together with:

```bash
pixi run test
```

The GUI probe is a normal CMake target sharing the production GUI implementation
and Qt MOC inputs; it does not reconstruct compile/link commands from Ninja
output. Probe assertions remain enabled in Release builds. The standalone
Python commands above remain available for focused checks.

DAQ lifecycle checks compile the production reader against a fake NI-DAQmx API
and also exercise the no-driver stub. They verify setup cleanup, acquisition
failure/restart and sample-rate validation without connecting to NI hardware:

```bash
pixi run python tests/test_external_daq.py
```

## Architecture

```
gui/
  core/
    telemetry.hpp          # control-thread -> GUI snapshot types
    commands.hpp           # GUI -> control-thread command variant
    controller_worker.*    # owns the bus; runs the 1 kHz loop on its own thread
    no_load_dataset.*      # typed CSV sources, serialization and atomic saves
  widgets/
    connection_panel.*     # connect/disconnect + joint selection
    jog_panel.*            # press-and-hold jog + homing
    limits_panel.*         # backdrive capture + explicit limits
    trajectory_panel.*     # waveform picker + play/stop
    plot_panel.*           # custom-painted reference/actual/error strip charts
    locked_rotor_test_panel.*    # current sweeps and optional PSU acquisition
    damped_current_settle_panel.* # preconditioning and torsiometer tare
    no_load_test_panel.*   # optional speed hold and independent acquisition
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

The PSU reader exposes a coherent typed progress snapshot with a display
message and a retained acquisition-start marker. Dialogs use its state for
completion/error decisions, never the wording of the display message.
No-load CSV serialization consumes a read-only dataset view outside the dialog,
preserving separate sensor, drive and supply clocks and reporting save errors
without discarding the in-memory data.
NI tasks use scoped ownership so failed setup and reader shutdown share the
same cleanup path and setup has a single error boundary. Filter and encoder
window state are local to the acquisition worker, using the actual negotiated
sample rates and resetting naturally on each restart.
