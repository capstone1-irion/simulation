# Load-coupled normal motor / battery reference model

## Current CMU service versus the original reference

The active operator entry point is `tools/arion_sim.py`; use the standalone
[CMU service guide](cmu-service-guide.md). It generates `custom_x500_cmu` from
`profiles/battery/cmu_runtime/catalog.json`, disables command fallback, includes
the vehicle in the run's initial `world.sdf`, and connects PX4 to that existing
vehicle with `PX4_GZ_MODEL_NAME`.

Runtime observations below are **historical records**, not new verification.
The approved purge does not retain their raw flight logs or validation outputs
locally; those files cannot now be used for evidence readback. Current runtime
sources, the CMU catalog/CSVs/provenance and active build dependencies remain.

- The four measured same-cell RPT stages have relative capacities
  **100.000000 / 95.027445 / 90.422638 / 84.812594%**. The virtual **4S2P** pack is
  an assumption, not a measured x500 pack. Cell OCV is a discharge-derived
  **equivalent approximation**, and resistance is an **effective DC 1-second
  step estimate**. The approximately 95% stage's estimated resistance remains
  slightly below the initial value; no monotonic aging trend is imposed.
- The active catalog supplies each stage's capacity, resistance, OCV curve and
  initial SOC. It does **not** use the original **5 Ah / 0.04 Ω / linear OCV**
  defaults below. Those defaults and the original normal-flight findings remain
  documented as a legacy reference, not a retained standalone flight workflow.
- Current motor-fault controls scale modeled current and reported RPM. The
  changed current enters the battery load; propulsion forces, joint motion and
  flight-control outputs are not directly changed. This is still one-way
  telemetry/electrical-load coupling, not a physical motor-failure model or a
  continuously aging electrochemical model.
- `clear` and battery `reset` do not reboot PX4. Ground-only `world-reset` stops
  the sender and PX4, resets Gazebo's world/time, and starts fresh PX4/sender PIDs
  while retaining Gazebo, the collector, service and run directory. New required
  message counts and Disarmed state are checked. `px4_epoch` and ground-truth
  boundaries distinguish clock generations; a short telemetry gap is expected.
  Ground reset/recovery and two subsequent target-3 m flights in PX4 epoch 1
  (100% and 84.812594% profiles, each with motor 1/4 fault/clear and landing/auto
  disarm) have been observed separately. These are not 24/7 endurance evidence;
  final CLI stop and cleanup were also confirmed separately: all owned current
  and prior-epoch processes exited 0, with no forced kill or surviving groups,
  and owned IPC/control sockets were removed for that run. This is not a live
  service-status check.
- The initial six-profile bundle and replay tooling are removed from the
  retained tree. NASA is retired from active inputs; it is not an alternate
  runtime source. CMU regeneration and real-source publication tests require
  reacquiring the pinned originals; current service execution uses the retained
  catalog without those originals.

See [CMU derivation and limits](cmu-runtime-profiles.md),
[catalog](../profiles/battery/cmu_runtime/catalog.json),
[provenance](../profiles/battery/cmu_runtime/provenance.json), and
[NASA retirement](nasa-retirement.md). Source attribution is in
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

**The following sections preserve the original normal-reference design and its
historical verification scope.** Statements about no CMU data or no fault
injection apply to that original path, not to the current CMU service above.

## Original scope and integration (legacy reference)

`ArionCoupledPowerPlugin` / `arion::CoupledPowerState` is a Gazebo Harmonic
(`gz-sim8`) **telemetry system**, not a replacement propulsion system. It observes
four simulated rotors, estimates their electrical load, and feeds that load into
one reference battery. Existing flight-control commands, motor physics, joint
velocity commands and forces are never modified. The only requested ECM state is
the joint-velocity observation component.

The fixed motor plugin, fixed `custom_x500` model, motor bridge and battery
sender remain separate; the fixed battery plugin/model have been removed.
**Do not attach the coupled plugin and a fixed
state publisher to the same model at once**: their topic/schema contracts are
intentionally identical. A model which merge-includes stock `model://x500` and
attaches only this extra plugin uses the defaults. The now-removed
`custom_x500_power` additionally disabled command fallback, so its historical
flight observations required actual simulated joint observations. The current
service generates `custom_x500_cmu` and does not need that old model.

This implementation is an uncalibrated simulation reference, not measured drone
telemetry, a calibrated battery pack, degradation/SOH, or evidence of real-drone
AI accuracy. No NASA/CMU cell data or inferred hardware calibration is used.

## Rotor-state provenance and motor current

- Motor slots 0–3 correspond to `rotor_0_joint` … `rotor_3_joint`, matching stock
  x500 motor numbers. `PreUpdate` requests `Joint::EnableVelocityCheck(ecm)`;
  `PostUpdate` reads the resulting `components::JointVelocity` after physics.
- The primary RPM is `abs(joint_rad_s) * 10 * 60 / (2*pi)`. The factor 10 restores
  the stock motor model's `rotorVelocitySlowdownSim=10`. Opposite rotor directions
  do not create negative RPM/current. This is **simulated joint feedback**, not a
  hardware RPM measurement and not `JointVelocityCmd` or PX4 command echo.
- If a joint/component is absent or its velocity vector is empty, optional fallback
  uses the observed `/<model>/command/motor_speed` `gz.msgs.Actuators` input. It
  performs a first-order modeled response, never copies a command into a measured
  field: `rpm += (target_rpm-rpm) * (1-exp(-dt/tau))`, with `tau=0.0125 s` on
  acceleration and `0.025 s` on deceleration. The target is nonnegative rotor
  rad/s capped at 1000, converted to RPM **without** the slowdown factor.
  These constants match the inspected x500 SDF, not fitted motor measurements.
- Joint feedback always takes precedence, independently per motor. Transition to
  fallback starts from the last observed/modeled RPM. Fallback requires a valid
  four-element command received within 0.5 **simulation** seconds. This is arrival
  freshness, not independent measurement or a guarantee about a queued command's
  original generation time. The callback captures only a shared mutex/mailbox,
  not the plugin object. Reset clears the cached command and response state.
- Disable fallback for strict joint-only observations with
  `<allow_command_fallback>false</allow_command_fallback>`. A present but malformed,
  nonfinite or out-of-envelope joint observation is rejected, not silently replaced
  with a command estimate. The envelope is `abs(joint_rad_s)<=200`, allowing twice
  the stock 1000 rad/s unslowed command limit.

Motor current uses the 15 RPM→ampere rows in the user-provided
`X500_모터_사양_확인.md`, section **16 V 벤치 시험 표**: AIR2216II-KV920 + T1045II,
4042 RPM/1.44 A through 9857 RPM/16.37 A. RPM is interpolated linearly between
adjacent rows. Throttle percentage is not used as a proxy for measured RPM.

| Region | Rule | Evidence status |
|---|---|---|
| 0 ≤ RPM < 4042 | `I = 1.44*(RPM/4042)^2` | Explicit assumed low-speed continuation; zero at rest, no separately modeled ESC idle current |
| 4042 ≤ RPM ≤ 9857 | Piecewise linear interpolation of the supplied table | Reference to that bench/propeller/16 V condition, not calibration of this Gazebo airframe |
| RPM > 9857 | Cap current at 16.37 A; retain the observed RPM | Explicit safety bound, **not** high-speed extrapolation or validated performance |

The 0.8 A no-load specification at **10 V** is not reused as a 16 V idle value.
The motor's 115 mΩ resistance is not the battery's resistance.

## Legacy battery defaults and one-way semantics

| Quantity | Legacy default / model (not CMU runtime) |
|---|---|
| Pack | 4 balanced series cells, one battery with ID 0 |
| Nominal/reference capacity | 5 Ah, assumed; not established by the motor table |
| Pack internal resistance | 0.04 Ω, assumed for the **whole pack**, not per cell |
| Initial SOC | 0.95 |
| Auxiliary/avionics current | 0.5 A |
| Battery current | `I_pack = I_motor0 + I_motor1 + I_motor2 + I_motor3 + I_aux` |
| Consumed charge | `consumed_mAh += I_pack * dt_s / 3.6`, starting at zero for the run |
| Remaining SOC | `initial_SOC - consumed_mAh/(capacity_Ah*1000)`, bounded to [0,1] |
| Cell open-circuit voltage | `3.3 + 0.9*SOC` V — deliberately simple, **uncalibrated linear reference** |
| Pack terminal voltage | `4*cell_OCV - I_pack*R_pack` |
| Individual cell voltage | Pack terminal voltage / 4; no imbalance model |
| Temperature | Constant 25 °C placeholder required by the existing finite-field contract; **not thermal simulation or measured temperature** |

Consumed charge saturates at the initial available charge (not full nameplate
capacity when starting below 100% SOC). At zero SOC the model still reports the
requested modeled load; it does not cut thrust or represent battery failure.
A 0.004 V pack / 0.001 V per-cell lower floor exists only to keep the existing
wire contract encodable at extreme configurations. It is not a physical voltage
cutoff; the `voltage_floor_active` diagnostic marks it. Such saturated/empty
segments are outside the intended normal-flight reference range.

**The motor-state voltage fields report this same sagged pack terminal voltage**,
not fixed 16 V. Motor current is nevertheless still the table's **16 V-equivalent
current estimate**, and neither joint RPM nor command response depends on battery
voltage. This is intentionally **one-way motor load → battery** coupling, not a
self-consistent electrical feedback/power-conservation model. There is no motor
wiring drop, ESC efficiency map, transient winding current, thermal behavior,
wind/payload-specific current calibration, aging or fault injection. Exclude the
constant temperature and simulator-only diagnostic/ground-truth metadata from
claims of measured AI features.

## Time, invalid data and reset

The pure `CoupledPowerModel::update(now_ns, dt_s, paused, input)` uses simulation
time only. Valid updates use `min(Gazebo dt, elapsed simulation time)`; the first
observation establishes the origin without retroactive discharge. No wall clock
is read. Paused or duplicate-time updates do not advance RPM/charge or publish.
A paused rewind still restores initial state. Rewind and `ISystemReset` restore
configured initial SOC, zero consumed charge, zero fallback RPM, clear cached
commands, reacquire joint observations and reset both publication schedules.

Nonfinite/negative non-rewind dt, negative timestamps, invalid required commands,
invalid joint observations or lack of all four valid rotor sources suppress both
state messages. The last numerical state is retained with `sample.valid=false`;
unknown input is not presented as zero motor load. Missing intervals are not
backfilled: battery charge holds over those intervals, so investigate diagnostic
warnings/gaps rather than treating discontinuous input as complete discharge
accounting. A negative dt on a rewind cannot discharge backward; after an explicit
Gazebo reset the adapter may skip that one negative-dt update.

## Exact transport contract

Both topics use `gz.msgs.Float_V`, with `header.stamp` from simulation time and
exactly one `header.data` entry whose key is `schema`.

| Topic | Schema | Ordered float data | Nominal simulation-time rate |
|---|---|---|---|
| `/arion/<model>/motor_state` | `arion.motor_state.v1` | RPM[0..3], voltage_V[0..3], current_A[0..3] — **12 floats** | 20 Hz |
| `/arion/<model>/battery_state` | `arion.battery_state.v1` | ID=0, cell_V[0..3], pack_A, SOC fraction, temperature_°C, consumed_mAh — **9 floats** | 5 Hz |

Rates are scheduled on physics updates, with no missed-frame catch-up burst;
coarse/non-divisor timesteps can lower the effective rate. Independent schedules
mean a battery frame need not have a motor frame at exactly the same stamp; at
every model step the current-sum relationship holds before float quantization.
The existing `MotorFrame.hpp`/`BatteryFrame.hpp` parsers are unchanged and exercised
by the pure tests.

Additional diagnostic header keys do not change numeric layouts or the parsers:
`power_model`, `current_model`, `motor_voltage_source`, `temperature_source`,
`voltage_floor_active`, `charge_exhausted`, plus four ordered values each for
`rpm_source` (`joint_velocity` / `modeled_command_first_order`) and
`current_curve_region` (`assumed_low_rpm` / `bench_interpolation_16V` /
`capped_above_bench`). Preserve these Gazebo diagnostics during validation; they
are not new fields in standard MAVLink ESC_STATUS or BATTERY_STATUS.

## Build target and SDF configuration

Required source: `plugins/power/CoupledPowerState.cpp`; include directory:
`include`; C++17. Target name **`ArionCoupledPowerPlugin`**, shared-library output
**`libArionCoupledPowerPlugin.so`**. Link the already-used targets
`gz-sim8::gz-sim8`, `gz-transport13::core`, `gz-plugin2::register` (and normal platform
thread support). Registered class **`arion::CoupledPowerState`** implements
Configure, PreUpdate, PostUpdate and Reset. The pure-test executable is
`coupled_power_model_test` from `tests/coupled_power_model_test.cpp`, with no
Gazebo dependencies.

All tags below are optional in the legacy path. Without a profile catalog, the
plugin selects the legacy defaults; the CMU service instead supplies
`battery_profiles/profile` entries and an `initial_profile`. The following XML
is **not a CMU configuration**. The removed initial flight model explicitly
set `allow_command_fallback=false`.
Attach this inside the model, retaining all stock x500
propulsion plugins and omitting fixed Arion state publishers:

```xml
<plugin filename="ArionCoupledPowerPlugin" name="arion::CoupledPowerState">
  <capacity_ah>5.0</capacity_ah>
  <pack_resistance_ohm>0.04</pack_resistance_ohm>
  <initial_soc>0.95</initial_soc>
  <auxiliary_current_a>0.5</auxiliary_current_a>
  <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
  <allow_command_fallback>true</allow_command_fallback>
</plugin>
```

Validation bounds: capacity [0.01,1000] Ah, whole-pack resistance [0,1] Ω, initial
SOC [0,1], auxiliary current [0,100] A; all finite. The slowdown declaration must
be exactly 10 and must match the stock physics plugins. Other factors/models need
an intentional model extension, not a changed metadata label. Malformed/out-of-
range configuration disables the plugin with an error; no silent default on a
present invalid tag. Fallback's time constants and 1000 rad/s limit are fixed to
the inspected stock x500 configuration.

## Original focused implementation evidence (historical)

The implementation worker wrote only these four new repository paths:

- `include/arion/CoupledPowerModel.hpp`
- `plugins/power/CoupledPowerState.cpp`
- `tests/coupled_power_model_test.cpp`
- `docs/coupled-power-model.md`

Historical output location: `../hermes/coupled-flight/model/` relative to the
simulation repo. The logs, helper scripts and evidence JSON listed below are
not retained after the purge; this list describes the original recorded scope.

- `01-joints-red.log` … `12-clocks-green.log`: six small RED→GREEN increments.
  Missing-feature compile failures and behavioral failures were recorded as such,
  not represented as successful execution. Final native C++17 pure tests: six
  named cases passed (joint response/current table, modeled fallback response,
  current sum/SOC/sag/existing wire contracts, sim-time/pause/rewind, input/config
  validity and bounded reset, 20 Hz/5 Hz clocks).
- `13-plugin-build.log`: first strict build failed because Gazebo Transport 13
  `Publisher::Publish` is non-const. The publisher reference was corrected.
- `14-plugin-build-fixed.log`: strict native shared-library build succeeded;
  `ldd -r` exited 0 with no unresolved symbols. Built against installed Gazebo
  Sim 8.15.0 / Transport 13.6.0 / Msgs 10.4.0.
- `run_focused.py`, `build_plugin.py`: recorded bounded compile/test commands and
  per-stage source hashes; `final-evidence.json` recorded final source/binary hashes.

No Gazebo/PX4 runtime, packet-route verification, whole-repository test suite,
independent review or flight was run by this worker. CMake and the former opt-in
flight model were separate integration changes. The worker's build-only result
did not establish runtime loading, JointVelocity provenance, flight, PX4
re-streaming or cleanup; the separate historical integration is described below.

API checks used installed `gz/sim8/gz/sim/Joint.hh`, `System.hh`,
`components/JointVelocity.hh`, `gz/transport13/gz/transport/Node.hh`, and official
Gazebo `gz-sim8_8.15.0` `MulticopterMotorModel.cc` (the native motor model restores
real rotor speed by multiplying joint velocity by its slowdown factor). This
supports the API/unit choice; it does not replace runtime observation.

## Original normal-reference integration result

On 2026-10-04 KST, the parent built the opt-in CMake targets and ran the separate
`custom_x500_power` model with command fallback disabled. One actual local SITL
takeoff, 8-second hover near 3 m, landing and automatic disarm passed. The parent
matched hover producer data to PX4 uORB snapshots and autopilot-origin MAVLink.
This supplements (does not rewrite) the worker's build-only scope above.
Historical output paths (not retained): project
`hermes/coupled-flight/runs/flight-zgit9bn0/` and
`hermes/coupled-flight/runtime-readback.json`. The standalone harness, initial
model and its run/data-contract guides are removed from the retained tree.
At that original stage, live world reset
and repeat-flight testing remained unperformed; pure-model reset tests were not
those runtime tests. Later CMU service reset and flight observations are listed
at the top of this document and in the [service guide](cmu-service-guide.md);
they do not rewrite the scope of the earlier reference-model run.
