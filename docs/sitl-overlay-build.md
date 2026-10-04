# Project-local PX4 SITL overlay

This builds a **new Linux SITL executable**, not real-flight firmware. The PX4
checkout, its existing dirty files, cached build products and Git index/HEAD are
read-only inputs. Nothing here starts PX4 or Gazebo or sends MAVLink packets.
The existing external `BATTERY_STATUS` receiver and standard MAVLink message
remain the route; this does not add a second battery bridge or custom wire units.

## Prerequisites and build

Use a completed, trusted local PX4 SITL build with `arion_motor_bridge` already
registered in its generated `apps.cpp`, module aliases and link dependencies.
The checkout must be at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
The cached `compile_commands.json` paths must still match that checkout and build
directory. This is an overlay on an existing build, not a fresh PX4 installer.
The build also retains the cached Gazebo ESC publisher-switch change from
`0001-gz-esc-status-source-switch.patch`; it does not apply that patch to PX4.

Python 3, the original C++ compiler/libraries, Ninja, GNU ar, patch, readelf and
ldd must already be installed. No download, CMake configuration or dependency
installation is performed. Example team-neutral workspace layout:

```sh
workspace="$HOME/px4-workspace"
python3 -B "$workspace/simulation/tools/build_sitl_overlay.py" \
  --repo "$workspace/simulation" \
  --px4-source "$workspace/simulation/PX4-Autopilot" \
  --base-build "$workspace/artifacts/motor-build/px4" \
  --output "$workspace/artifacts/coupled-sitl" \
  --ipc-dir "$workspace/ipc"
```

The output is `<output>/bin/px4`. Keep its `bin/px4-alias.sh`, module command
symlinks and `etc/` tree together. The output directory can contain separately
named evidence, but the builder refuses existing generated build directories or
`manifest.json`. Use a new output directory for a rebuild; previous evidence is
not overwritten. Do not move the executable to an existing build's `bin/`.

### What gets compiled and linked

- Copy and modify only POSIX `main.cpp`, daemon `sock_protocol.cpp`, MAVLink
  `mavlink_receiver.cpp` and `streams/BATTERY_STATUS.hpp` in the overlay directory.
- Copy `mavlink_messages.cpp` so its quoted stream-header include selects the
  corrected header. Other stream headers come from the original include path.
- Copy and **compile the current** `px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp`
  so the lifecycle fixes are not lost to its older cached archive.
- Reuse the original compile flags, generated headers and immutable dependency
  archives. Rewrite object outputs/dependency paths and `PX4_BINARY_DIR` for this
  overlay; preserve the original parent include directory for copied sources.
- Replace only the changed members of copied `px4_daemon`, `modules__mavlink` and
  `modules__arion_motor_bridge` archives. Relink without executing Ninja's build
  or post-link commands. `ninja -t commands bin/px4` is read-only discovery.
- Copy the matching startup tree and generated aliases, and make module symlinks
  point to this new executable. The old executable is never copied over it.

## Battery conversion contract and limits

The patch is a reviewable input to the helper, **not an instruction to apply it
to the original PX4 checkout**.

| Standard MAVLink ingress | SITL uORB result / re-stream behavior |
| --- | --- |
| Temperature in cdegC | Divide by 100 to degrees C; `INT16_MAX` becomes NaN. Output rounds back to cdegC or uses `INT16_MAX`. |
| Battery ID 0 | uORB ID 1 and `SOURCE_EXTERNAL`; output ID 0. |
| IDs 1–255 | Rejected, because the existing receiver has one single-instance publisher. No overflow or changing BAT1 identity. |
| Current in cA | Divide by 100; wire `-1` becomes the uORB `-1 A` unknown sentinel. Correct output maps that sentinel back to wire `-1`, not `-100`. Other negative currents remain negative. |
| Remaining percent | Valid 0–100 becomes 0–1. Unknown/out-of-range becomes uORB `-1`, output `-1`; unknown remaining does not trigger a false emergency warning. |
| Time remaining | Positive seconds retained; zero/negative becomes NaN, output zero (unsupported). |
| Cell voltages | Contiguous prefix of the first ten mV slots is summed as volts. Four-cell fixture stays four cells. |
| Total-only voltage | Single occupied slot or leading `UINT16_MAX-1` total-voltage chunk means unknown cell count 0, not a physical one-cell pack. A true single cell is indistinguishable on this wire representation and also gets count 0. |
| All voltage slots unknown | 0 V, cell count 0 and disconnected, rather than a connected zero-volt battery. |

Consumed charge remains mAh. No current average or actuation scale is invented;
these are initialized to their uORB unknown sentinels. The original system-ID /
non-autopilot-component filter and remaining-charge warning thresholds remain.

Limits are deliberate: one external battery, one authoritative sender; no new
sender allowlist, multi-battery registry or stale-data watchdog. Cells 11–14 are
not ingested by this pinned receiver. Energy, incoming charge-state/fault metadata
and battery chemistry are not newly mapped. Exactly `-1 A` charging current
cannot be distinguished from uORB's reserved unknown value. The corrected output
current/SOC sentinel handling applies to all battery instances in this SITL
binary; the real firmware and MAVLink schema are untouched.

## Runtime ownership and isolation

The builder creates the selected IPC **directory only**. PX4, when separately
launched, uses `<ipc-dir>/px4_lock-<instance>` and
`<ipc-dir>/px4-sock-<instance>`. Both server and module-command clients use the
same compiled prefixes. Paths under `/tmp` or too long for Linux UNIX sockets
are rejected. This helper never launches PX4, even for `--help` or a smoke test.
The runtime owner must keep the IPC directory private to the experiment and
clean up only its own stopped instance.

Before normal rcS startup, the runtime owner must set `BAT1_SOURCE=1` and
`SIM_BAT_ENABLE=0`. Changing the latter after the simulator has started is not
publisher isolation. Keep external ingress separate from the telemetry collector,
and accept only autopilot-origin output as proof of re-streaming, not forwarded
sender packets. The runtime owner supplies the custom model/plugin environment
and performs the actual fixed-battery and takeoff/hover/land verification.

## Focused checks and evidence

```sh
ARION_PX4_SOURCE="$workspace/simulation/PX4-Autopilot" \
ARION_OVERLAY_TEST_ROOT="$workspace/artifacts/overlay-unit-tests" \
TMPDIR="$workspace/artifacts/overlay-unit-tests" \
  python3 -B "$workspace/simulation/tests/test_sitl_overlay.py" -v
```

These are standalone C++ conversion seams extracted from the actual patched
handler/stream expressions, plus build-command, IPC and packaging contracts.
They run with undefined-behavior/float-cast sanitizers and never start PX4 or
Gazebo. They are not wire transport, uORB runtime or flight evidence. The test
root is mandatory so temporary files remain under an explicit artifact path.

The build creates:

- `manifest.json`: exact commands/exit codes, source/object and ELF hashes,
  runtime-layout hashes, IPC prefixes, and protected-input preservation result.
- `protected-before.json`: hashes/symlink identities for tracked PX4 sources and
  submodules, all existing cached-build files, relevant Git state and bridge.
- `logs/`: per-step compiler, archiver, linker, ELF-header and dependency logs.
- `link.map` and `link-replacements.json`: exact baseline-to-overlay substitutions.
- `overlay/`, `obj/`, `archives/`, `bin/`, `etc/`: the isolated artifact.

A successful manifest means compile/link, ELF/dependency/layout inspection and
protected-byte checks passed. Its `runtime_verified` remains false: runtime and
flight acceptance belong to the separate experiment, not this builder.
