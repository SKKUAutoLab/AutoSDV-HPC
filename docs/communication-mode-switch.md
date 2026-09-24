# Common mode switching and GUI

The common controller supports `cyclonedds`, `zenoh`, `zenoh_bridge`, and
`zenoh_bridge_pico`. It does not build ZCU backends. Both `zcu` and
`zcu_health_node` must exist in `build-<mode>` on every board. The Pico readiness
check requires both binaries to dynamically link libzenohpico; static builds need
an explicit build identity check before being accepted.

As of 2026-09-10, `zenoh_bridge` is built and deployed on all four ZCUs.
See [Bridge implementation and hardware validation](zenoh-bridge-validation-2026-09-10.md).
`zenoh_bridge_pico` now has a Linux backend and a pinned, patched Pico installer.
See [Pico build and operating instructions](zenoh-pico-linux.md).

## Installation

Build the GUI and launch package on HPC:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select launch_pkg autosdv_gui_pkg --symlink-install
source install/local_setup.bash
sudo python3 scripts/install_communication_mode.py hpc --workspace "$PWD"
```

Install the same two standalone Python files on each ZCU (Python 3, systemd,
tc and either ldd or the AArch64 glibc loader are required):

```sh
scp scripts/communication_host.py scripts/install_communication_mode.py root@10.0.0.2:/tmp/
ssh root@10.0.0.2 'python3 /tmp/install_communication_mode.py zcu --workspace /home/root/Workspace/S32G3_ZCU'
```

Repeat for 10.0.0.3, .4 and .5. SSH host keys must already be trusted and
key-based root login must work. The installer does not start or stop vehicle
processes. It installs `/usr/local/libexec/autosdv-communication-host`, a root-owned
configuration, and `autosdv-communication.service`. ZCU drop-ins order gateway
and health after the communication policy. Services are enabled at successful
commit; before the first selection the ZCU boot hook preserves its legacy policy.

The CLI/GUI invokes the local helper with `sudo -n`: configure the operator's
sudo policy for that installed helper, or use an already applicable policy.
No password is collected or stored in the GUI. Missing permission is a preflight
error. Do not run the GUI as root. This implementation does not install sudoers rules.

## Use

```sh
scripts/select_communication_mode.sh check cyclonedds
scripts/select_communication_mode.sh apply cyclonedds
scripts/select_communication_mode.sh status
```

Close the GUI before CLI `apply`; if the GUI is running, use its **Communication
mode…** button instead. The GUI dialog lists all four modes. **Check devices**
checks all five hosts without changing their mode. **Apply and restart GUI** is
enabled only after the selected mode passes. Changing selection invalidates
that result; Apply also repeats preflight to detect changes since Check.

The GUI stops driving, its managed nodes, timers, and ROS context before applying.
On success it restarts as a new process with the persisted RMW. Previously running
camera/perception nodes are restored; drive and motion planner require explicit
restart and activation. Normal GUI launch starts the existing supervisor and
diagnostics as before. CLI applies communication infrastructure and stops registered
HPC nodes; start the GUI/launch files afterward to start application nodes.

Progress and errors are shown per host. A failed preflight leaves host configuration
unchanged. If failure occurs after Apply has begun, reachable hosts are marked
failed and stopped; failures to stop unreachable hosts are reported. No automatic
rollback or driving restart occurs. Retry via the same dialog; after exiting,
the next GUI launch opens recovery before creating a ROS context.

## State and boot

`/var/lib/autosdv/communication-mode.json` records `applying`, `failed`, or `ready`.
The HPC record is committed last after all ZCUs commit. A system-wide flock
serializes apply operations. `/run/autosdv-communication-mode` holds the staged
mode; it disappears at boot. Interrupted/failed durable state blocks boot restore
until a new coordinated apply succeeds.

After successful selection, the systemd service restores the communication mode
on boot. HPC GUI and launch profiles read the persisted mode; conflicting
`AUTOSDV_RMW` overrides are rejected. Bridge and Pico share HPC CycloneDDS loopback
profiles. Wired CycloneDDS profiles explicitly disable multicast and include the HPC
address as a discovery peer, so both ZCU traffic and local GUI/processing
traffic use unicast. Generic HPC nodes use automatic participant indices 0–19;
camera nodes keep indices 20–24. This first controller fixes IVN addresses to 10.0.0.10 and .2–.5,
ROS domain to 0, ZCU UDP camera endpoints to 17451/17453/17455/17457/17459,
and the control endpoint to 7448, matching the current builds. It is not a
multi-domain or alternative-topology selector.

HPC owns nftables through the communication service. Legacy autosdv-dscp-qos
is stopped during switching. ZCU retires legacy camera DSCP units, removes only
their pref 10 and the new reserved prefs 101–105, and installs per-flow CS5 rules.
Other tc filters are preserved. An unmanaged Zenoh UDP listener on 7447/7448 must
be stopped before the controller can take ownership.

## Meaning of success and validation limits

`ready` means the requested binaries/services were applied: each ZCU's running
executable hashes match the selected build, services are active, and required
HPC UDP listeners are present. `data_path_verified` remains **false**. It does not
claim that camera data, control/CAN, health payloads or DSCP packets were verified.
Those are subsequent per-mode integration gates.

On 2026-09-08, all four ZCUs were reachable and the adapters were installed.
Physical coordinated transitions succeeded in this order: FastDDS → CycloneDDS
→ Zenoh → CycloneDDS. All five hosts committed each selected mode. Both new
modes delivered five camera streams at about 30 FPS (640×480 JPEG, decoding
confirmed) and health messages from all four zones. Bridge/Pico Apply requests
were rejected in preflight because the ZCU builds are absent.

Two deployment issues were corrected: the minimal ZCU image omits `ldd`, so the
adapter falls back to the AArch64 glibc loader; the HPC's actual IVN interface is
`enP2p1s0`, so the adapter now resolves the interface owning `10.0.0.10` and passes
it explicitly to the QoS service, also checking its presence before switching.
The old hardcoded `enp4s0` rule had matched no packets. Packet capture confirmed
ZCU camera CS5 for both modes and HPC Zenoh port 7448 CS6 after the correction.

All four ZCUs were rebooted and restored CycloneDDS automatically, with changed
boot IDs and running binary hashes matching their CycloneDDS builds. HPC restored
its committed mode after stopping the service, removing only its volatile runtime
selection and starting the service again; HPC itself was not rebooted. The GUI
profile loader selected `rmw_cyclonedds_cpp` from the persisted state. Desktop
button interaction and GUI process restart remain untested; offscreen Qt checks
against real boards were tested in the preceding session.

The final persisted mode is CycloneDDS on all five hosts. The GUI remains closed.
No probe published control commands. CAN RX was unobserved in all four zones,
as in the original FastDDS baseline; control/CAN delivery, operating limits,
long-duration resource measurements and complete four-mode acceptance remain
pending. `data_path_verified` therefore remains false. The initial missing ROS type-hash warnings were resolved in a follow-up by
adding type-hash USER_DATA to the native CycloneDDS endpoints; the physical
validation report records before/after checks. The legacy ROS_LOCALHOST_ONLY
value is now cleared to avoid rcl deprecation warnings. Camera service environment settings were
preserved; actual received encoding was JPEG even though the existing service
sets `AUTOSDV_CAMERA_ENCODING=h264`.

Fresh verification after the fixes: 44 focused tests passed, including two
interface selection tests. GUI profile unit tests now isolate the persisted mode
file so they do not depend on the vehicle's current selection. The launch package
CTest target also passed with the ROS and workspace setup sourced. Both ROS
packages were built in the earlier implementation session; these follow-up changes
are Python adapter/test changes. Detailed measurements are recorded in
[the physical validation report](communication-validation-2026-09-08.md).

To rerun focused tests:

```sh
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/launch_pkg/test/test_communication_mode.py \
  src/autosdv_gui_pkg/test/test_mode_dialog.py \
  src/autosdv_gui_pkg/test/test_control_logic.py \
  network_qos/test_autosdv_dscp_qos.py
```
