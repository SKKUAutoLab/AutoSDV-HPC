# AutoSDV HPC Control GUI

PyQt5 GUI for starting and stopping the HPC ROS 2 nodes, manual driving,
command monitoring, ZCU diagnostics, and fail-safe command arbitration.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces_pkg ivn_diagnostics_pkg \
  data_collection_pkg autosdv_gui_pkg
source install/local_setup.bash
ros2 run autosdv_gui_pkg autosdv_gui
```

Run the GUI from the repository root. The process manager also resolves the
workspace root from the installed package so that the existing
`best_urp.pt` model can be found by the YOLO node.

## Driving controls

- `W`: forward acceleration
- `S`: reverse acceleration
- `A` / `D`: left/right steering
- `R`: center steering without resetting speed
- `Space`: latched emergency stop

The vehicle can only be armed when Camera 01 and all requested pipeline
dependencies are running, all four ZCUs are reachable, and E-Stop is reset.
Manual Drive and Motion Planner are mutually exclusive.

## Single video preview

The `Preview` checkboxes are mutually exclusive across Camera 01-05, YOLO,
Lane Extractor, and Path Planner. The GUI subscribes only to the selected image
topic and scales the latest frame into the embedded preview while preserving
its aspect ratio. Motion Planner and Manual Drive have disabled Preview boxes
because they do not publish image topics. YOLO and Path Planner start their
preview-renderer companion processes only when needed.

Preview is refreshed by a coalesced Qt notification when a ROS frame arrives.
To avoid the high CPU cost of deserializing raw images in Python, a managed C++
bridge subscribes to the selected raw topic and publishes a JPEG
`CompressedImage` only while Preview is selected. Qt decodes that compact
message directly, and frames are painted without intermediate `QPixmap` or
scaled-frame copies. The ROS bridge uses a dedicated `SingleThreadedExecutor`;
the Python `MultiThreadedExecutor` is avoided because it busy-spins under the
30 FPS image workload on the target HPC.
For nodes that provide both display modes, `Window`
and `Preview` are mutually exclusive: selecting either one automatically
clears the other.

Only one GUI and one safety supervisor may run at a time. On startup the GUI
stops supervisor processes left behind by an earlier GUI crash. The supervisor
also watches its GUI owner PID and exits with zero commands if that owner
disappears. If ROS discovery still reports multiple safety-status publishers,
the GUI shows `Supervisor: DUPLICATE` and blocks both control modes.

## Safety limitation

The GUI and `safety_supervisor_node` publish stop commands when the GUI,
selected controller process, or any ZCU connection fails. The supervisor does
not use GUI-heartbeat or controller-command timeouts, so a temporary pause does
not latch E-Stop. A complete fail-safe for HPC power loss or supervisor process
loss still requires the planned command-timeout watchdog in the ZCU/ECU.

## ZCU management

The GUI connects to `root@10.0.0.2` through `root@10.0.0.5` using SSH
BatchMode. Register each host key before use and verify that key-based,
passwordless login works. The managed service is `zcu.service`.

## IVN diagnostics

The GUI starts `IVN_diagnostics_node` as a separately managed ROS 2 node.
Its automatic monitor publishes `/ivn/diagnostics/status` once per second and
combines ZCU reachability, DDS publisher discovery, and the Zone-specific
reports received on `/ivn/zcu_health`. Health timeout age is measured from the
HPC receipt time, so it does not depend on synchronized ZCU clocks. The
automatic monitor intentionally does not subscribe to high-bandwidth camera
payloads.

`Run Manual Deep Check` sends a `/ivn/diagnostics/run_manual` action goal. This
on-demand check performs three-packet network probes and waits for one actual
payload from every camera topic. It also requires a new health heartbeat from
each of `zone_1` through `zone_4` after the Action starts. Missing data can make
the manual check take about 30 seconds. The automatic monitor continues to
publish while the manual action is running. The obsolete steering-feedback
topic `/topic_status_signal` is not part of either diagnostic path.
