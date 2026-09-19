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

Manual Drive can start without camera or perception/planning nodes. Autonomous
driving still requires its pipeline dependencies. Both modes require the Safety
Supervisor, all four ZCUs reachable, and E-Stop reset.
Manual Drive and Motion Planner are mutually exclusive.

### Xbox Controller

Install `sudo apt install python3-pygame` (also declared as a ROS package
dependency). Connect/pair the Xbox controller to the HPC using USB or Bluetooth,
then select **Driving Control → Manual input → Xbox Controller**. The connection
status shows the detected device. Start Manual Drive using the usual GUI button.
Release all controls once after starting. No RB hold is required.
While waiting for the controller or GUI focus at startup, the node remains
running with neutral commands instead of immediately latching E-Stop. After a
focused, neutral controller sample has been accepted, connection/focus loss
continues to latch E-Stop. B can stop even before neutral input is accepted.

| Control | Action |
| --- | --- |
| RT | Pressure-proportional forward acceleration; brakes first if reversing |
| LT | Pressure-proportional braking; keep holding after stopping to reverse |
| Left stick left / right | Position steering; release to return to center |
| D-Pad left / right | Adjust steering like A / D; release to hold the angle |
| X | Center steering (same as R) |
| B | Latched emergency stop during Manual Drive |
| Y | Reset E-Stop; restart Manual Drive after an Xbox-mode emergency stop |

Trigger pressure scales acceleration/braking rate, not a target speed. A held
trigger can eventually reach the configured maximum speed. Direction changes
first reach zero, then accelerate in the new direction on the next control tick.
Pressing both triggers only brakes toward zero and never changes direction.
Releasing both triggers uses the existing deceleration rate.

Stick position sets a target within `max_steering`; movement and automatic
centering are limited by `steering_rate`. D-Pad uses the same rate and limits as
keyboard steering. After D-Pad release the angle stays held until the stick moves
outside its deadzone or X is pressed. Stick input takes priority if used together
with D-Pad. D-Pad up/down are unassigned.

Stick deadzone is 20%, trigger deadzone is 10% (`GamepadGate` defaults). The
remaining travel is rescaled continuously to the full 0–100% input range.
SDL's [controller mapping](https://www.pygame.org/docs/ref/sdl2_controller.html)
handles the different USB/Bluetooth axis layouts. The first SDL-recognized Xbox
controller is selected; connect only the intended driving controller.

Losing the controller connection, losing GUI focus,
opening a modal dialog, or changing input mode during manual driving latches
E-Stop. If Manual Drive was active in Xbox mode when E-Stop was triggered,
pressing Y (or GUI Reset E-Stop) automatically restarts that node after a
successful reset. Release Y and all driving controls: restart waits for neutral
input, GUI focus, node termination, completion of fallback stop commands, and
the supervisor's cleared E-Stop status. Normal start checks still apply,
including supervisor and ZCU connectivity checks. Failed reset requests do
not restart the node. Y sends one reset request per press and requires no active
control source. B takes priority over Y.

Keyboard mode, autonomous mode, and an ordinary Stop do not enable automatic
restart. Changing input mode or explicitly stopping a control-chain node cancels
a pending restart. Reconnection alone never restarts driving: a successful
E-Stop reset is required.
Keyboard/on-screen driving inputs are ignored in Xbox mode; keyboard Space and
the GUI E-Stop remain available. Autonomous driving is unaffected by controller
disconnects. If no controller appears, check OS pairing and the logged-in user's
read permissions for `/dev/input/event*`.

The analog extension changes the ROS `DriveInput` message. Rebuild
`interfaces_pkg`, `data_collection_pkg`, and `autosdv_gui_pkg` together using the
build instructions above, then restart the GUI and Manual Drive so all publishers
and subscribers use the new message definition. Legacy keyboard fields retain
their behavior when `analog` is false.

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

### Runtime camera inputs

Open **Camera inputs…** in the main window. Opening the window only queries the
five senders. Use **Camera** / **Video** per row, or **All → Camera** / **All →
Video**. Each row has a video path on that Raspberry Pi, initially
`/home/pi/camera/driving_simulation.mp4`. The actual source is queried after each
switch, and failures remain visible alongside the last confirmed source. A failed
status query displays Unknown. **Refresh all** reads the current state again.

Image 01 is the ZCU1 webcam (`12.0.0.5`, port 8586); images 02–05 are the ZCU1–4
cameras (`11.0.0.5`, port 8585). The GUI uses nonblocking subprocesses through
`autosdv_gui_pkg.camera_source`; it does not restart ROS or ZCU nodes. Camera
processes switch their capture input and keep the existing UDP image format and
frame counter. Switching may briefly pause images; all-camera switching is not
frame-synchronized. Runtime selection is not saved across camera service restarts.

Requirements: the runtime-capable `cam.py` / `webcam.py` and `camera_control.py`
from `Rpi_CameraModule` must be deployed under `/home/pi/camera`. The HPC account
uses `~/.ssh/id_ed25519_rpi`, SSH jump hosts `root@10.0.0.2`–`.5`, and trusted
HostKeyAlias entries `rpi-zcu1-webcam`, `rpi-zcu1`–`rpi-zcu4`. Control UDP sockets
bind to `127.0.0.1`; the command is sent locally on the Pi over authenticated SSH.
No routes to the overlapping Pi subnets are needed. Video files must already
exist on each target. Initial sources remain defined by the camera systemd units.

For scripts, the same controller can be used without starting the GUI:

```bash
python3 -m autosdv_gui_pkg.camera_source camera_01 status
python3 -m autosdv_gui_pkg.camera_source camera_01 video /home/pi/camera/driving_simulation.mp4
python3 -m autosdv_gui_pkg.camera_source camera_01 camera
```

The camera control window leaves W/A/S/D/R and Space available to its own input
widgets; typing video paths does not issue driving commands.

## 통신 모드: Linux Pico

Communication mode에서 **Zenoh–DDS Bridge + Pico**를 선택해 Check → Apply하면
HPC와 ZCU 4대가 `zenoh_bridge_pico` 모드로 전환된다. GUI·Manual Drive 등 HPC 노드는
CycloneDDS를 사용하고, ZCU는 Pico를 통해 HPC Bridge와 통신한다.
빌드·필수 라이브러리 패치 및 검증 범위는 [Pico 운영 문서](../../docs/zenoh-pico-linux.md)에 있다.
