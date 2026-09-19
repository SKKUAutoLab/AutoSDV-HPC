# Runtime camera source control — 2026-09-09

Implemented the Camera inputs dialog in autosdv_gui_pkg and deployed runtime
senders plus camera_control.py to all five Raspberry Pis. The GUI package was
built with colcon --symlink-install. Existing source changes were preserved;
no ZCU image receiver or camera systemd startup configuration was changed.

## Verification

- 69 tests passed: GUI package tests plus both runtime sender tests.
- Python compilation and git diff whitespace checks passed.
- The offscreen Qt dialog queried all five real senders over SSH successfully.
  UI tests checked asynchronous event processing and mixed per-camera outcomes.
- Image 01 (ZCU1 webcam): camera → video → camera passed.
- Image 02 (ZCU1 camera): video → camera → video passed.
- Image 03 (ZCU2 camera): physical camera initialization failed; device camera
  enumeration reported `No cameras available!`. Existing video was preserved.
- Image 04 (ZCU3 camera): same physical camera detection failure; video preserved.
- Image 05 (ZCU4 camera): video → camera → video passed.
- All five rejected a nonexistent video path while preserving their active input.
- All five had unchanged camera service PIDs throughout runtime switching.
- tcpdump on the corresponding ZCU pfe1/pfe2 interface captured three incoming
  image UDP packets after each attempted switch and after restoration.
- Final sources: image 01 Camera, images 02–05 Video, matching the initial state.

Raw results, PID comparisons and packet captures are recorded in
[camera-runtime-validation-2026-09-09.json](camera-runtime-validation-2026-09-09.json).
This verifies input-side packets, not visual content or downstream perception
accuracy. No GUI-driven vehicle movement was tested. Camera initialization can
briefly pause image delivery; blocking camera drivers can delay control replies.

## Deployment and recovery

Original sender code and service files are backed up on each Pi under
`/home/pi/camera/backups/20260909T064806Z/`. Deployment restarted each camera
service once. Subsequent source switches do not restart services. To roll back
one device, copy its backed-up cam.py or webcam.py to /home/pi/camera and restart
cam.service or webcam.service respectively. The service configuration itself
was not changed. Runtime selections reset to service defaults on restart.

ZCU2 and ZCU3 require separate camera connection/detection diagnosis before
physical-camera operation can be verified. Source control remains usable for
video input and reports the failed Camera selection without claiming success.
