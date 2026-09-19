# YOLO runtime on the current Thor HPC

The GUI launches ROS nodes using `/usr/bin/python3` (Python 3.12), not Conda.
The initial environment lacked `ultralytics`, `torch`, and `torchvision`, causing
YOLO to fail before creating its ROS node. The current device is NVIDIA Thor,
L4T R38.4, with CUDA 13.0 reported by its driver.

The selected runtime is PyTorch 2.9.1 / torchvision 0.24.1 from the official
CUDA 13.0 wheel index and Ultralytics 8.3.200. The configured model remains
`best_urp.pt`, with device `cuda:0`.

Installation as the GUI user (`nvidia`):

```sh
/usr/bin/python3 -m pip install --user --break-system-packages \
  torch==2.9.1 torchvision==0.24.1 \
  --index-url https://download.pytorch.org/whl/cu130
/usr/bin/python3 -m pip install --user --break-system-packages --no-deps \
  ultralytics==8.3.200 polars==1.35.2 polars-runtime-32==1.35.2 \
  ultralytics-thop==2.0.18
```

The second command deliberately preserves the existing ROS environment:
NumPy 1.26.4 and the distribution-provided OpenCV 4.6.0. Matplotlib, Pillow,
PyYAML, requests, SciPy and psutil were already installed and met Ultralytics'
requirements. The system's OpenCV import is valid but is not advertised to pip
under the `opencv-python` distribution name, so pip metadata checks alone may
report that requirement missing. Do not replace the system OpenCV just to make
that metadata check pass. Recheck these dependencies before reusing the commands
on another machine. This is a setup record for this Thor, not a portable installer.

The CUDA 13.0 wheel installation route is documented in the
[Ultralytics Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson/).


## Validation (2026-09-08)

Installation completed in the `nvidia` user's Python 3.12 site packages.
`torch.cuda.is_available()` returned true, device NVIDIA Thor. A CUDA torchvision
NMS operation and `best_urp.pt` prediction on a 640×480 test image succeeded.
Runtime versions: torch 2.9.1+cu130, torchvision 0.24.1, ultralytics 8.3.200;
NumPy remained 1.26.4 and system OpenCV remained 4.6.0.

An offscreen Qt integration run used the actual GUI MainWindow/NodeProcessManager,
physical ZCU camera 01 and ROS nodes. Camera → YOLO (`cuda:0`) → YOLO visualizer
→ GUI compressed preview completed without GUI process errors. It received 206
DetectionArray messages and 22 preview frames, with a valid 640×480 GUI image.
The observed scene produced zero boxes; this confirms pipeline execution, not
model detection accuracy. Driving and motion planner were not started.
The integration run closed its GUI and managed nodes afterward. The existing
45 focused regression tests also passed. No YOLO application code or model
weights needed changing. Restart the GUI before using the installed runtime.

[Integration measurements](validation/yolo-runtime-2026-09-08.json).
