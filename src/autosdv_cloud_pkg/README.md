# AutoSDV Cloud Vehicle Clients

The package sends the `/ivn/diagnostics/status` dashboard summary through
MQTT and streams `/image_01` to MediaMTX through WHIP.

Set the credentials supplied by the Server PC before starting:

```bash
export AUTOSDV_MQTT_USER=vehicle
export AUTOSDV_MQTT_PASSWORD='...'
export AUTOSDV_WHIP_USER='...'
export AUTOSDV_WHIP_PASSWORD='...'
```

Build and run:

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select autosdv_cloud_pkg --symlink-install
source install/setup.zsh
ros2 launch autosdv_cloud_pkg autosdv_cloud.launch.py
```

The fixed first-stage identifiers are `vehicle_id=a0000` and
`camera_id=camera_01`. ROS parameters can override the broker, WHIP URL,
image topic, frame rate, and bitrate when needed for a test.
