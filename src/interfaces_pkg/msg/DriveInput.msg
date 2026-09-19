bool forward
bool reverse
bool steer_left
bool steer_right
bool reset
bool emergency_stop

# Keyboard inputs above remain digital. Controller inputs use normalized axes.
bool analog
float32 throttle
float32 brake
# Position mode follows steering [-1, 1]; otherwise steer_left/right accumulate.
bool steering_position
float32 steering
