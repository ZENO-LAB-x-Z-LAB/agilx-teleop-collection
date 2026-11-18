# AgxGripper status feedback message

std_msgs/Header header

# Raw position in the active controller mode.
# If mode == "width", the unit is m.
# If mode == "angle", the unit is deg.
float64 position

# Active gripper feedback/control mode reported by the hardware.
# Typical values: "width" or "angle".
string mode

# Gripper status
float64 width           # Current gripper opening width, unit: m; NaN when unavailable in angle mode
float64 force           # Current gripping force, unit: N

# Driver status
bool voltage_too_low        # Voltage too low
bool motor_overheating      # Motor overheating
bool driver_overcurrent     # Driver overcurrent
bool driver_overheating     # Driver overheating
bool sensor_status          # Sensor status
bool driver_error_status    # Driver error status
bool driver_enable_status   # Driver enable status
bool homing_status          # Homing/zero position status
