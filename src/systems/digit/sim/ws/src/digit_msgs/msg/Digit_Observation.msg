# Digit_Observation Custom ROS Message
#   Check lowlevelapi.h to see motor and joint order
Header header
float64 time    # time since program started
bool error
int16 battery_charge

# Base
geometry_msgs/Quaternion base_orientation
float64[3] base_translation # world frame
float64[3] base_linear_velocity # body frame
float64[3] base_angular_velocity # body or world?

# Imu
geometry_msgs/Quaternion imu_orientation
float64[3] imu_angular_velocity
float64[3] imu_linear_acceleration
float64[3] imu_magnetic_field

# Motor
float64[20] motor_position
float64[20] motor_velocity
float64[20] motor_torque

# Joint
float64[10] joint_position
float64[10] joint_velocity

# Motor Limits
float64[20] motor_limit_torque
float64[20] motor_limit_damping
float64[20] motor_limit_velocity

