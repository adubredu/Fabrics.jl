#include "Digit_Llapi.hpp"

/* Public Member Function Definitions */
Digit_Llapi::Digit_Llapi(ros::NodeHandle &nh)
{
    nh_ = nh;
}
void Digit_Llapi::Publish_Observation_(const llapi_observation_t &obs,
                                       const llapi_limits_t *const lim)
{

    observations_msg_.time = obs.time;
    observations_msg_.error = obs.error;
    observations_msg_.battery_charge = obs.battery_charge;

    /* Base */
    observations_msg_.base_orientation.w = obs.base.orientation.w;
    observations_msg_.base_orientation.x = obs.base.orientation.x;
    observations_msg_.base_orientation.y = obs.base.orientation.y;
    observations_msg_.base_orientation.z = obs.base.orientation.z;
    std::copy(std::begin(obs.base.translation), std::end(obs.base.translation), std::begin(observations_msg_.base_translation));
    std::copy(std::begin(obs.base.linear_velocity), std::end(obs.base.linear_velocity), std::begin(observations_msg_.base_linear_velocity));
    std::copy(std::begin(obs.base.angular_velocity), std::end(obs.base.angular_velocity), std::begin(observations_msg_.base_angular_velocity));

    /* Imu */
    observations_msg_.imu_orientation.w = obs.imu.orientation.w;
    observations_msg_.imu_orientation.x = obs.imu.orientation.x;
    observations_msg_.imu_orientation.y = obs.imu.orientation.y;
    observations_msg_.imu_orientation.z = obs.imu.orientation.z;
    std::copy(std::begin(obs.imu.angular_velocity), std::end(obs.imu.angular_velocity), std::begin(observations_msg_.imu_angular_velocity));
    std::copy(std::begin(obs.imu.linear_acceleration), std::end(obs.imu.linear_acceleration), std::begin(observations_msg_.imu_linear_acceleration));
    std::copy(std::begin(obs.imu.magnetic_field), std::end(obs.imu.magnetic_field), std::begin(observations_msg_.imu_magnetic_field));

    /* Motors */
    std::copy(std::begin(obs.motor.position), std::end(obs.motor.position), std::begin(observations_msg_.motor_position));
    std::copy(std::begin(obs.motor.velocity), std::end(obs.motor.velocity), std::begin(observations_msg_.motor_velocity));
    std::copy(std::begin(obs.motor.torque), std::end(obs.motor.torque), std::begin(observations_msg_.motor_torque));

    /* Joints */
    std::copy(std::begin(obs.joint.position), std::end(obs.joint.position), std::begin(observations_msg_.joint_position));
    std::copy(std::begin(obs.joint.velocity), std::end(obs.joint.velocity), std::begin(observations_msg_.joint_velocity));

    /* Motor Limits */
    std::copy(std::begin(lim->torque_limit), std::end(lim->torque_limit), std::begin(observations_msg_.motor_limit_torque));
    std::copy(std::begin(lim->damping_limit), std::end(lim->damping_limit), std::begin(observations_msg_.motor_limit_damping));
    std::copy(std::begin(lim->velocity_limit), std::end(lim->velocity_limit), std::begin(observations_msg_.motor_limit_velocity));

    /* Publish Observation */
    publisher_observation_.publish(observations_msg_);

    return;
}
void Digit_Llapi::Print_Commands_()
{
    std::cout << "\n=== PRINT COMMANDS ===\n\n";
    std::cout << "fallback_opmode: " << commands_msg_.fallback_opmode << "\n";
    std::cout << "apply_command: " << commands_msg_.apply_command << "\n";
    std::cout << "\n";
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_torque[" << i << "]: " << commands_msg_.motor_torque[i] << "\n";
    }
    std::cout << "\n";
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_velocity[" << i << "]: " << commands_msg_.motor_velocity[i] << "\n";
    }
    std::cout << "\n";
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_damping[" << i << "]: " << commands_msg_.motor_damping[i] << "\n";
    }

    return;
}
void Digit_Llapi::Print_Observation_()
{
    std::cout << "\n=== PRINT OBSERVATION ===\n\n";
    std::cout << "time: " << observations_msg_.time << "\n";
    std::cout << "error: " << observations_msg_.error << "\n";
    std::cout << "battery_charge: " << observations_msg_.battery_charge << "\n";

    std::cout << "\n-> Base\n";
    std::cout << "base_orientation [w,x,y,z]: [" << observations_msg_.base_orientation.w << ", " << observations_msg_.base_orientation.x << ", " << observations_msg_.base_orientation.y << ", " << observations_msg_.base_orientation.z << "]\n";
    std::cout << "base_translation [x,y,z]: [" << observations_msg_.base_translation[0] << ", " << observations_msg_.base_translation[1] << ", " << observations_msg_.base_translation[2] << "]\n";
    std::cout << "base_linear_velocity [x,y,z]: [" << observations_msg_.base_linear_velocity[0] << ", " << observations_msg_.base_linear_velocity[1] << ", " << observations_msg_.base_linear_velocity[2] << "]\n";
    std::cout << "base_angular_velocity [x,y,z]: [" << observations_msg_.base_angular_velocity[0] << ", " << observations_msg_.base_angular_velocity[1] << ", " << observations_msg_.base_angular_velocity[2] << "]\n";

    std::cout << "\n-> Imu\n";
    std::cout << "imu_orientation [w,x,y,z]: [" << observations_msg_.imu_orientation.w << ", " << observations_msg_.imu_orientation.x << ", " << observations_msg_.imu_orientation.y << ", " << observations_msg_.imu_orientation.z << "]\n";
    std::cout << "imu_angular_velocity [x,y,z]: [" << observations_msg_.imu_angular_velocity[0] << ", " << observations_msg_.imu_angular_velocity[1] << ", " << observations_msg_.imu_angular_velocity[2] << "]\n";
    std::cout << "imu_linear_acceleration [x,y,z]: [" << observations_msg_.imu_linear_acceleration[0] << ", " << observations_msg_.imu_linear_acceleration[1] << ", " << observations_msg_.imu_linear_acceleration[2] << "]\n";
    std::cout << "imu_magnetic_field [x,y,z]: [" << observations_msg_.imu_magnetic_field[0] << ", " << observations_msg_.imu_magnetic_field[1] << ", " << observations_msg_.imu_magnetic_field[2] << "]\n";

    std::cout << "\n-> Motors\n";
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_position[" << i << "]: " << observations_msg_.motor_position[i] << "\n";
    }
    std::cout << "\n";

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_velocity[" << i << "]: " << observations_msg_.motor_velocity[i] << "\n";
    }
    std::cout << "\n";

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_torque[" << i << "]: " << observations_msg_.motor_torque[i] << "\n";
    }

    std::cout << "\n-> Joints\n";
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        std::cout << "joint_position[" << i << "]: " << observations_msg_.joint_position[i] << "\n";
    }
    std::cout << "\n";

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        std::cout << "joint_velocity[" << i << "]: " << observations_msg_.joint_velocity[i] << "\n";
    }

    std::cout << "\n-> Motor Limits\n";
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_limit_torque[" << i << "]: " << observations_msg_.motor_limit_torque[i] << "\n";
    }
    std::cout << "\n";

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_limit_velocity[" << i << "]: " << observations_msg_.motor_limit_velocity[i] << "\n";
    }
    std::cout << "\n";

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::cout << "motor_limit_damping[" << i << "]: " << observations_msg_.motor_limit_damping[i] << "\n";
    }

    return;
}

/* Private Member Function Definitions */
void Digit_Llapi::Initialize_Subscribers_()
{
    subscriber_commands_ = nh_.subscribe("/digit_commands", 1, &Digit_Llapi::Subscriber_Commands_Callback_, this);
    return;
}
void Digit_Llapi::Subscriber_Commands_Callback_(const digit_msgs::Digit_Commands::ConstPtr &msg)
{
    commands_msg_.fallback_opmode = msg->fallback_opmode;
    commands_msg_.apply_command = msg->apply_command;

    std::copy(std::begin(msg->motor_torque), std::end(msg->motor_torque), std::begin(commands_msg_.motor_torque));
    std::copy(std::begin(msg->motor_damping), std::end(msg->motor_damping), std::begin(commands_msg_.motor_damping));
    std::copy(std::begin(msg->motor_velocity), std::end(msg->motor_velocity), std::begin(commands_msg_.motor_velocity));
    return;
}
