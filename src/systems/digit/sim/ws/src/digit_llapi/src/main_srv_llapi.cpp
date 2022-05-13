#include <string>
#include <iostream>

#include "ros/ros.h"
#include "lowlevelapi.h" 
#include "digit_msgs/Digit_Commands_srv.h"
#include "digit_msgs/Digit_Observation_srv.h"

llapi_command_t command = {0};
llapi_observation_t observation;
const llapi_limits_t *limits; 

bool command_server(digit_msgs::Digit_Commands_srv::Request &req,
digit_msgs::Digit_Commands_srv::Response &res)
{
    for (int i=0; i < NUM_MOTORS; i++)
    {
        command.motors[i].torque = req.cmd.motor_torque[i];
        command.motors[i].velocity = req.cmd.motor_velocity[i];
        command.motors[i].damping = req.cmd.motor_damping[i];
    }
    command.fallback_opmode = req.cmd.fallback_opmode;
    command.apply_command = req.cmd.apply_command;
    // llapi_send_command(&command);
    res.status.data = true;  
    return true;
}

bool observation_server(digit_msgs::Digit_Observation_srv::Request &req, digit_msgs::Digit_Observation_srv::Response &res)
{
    int ret_val = llapi_get_observation(&observation);

    res.obs.time = observation.time;
    res.obs.error = observation.error;
    res.obs.battery_charge = observation.battery_charge;

    /* Base */
    res.obs.base_orientation.w = observation.base.orientation.w;
    res.obs.base_orientation.x = observation.base.orientation.x;
    res.obs.base_orientation.y = observation.base.orientation.y;
    res.obs.base_orientation.z = observation.base.orientation.z;
    std::copy(std::begin(observation.base.translation), std::end(observation.base.translation), std::begin(res.obs.base_translation));
    std::copy(std::begin(observation.base.linear_velocity), std::end(observation.base.linear_velocity), std::begin(res.obs.base_linear_velocity));
    std::copy(std::begin(observation.base.angular_velocity), std::end(observation.base.angular_velocity), std::begin(res.obs.base_angular_velocity));

    /* Imu */
    res.obs.imu_orientation.w = observation.imu.orientation.w;
    res.obs.imu_orientation.x = observation.imu.orientation.x;
    res.obs.imu_orientation.y = observation.imu.orientation.y;
    res.obs.imu_orientation.z = observation.imu.orientation.z;
    std::copy(std::begin(observation.imu.angular_velocity), std::end(observation.imu.angular_velocity), std::begin(res.obs.imu_angular_velocity));
    std::copy(std::begin(observation.imu.linear_acceleration), std::end(observation.imu.linear_acceleration), std::begin(res.obs.imu_linear_acceleration));
    std::copy(std::begin(observation.imu.magnetic_field), std::end(observation.imu.magnetic_field), std::begin(res.obs.imu_magnetic_field));

    /* Motors */
    std::copy(std::begin(observation.motor.position), std::end(observation.motor.position), std::begin(res.obs.motor_position));
    std::copy(std::begin(observation.motor.velocity), std::end(observation.motor.velocity), std::begin(res.obs.motor_velocity));
    std::copy(std::begin(observation.motor.torque), std::end(observation.motor.torque), std::begin(res.obs.motor_torque));

    /* Joints */
    std::copy(std::begin(observation.joint.position), std::end(observation.joint.position), std::begin(res.obs.joint_position));
    std::copy(std::begin(observation.joint.velocity), std::end(observation.joint.velocity), std::begin(res.obs.joint_velocity));

    /* Motor Limits */
    std::copy(std::begin(limits->torque_limit), std::end(limits->torque_limit), std::begin(res.obs.motor_limit_torque));
    std::copy(std::begin(limits->damping_limit), std::end(limits->damping_limit), std::begin(res.obs.motor_limit_damping));
    std::copy(std::begin(limits->velocity_limit), std::end(limits->velocity_limit), std::begin(res.obs.motor_limit_velocity)); 
    res.status.data = true;
    std::cout << "sent observation\n";
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "llapi_server");
    ros::NodeHandle n;
    ros::Rate ros_rate(1000);

    /* The publisher address should be changed to the ip address of the robot */
    if (std::string(argv[1]) == "real")
    {
        const char *publisher_address = "10.10.1.1";
        llapi_init(publisher_address);
    }
    else
    {
        const char *publisher_address = "127.0.0.1";
        llapi_init(publisher_address);
    }

    command.apply_command = false;
    while (!llapi_get_observation(&observation))
    {
        llapi_send_command(&command);
    }
    std::cout << "=========== Connected ===========\n\n";

    limits = llapi_get_limits();


    

    ros::ServiceServer obs_service = n.advertiseService("observation_service", observation_server);
    ros::ServiceServer cmd_service = n.advertiseService("command_service", command_server);

    ROS_INFO("llapi server is ready.");
    // ros::spin();
    while (ros::ok())
    {
        llapi_send_command(&command);
        ros::spinOnce();
        ros_rate.sleep();
    }

    return EXIT_SUCCESS;
}