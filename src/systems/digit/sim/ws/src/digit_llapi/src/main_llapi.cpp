#include <string>
#include <iostream>

#include "ros/ros.h"
#include "lowlevelapi.h"
#include "Digit_Llapi.hpp"

int main(int argc, char *argv[])
{
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

    /* Initialize ROS Node */
    ros::init(argc, argv, "digit_llapi_node"); // node name
    ros::NodeHandle nh("~");
    ros::Rate ros_rate(1000);

    /* Initialize class */
    Digit_Llapi digit_llapi(nh);

    /* Define inputs and outputs (updated each iteration) */
    llapi_command_t command = {0};
    llapi_observation_t observation;

    /* Connect to robot (need to send commands until the subscriber connects) */
    command.apply_command = false;
    while (!llapi_get_observation(&observation))
    {
        llapi_send_command(&command);
    }
    std::cout << "=========== Connected ===========\n\n";

    // Get local copy of command limits (torque and damping)
    const llapi_limits_t *limits = llapi_get_limits();

    while (ros::ok())
    {

        /* Update observation */
        int return_val = llapi_get_observation(&observation);
        if (return_val < 1)
        {
            /* Error occurred - next observation not ready */
            // std::cout << "Error occurred\n\n";
        }
        else if (return_val)
        {
            /* New data received */
            // std::cout << "New data received\n\n";

            /* Update Command Subscriber */
            ros::spinOnce();
            // digit_llapi.Print_Commands_();      // Print commands for check

            /* Publish observation */
            digit_llapi.Publish_Observation_(observation, limits);
            // digit_llapi.Print_Observation_();   // Print observation for check

            /* Update command fields from message */
            for (int i = 0; i < NUM_MOTORS; ++i)
            {
                command.motors[i].torque = digit_llapi.commands_msg_.motor_torque[i];
                command.motors[i].velocity = digit_llapi.commands_msg_.motor_velocity[i];
                command.motors[i].damping = digit_llapi.commands_msg_.motor_damping[i];
            }
            command.fallback_opmode = digit_llapi.commands_msg_.fallback_opmode;
            command.apply_command = digit_llapi.commands_msg_.apply_command;
            // command.fallback_opmode = Locomotion;
            // command.apply_command = true;

            /* Send Command (updated from callback function) */
            llapi_send_command(&command);
        }
        else
        {
            // No new data
            std::cout << "No new data received\n\n";
        }

        /* Check if llapi has become disconnected */
        if (!llapi_connected())
        {
            std::cout << "\n------------------------- Disconnected! ---------------------------\n";
            // Handle error case. You don't need to re-initialize subscriber
            // Calling llapi_send_command will keep low level api open
        }
        // std::cout << "\n\n -----------------------------------------------------------------------------\n\n";
        
        /* ros sleep */
        ros_rate.sleep();
    }

    return 0;
}