#include <iostream>
#include <ros/ros.h>
#include <digit_msgs/Digit_Observation.h>
#include <digit_msgs/Digit_Commands.h>

#include "lowlevelapi.h"

class Digit_Llapi
{
public:
    /* Public Member Functions */
    Digit_Llapi(ros::NodeHandle & nh);
    void Publish_Observation_(const llapi_observation_t &obs,
                              const llapi_limits_t *const lim);
    void Print_Commands_();
    void Print_Observation_();

    /* Public Member Variables */
    digit_msgs::Digit_Commands commands_msg_;
    digit_msgs::Digit_Observation observations_msg_;

private:
    /* Private Member Functions */
    void Initialize_Subscribers_();
    void Subscriber_Commands_Callback_(const digit_msgs::Digit_Commands::ConstPtr &msg);

    /* Private Member Variables */
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_commands_;
    ros::Publisher publisher_observation_ = nh_.advertise<digit_msgs::Digit_Observation>("observation", 1);
};