#include <ros/ros.h>
#include <pc_attitude_estimation/attitude_estimation.hpp>

int main(int argc, char **argv)
{
        ros::init(argc, argv, "attitude_estimation_node");
        ROS_INFO("attitude_estimation_node running...");
        pc_attitude_estimation::AttitudeEstimation attitude_estimation;
        ros::spin();
        return 0;
}
