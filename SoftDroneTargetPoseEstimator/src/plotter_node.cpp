// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    plotter_node.cpp
 * @author  Samuel Ubellacker
 * 
 * @brief Plots relevant information.
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <target_pose_estimating/plotter_ros.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plotter_node");
    ros::NodeHandle nh("~");

    ROS_INFO("plotter_node running...");

    sdrone::PlotterROS plotter(nh);
    ros::spin();

    return 0;
}
