// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    reproject_keypoints_node.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief Reprojects 2D keypoint pixel locations
 *        to 3D coordinates using depth image and
 *        camera intrinsics.
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <target_pose_estimating/reproject_keypoints_ros.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reproject_keypoints_node");
	ros::NodeHandle nh("~");

	ROS_INFO("reproject_keypoints_node running...");

	sdrone::ReprojectKeypointsROS reproject_keypoints(nh);
	ros::spin();

	return 0;
}
