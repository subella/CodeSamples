// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    keypoint_detector_node.cpp
 * @author  Samuel Ubellacker
 * 
 * @brief Runs an neural net to detect pre-labelled keypoints from an rgb image.
 *        Subscribes to the Image topic and publishes a custom Keypoints message.
 *        Specify the file location of the jitted model (model.ts).
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <target_pose_estimating/keypoint_detector_ros.hpp>

std::string model_file_name_;
bool should_publish_annotated_img_;

void load_params(const ros::NodeHandle& nh)
{
  nh.getParam("model_file_name", model_file_name_);
  nh.getParam("should_publish_annotated_img", should_publish_annotated_img_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keypoint_detector_node");
    ros::NodeHandle nh("~");

    ROS_INFO("keypoint_detector_node running...");


    load_params(nh);
    sdrone::KeypointDetectorROS keypoint_detector(nh, model_file_name_, should_publish_annotated_img_);
    ros::spin();

    return 0;
}
