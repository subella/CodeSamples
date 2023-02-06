// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    keypoint_detector_ros.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief ROS wrapper for interfacing keypoint detector with ROS.
 */
//-----------------------------------------------------------------------------

#ifndef Keypoint_Detector_ROS_HPP
#define Keypoint_Detector_ROS_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <target_pose_estimating/keypoint_detector.hpp>

#include "softdrone_target_pose_estimator/Keypoints2D.h"

namespace sdrone
{

class KeypointDetectorROS : public KeypointDetector {
  public:
    typedef sensor_msgs::ImageConstPtr ImageMsg;
    typedef softdrone_target_pose_estimator::Keypoint2D Keypoint2D;
    typedef softdrone_target_pose_estimator::Keypoints2D Keypoints2D;

    KeypointDetectorROS(const ros::NodeHandle &nh);

    KeypointDetectorROS(const ros::NodeHandle &nh,
                        const std::string     &model_file_name,
                        const bool            should_publish_annotated_img);

    ~KeypointDetectorROS() = default;


  private:

    bool should_publish_annotated_img_;

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;

    ros::Time time_stamp_;

    image_transport::Subscriber rgb_image_sub_;

    ros::Publisher keypoints_pub_;

    image_transport::Publisher annotated_img_pub_;

    void rgbImageCallback(const ImageMsg& rgb_img);

    Keypoints2D tensorToKeypoints2D(torch::Tensor& tensor_kpts);
    
};

}; //namespace sdrone

#endif // Tracker_ROS_HPP
