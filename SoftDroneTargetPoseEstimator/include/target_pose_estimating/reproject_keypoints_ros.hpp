// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    reproject_keypoints_ros.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief ROS wrapper for reprojecting 2D keypoints to 3D.
 */
//-----------------------------------------------------------------------------

#ifndef REPROJECT_KEYPOINTS_ROS_HPP
#define REPROJECT_KEYPOINTS_ROS_HPP


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <target_pose_estimating/reproject_keypoints.hpp>

#include "softdrone_target_pose_estimator/Keypoints2D.h"
#include "softdrone_target_pose_estimator/Keypoints3D.h"

namespace sdrone
{

class ReprojectKeypointsROS : public ReprojectKeypoints {

  public:
    
    typedef softdrone_target_pose_estimator::Keypoint2D Keypoint2DMsg;
    typedef softdrone_target_pose_estimator::Keypoints2D Keypoints2DMsg;
    typedef softdrone_target_pose_estimator::Keypoint3D Keypoint3DMsg;
    typedef softdrone_target_pose_estimator::Keypoints3D Keypoints3DMsg;
    typedef sensor_msgs::Image ImageMsg;
    typedef sensor_msgs::CameraInfo CameraInfoMsg;
    typedef message_filters::sync_policies::ApproximateTime<Keypoints2DMsg, ImageMsg> SyncPolicy;

    ReprojectKeypointsROS(const ros::NodeHandle &nh);

    ~ReprojectKeypointsROS() = default;

  private:

    ros::NodeHandle nh_;

    ros::Time time_stamp_;

    std::string frame_id_;

    image_transport::ImageTransport it_;

    message_filters::Subscriber<Keypoints2DMsg> keypoints_2D_sub_;

    ros::Subscriber keypoints_3D_sub_;

    ros::Subscriber rgb_cam_info_sub_;

    image_transport::SubscriberFilter depth_img_sub_;

    ros::Publisher keypoints_2D_pub_;

    ros::Publisher keypoints_3D_pub_;

    message_filters::Synchronizer<SyncPolicy> sync_;

    void rgbCamInfoCallback(const CameraInfoMsg& camera_info_msg);

    void keypoints2DCallback(const Keypoints2DMsg::ConstPtr& keypoints_2D_msg, const ImageMsg::ConstPtr& depth_img_msg);

    void keypoints3DCallback(const Keypoints3DMsg& keypoints_3D_msg);

    void makeKeypoints2DMat(const std::vector<Keypoint2DMsg>& keypoints_2D, Eigen::MatrixX2i& keypoints_2D_mat);

    void makeKeypoints3DMat(const std::vector<Keypoint3DMsg>& keypoints_3D, Eigen::MatrixX3d& keypoints_3D_mat);

    void keypoints3DMatToKeypoints3DMsg(Eigen::MatrixX3d& keypoints_3D_mat, Keypoints3DMsg& keypoints_3D_msg);

    void keypoints2DMatToKeypoints2DMsg(Eigen::MatrixX2i& keypoints_2D_mat, Keypoints2DMsg& keypoints_2D_msg);

};

}; //namespace sdrone

#endif // REPROJECT_KEYPOINT_ROS_HPP
