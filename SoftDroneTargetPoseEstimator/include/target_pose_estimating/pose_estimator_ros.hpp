// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    pose_estimator_ros.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief ROS wrapper for interfacing pose estimator with ROS.
 */
//-----------------------------------------------------------------------------

#ifndef Pose_Estimator_ROS_HPP
#define Pose_Estimator_ROS_HPP

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <target_pose_estimating/pose_estimator.hpp>
#include <target_pose_estimating/helper_functions.hpp>
#include "softdrone_target_pose_estimator/Keypoints3D.h"

namespace sdrone
{

class PoseEstimatorROS : public PoseEstimator {
  public:
    typedef softdrone_target_pose_estimator::Keypoint3D Keypoint3DMsg;
    typedef softdrone_target_pose_estimator::Keypoints3D Keypoints3DMsg;
    typedef geometry_msgs::PoseWithCovarianceStamped PoseWCov;

    PoseEstimatorROS(const ros::NodeHandle &nh);

    PoseEstimatorROS(const ros::NodeHandle& nh,
                     const std::string&     cad_frame_file_name,
                     const TeaserParams&    params);

    ~PoseEstimatorROS() = default;


  private:

    ros::NodeHandle nh_;

    ros::Time time_stamp_;

    std::string frame_id_;

    ros::Subscriber keypoints_sub_;

    ros::Publisher pose_pub_;

    ros::Publisher transformed_cad_frame_pub_;

    double observation_covariance_translation_;

    double observation_covariance_rotation_;

    void keypoints3DCallback(const Keypoints3DMsg& keypoints_3D_msg);

    void keypoints3DToEigen(const std::vector<Keypoint3DMsg> keypoints_3D, Eigen::MatrixX3d& keypoints_3D_mat);

    void keypoints3DMatToKeypoints3DMsg(Eigen::MatrixX3d& keypoints_3D_mat, Keypoints3DMsg& keypoints_3D_msg);

    void eigenToPoseWCov(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, PoseWCov& pose);

};

}; //namespace softdrone

#endif // Tracker_ROS_HPP
