// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    plotter_ros.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief ROS wrapper for interfacing plotter with ROS.
 */
//-----------------------------------------------------------------------------

#ifndef Plotter_ROS_HPP
#define Plotter_ROS_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <target_pose_estimating/plotter.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "softdrone_target_pose_estimator/Keypoints2D.h"
#include "softdrone_target_pose_estimator/Keypoints3D.h"

namespace sdrone
{

class PlotterROS : public Plotter {
  public:
    typedef sensor_msgs::Image ImageMsg;
    typedef softdrone_target_pose_estimator::Keypoint3D Keypoint3DMsg;
    typedef softdrone_target_pose_estimator::Keypoints3D Keypoints3DMsg;
    typedef geometry_msgs::PoseWithCovarianceStamped PoseWCov;
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, PoseWCov> SyncPolicy;

    PlotterROS(const ros::NodeHandle& nh);

    ~PlotterROS() = default;


  private:

    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;

    image_transport::ImageTransport it_;

    image_transport::SubscriberFilter rgb_img_sub_;

    ros::Subscriber cad_keypoints_sub_;

    message_filters::Subscriber<PoseWCov> pose_sub_;

    image_transport::Publisher reprojected_cad_keypoints_img_pub_;

    message_filters::Synchronizer<SyncPolicy> sync_;

    void cadKeypointsCallback(const Keypoints3DMsg& cad_frame_keypoints);

    void annotateCadFrameCallback(const ImageMsg::ConstPtr& rgb_img_msg, 
                                  const PoseWCov::ConstPtr& pose_msg);
    
};

}; //namespace sdrone

#endif // Plotter_ROS_HPP
