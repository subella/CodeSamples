// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    plotter_ros.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/helper_functions.hpp>
#include <target_pose_estimating/plotter_ros.hpp>

namespace sdrone
{

PlotterROS::
PlotterROS(const ros::NodeHandle& nh)
  : nh_(nh),
    it_(nh),
    rgb_img_sub_(it_, "rgb_img_in", 1),
    pose_sub_(nh_, "estimated_pose_in", 1),
    sync_(SyncPolicy(10), rgb_img_sub_, pose_sub_),
    tf_listener_(tf_buffer_)
{
  sync_.registerCallback(boost::bind(&PlotterROS::annotateCadFrameCallback, this, _1, _2));
  cad_keypoints_sub_ = nh_.subscribe("cad_keypoints_in", 1, &PlotterROS::cadKeypointsCallback, this);

  reprojected_cad_keypoints_img_pub_ = it_.advertise("reprojected_keypoints_out",  1);

};

void PlotterROS::
cadKeypointsCallback(const Keypoints3DMsg& cad_frame_keypoints_msg)
{
  if (is_initialized_)
    return;

  int num_kpts = cad_frame_keypoints_msg.keypoints_3D.size();
  Eigen::MatrixX3d cad_frame_keypoints_mat(num_kpts, 3);
  keypoints3DMsgToEigenMat(cad_frame_keypoints_msg, cad_frame_keypoints_mat);
  initCadFrameKeypoints(cad_frame_keypoints_mat);
}

void PlotterROS::
annotateCadFrameCallback(const ImageMsg::ConstPtr& rgb_img_msg,
                         const PoseWCov::ConstPtr& pose_msg)
{
  if (!is_initialized_)
    return;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  geometry_msgs::TransformStamped map_to_optical =
    tf_buffer_.lookupTransform("target_cam_color_optical_frame", pose_msg->header.frame_id, pose_msg->header.stamp, ros::Duration(1.0));

  geometry_msgs::PoseWithCovarianceStamped pwcs_optical;
  tf2::doTransform(*pose_msg, pwcs_optical, map_to_optical);


  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  //poseWCovToEigenMat(*pose_msg, R, t);
  poseWCovToEigenMat(pwcs_optical, R, t);
  Eigen::MatrixX3d transformed_cad_frame_keypoints(cad_frame_keypoints_.rows(), 3);
  t *= 1000;
  transformCadFrame(R, t, transformed_cad_frame_keypoints);

  Eigen::MatrixX2i cad_frame_keypoints_2D (cad_frame_keypoints_.rows(), 2);
  reproject_keypoints.projectKeypoints(transformed_cad_frame_keypoints, cad_frame_keypoints_2D);

  drawCadFrameKeypoints(cv_ptr->image, cad_frame_keypoints_2D);
  reprojected_cad_keypoints_img_pub_.publish(cv_ptr->toImageMsg());
}


}; //namespace sdrone
