// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    pose_estimator_ros.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/pose_estimator_ros.hpp>

namespace sdrone
{

PoseEstimatorROS::
PoseEstimatorROS(const ros::NodeHandle &nh)
  : nh_(nh),
    PoseEstimator()
{
};

PoseEstimatorROS::
PoseEstimatorROS(const ros::NodeHandle& nh,
                 const std::string&     cad_frame_file_name,
                 const TeaserParams&    params)
  : nh_(nh),
    PoseEstimator(cad_frame_file_name, params)
{
  keypoints_sub_ = nh_.subscribe("keypoints_3d_in", 1, &PoseEstimatorROS::keypoints3DCallback, this);
  pose_pub_ = nh_.advertise<PoseWCov>("estimated_pose_out",  1);
  transformed_cad_frame_pub_ = nh_.advertise<Keypoints3DMsg>("transformed_cad_out",  1);

  nh_.getParam("observation_covariance_translation", observation_covariance_translation_);
  nh_.getParam("observation_covariance_rotation", observation_covariance_rotation_);
};

void PoseEstimatorROS::
keypoints3DCallback(const Keypoints3DMsg& keypoints_3D_msg)
{
  time_stamp_ = keypoints_3D_msg.header.stamp;
  auto keypoints_3D = keypoints_3D_msg.keypoints_3D;
  int num_kpts = keypoints_3D.size();
  Eigen::MatrixX3d keypoints_3D_mat(num_kpts, 3);
  keypoints3DToEigen(keypoints_3D, keypoints_3D_mat);

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  int success = solveTransformation(keypoints_3D_mat, R, t);

  if (success)
  {
    PoseWCov pose_cov;
    eigenToPoseWCov(R, t, pose_cov);
    pose_pub_.publish(pose_cov);


    Eigen::MatrixX3d transformed_keypoints_3D_mat(num_kpts, 3);
    transformCadFrame(R, t, transformed_keypoints_3D_mat);

    Keypoints3DMsg keypoints_3D_msg;
    keypoints3DMatToKeypoints3DMsg(transformed_keypoints_3D_mat, keypoints_3D_msg);
    transformed_cad_frame_pub_.publish(keypoints_3D_msg);
  }
}

// TODO: move code to static helper
void PoseEstimatorROS::
keypoints3DToEigen(const std::vector<Keypoint3DMsg> keypoints_3D, Eigen::MatrixX3d& keypoints_3D_mat)
{
  for (int i=0; i < keypoints_3D.size(); i++)
  {
    Keypoint3DMsg keypoint_3D = keypoints_3D.at(i);
    keypoints_3D_mat.row(i) << keypoint_3D.x, keypoint_3D.y, keypoint_3D.z;
  }  
}

// TODO: Move code to static helper
void PoseEstimatorROS::
keypoints3DMatToKeypoints3DMsg(Eigen::MatrixX3d& keypoints_3D_mat, Keypoints3DMsg& keypoints_3D_msg)
{
  keypoints_3D_msg.header.stamp = time_stamp_;
  for (int i=0; i < keypoints_3D_mat.rows(); i++)
  {
    Keypoint3DMsg keypoint_3D_msg;
    keypoint_3D_msg.x = keypoints_3D_mat(i, 0);
    keypoint_3D_msg.y = keypoints_3D_mat(i, 1);
    keypoint_3D_msg.z = keypoints_3D_mat(i, 2);
    keypoints_3D_msg.keypoints_3D.push_back(keypoint_3D_msg);
  }  

}

void PoseEstimatorROS::
eigenToPoseWCov(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, PoseWCov& pose_cov)
{
  pose_cov.header.stamp = time_stamp_;
  pose_cov.header.frame_id = "target_cam_color_optical_frame";

  pose_cov.pose.pose.position.x = t[0]/1000.;
  pose_cov.pose.pose.position.y = t[1]/1000.;
  pose_cov.pose.pose.position.z = t[2]/1000.;
  
  Eigen::Quaterniond q(R);
  pose_cov.pose.pose.orientation.w = q.w();
  pose_cov.pose.pose.orientation.x = q.x();
  pose_cov.pose.pose.orientation.y = q.y();
  pose_cov.pose.pose.orientation.z = q.z();

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6,6);
  cov(0,0) = observation_covariance_translation_;
  cov(1,1) = observation_covariance_translation_;
  cov(2,2) = observation_covariance_translation_;
  cov(3,3) = observation_covariance_rotation_;
  cov(4,4) = observation_covariance_rotation_;
  cov(5,5) = observation_covariance_rotation_;
  eigenMatToCov(cov, pose_cov);

}

}; //namespace sdrone
