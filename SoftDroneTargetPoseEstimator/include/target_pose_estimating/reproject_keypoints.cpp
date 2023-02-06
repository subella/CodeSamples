// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    keypoints_3D.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/reproject_keypoints.hpp>

#define DEPTH_CONVERSION ((double) 1)

namespace sdrone
{

ReprojectKeypoints::
ReprojectKeypoints()
{
  is_initialized_ = false;
};

ReprojectKeypoints::
ReprojectKeypoints(Eigen::Matrix3d& camera_intrinsics)
{
  init(camera_intrinsics);
};

void ReprojectKeypoints::
init(Eigen::Matrix3d& camera_intrinsics)
{
    camera_intrinsics_ = camera_intrinsics;
    is_initialized_ = true;
};

int ReprojectKeypoints::
reprojectSingleKeypoint(Eigen::Vector2i& keypoint_2D_vec, double z, Eigen::Vector3d& keypoint_3D_vec)
{
    if (!is_initialized_)
      return 0;
    double fx = camera_intrinsics_(0,0);
    double fy = camera_intrinsics_(1,1);
    double cx = camera_intrinsics_(0,2);
    double cy = camera_intrinsics_(1,2);
    int px = keypoint_2D_vec(0);
    int py = keypoint_2D_vec(1);
    double x = (px - cx) * z / fx;
    double y = (py - cy) * z / fy;
    keypoint_3D_vec << x, y, z;
    return 1;
}

int ReprojectKeypoints:: 
projectSingleKeypoint(Eigen::Vector3d& keypoint_3D_vec, Eigen::Vector2i& keypoint_2D_vec)
{
  if (!is_initialized_)
    return 0;
  double fx = camera_intrinsics_(0,0);
  double fy = camera_intrinsics_(1,1);
  double cx = camera_intrinsics_(0,2);
  double cy = camera_intrinsics_(1,2);
  int px = round((fx * keypoint_3D_vec[0] / keypoint_3D_vec[2] + cx));
  int py = round((fy * keypoint_3D_vec[1] / keypoint_3D_vec[2] + cy));
  keypoint_2D_vec << px, py;
  return 1;
}

int ReprojectKeypoints::
reprojectKeypoints(Eigen::MatrixX2i& keypoints_2D_mat, cv::Mat& depth_img, Eigen::MatrixX3d& keypoints_3D_mat)
{
  if (!is_initialized_)
    return 0;

  for (int i=0; i < keypoints_2D_mat.rows(); i++)
  {
    Eigen::Vector2i keypoint_2D_vec = keypoints_2D_mat.row(i);
    unsigned short depth_img_z = depth_img.at<unsigned short>(keypoint_2D_vec[1], keypoint_2D_vec[0]);
    double z = DEPTH_CONVERSION * depth_img_z;
    Eigen::Vector3d keypoint_3D_vec;
    reprojectSingleKeypoint(keypoint_2D_vec, z, keypoint_3D_vec);
    keypoints_3D_mat.row(i) = keypoint_3D_vec;
  }

  return 1;
}

int ReprojectKeypoints::
projectKeypoints(Eigen::MatrixX3d& keypoints_3D_mat, Eigen::MatrixX2i& keypoints_2D_mat)
{
  if (!is_initialized_)
    return 0;

  for (int i=0; i < keypoints_3D_mat.rows(); i++)
  {
    Eigen::Vector3d keypoint_3D_vec = keypoints_3D_mat.row(i);
    Eigen::Vector2i keypoint_2D_vec;
    projectSingleKeypoint(keypoint_3D_vec, keypoint_2D_vec);
    keypoints_2D_mat.row(i) = keypoint_2D_vec;
  }

  return 1;
}

}
