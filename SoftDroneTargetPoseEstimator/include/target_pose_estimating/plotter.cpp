// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    plotter.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/plotter.hpp>

namespace sdrone
{

// Plotter::
// Plotter()
// {
//   is_initialized_ = false;
// };

Plotter::
Plotter()
{
  // TODO: fix this architecture
  Eigen::Matrix3d camera_intrinsics;
  camera_intrinsics << 629.10406494140625,                  0,   637.203369140625,
                         0,   637.203369140625,                  0,
                         637.203369140625,                  0,   628.583251953125;

  reproject_keypoints.init(camera_intrinsics);
  is_initialized_ = false;
};

void Plotter::
initCadFrameKeypoints(Eigen::MatrixX3d& cad_frame_keypoints)
{
  cad_frame_keypoints_ = cad_frame_keypoints;
  is_initialized_ = true;
};

void Plotter::
transformCadFrame(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::MatrixX3d& transformed_cad_frame_keypoints){
  for (int i=0; i < cad_frame_keypoints_.rows(); i++)
  {
    transformed_cad_frame_keypoints.row(i) = R * cad_frame_keypoints_.row(i).transpose() + t;
  }
}

void Plotter::
drawCadFrameKeypoints(cv::Mat& img, Eigen::MatrixX2i& keypoints_2D_mat)
{
  for (int i = 0; i < keypoints_2D_mat.rows(); i++)
  {
    int px = keypoints_2D_mat.coeff(i, 0);
    int py = keypoints_2D_mat.coeff(i, 1);
    cv::Point pt = cv::Point(px, py);
    cv::circle(img, pt, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
  }
}

}
