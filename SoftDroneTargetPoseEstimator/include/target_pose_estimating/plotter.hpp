// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    plotter.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief Class for plotting.
 */
//-----------------------------------------------------------------------------

#ifndef Plotter_HPP
#define Plotter_HPP

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include <target_pose_estimating/reproject_keypoints.hpp>

namespace sdrone
{

class Plotter {
  public:
    // Plotter();

    Plotter();

  protected:

    ReprojectKeypoints reproject_keypoints;

    bool is_initialized_;

    Eigen::MatrixX3d cad_frame_keypoints_;

    void initCadFrameKeypoints(Eigen::MatrixX3d& cad_frame_keypoints);

    void transformCadFrame(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::MatrixX3d& transformed_cad_frame_keypoints);

    void drawCadFrameKeypoints(cv::Mat& img, Eigen::MatrixX2i& keypoints_2D_mat);

};

}; //namespace sdrone

#endif // Plotter_HPP