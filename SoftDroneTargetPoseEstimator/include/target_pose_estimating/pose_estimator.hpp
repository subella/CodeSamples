// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    pose_estimator.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief Class for solving point cloud rigid transformation.
 */
//-----------------------------------------------------------------------------
#include <fstream>
#include <teaser/registration.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#ifndef Pose_Estimator_HPP
#define Pose_Estimator_HPP


namespace sdrone
{

typedef teaser::RobustRegistrationSolver::Params TeaserParams;
typedef teaser::RobustRegistrationSolver TeaserSolver;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

class PoseEstimator {

  public:

    PoseEstimator();

    PoseEstimator(const std::string&  cad_frame_file_name,
                  const TeaserParams& params);

    int solveTransformation(Eigen::MatrixX3d& keypoints_3D, Eigen::Matrix3d& R, Eigen::Vector3d& t);

  protected:

    bool is_initialized_;

    Eigen::MatrixX3d cad_frame_keypoints_;

    teaser::RobustRegistrationSolver solver_;

    TeaserParams params_;

    void init(const std::string& cad_frame_file_name);

    void initCadFrame(const std::string& cad_frame_file_name);   

    void transformCadFrame(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::MatrixX3d& transformed_cad_frame_keypoints);   



};

}; //namespace sdron

#endif // Pose_Estimator_HPP