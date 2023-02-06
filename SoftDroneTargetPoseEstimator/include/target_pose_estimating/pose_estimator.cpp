// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    pose_estimator.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/pose_estimator.hpp>

#define MAXBUFFERSIZE  ((int) 300)

namespace sdrone
{

PoseEstimator::
PoseEstimator()
    :solver_()
{
  is_initialized_ = false;
};

PoseEstimator::
PoseEstimator(const std::string&  cad_frame_file_name,
              const TeaserParams& params)
    : solver_(params)
{
    init(cad_frame_file_name);
    params_ = params;

    // Supress all cout output because we can't turn off 
    // pmc output.
    //std::cout.rdbuf(NULL);
    is_initialized_ = true;
};

void PoseEstimator::
init(const std::string&  cad_frame_file_name)
{
    initCadFrame(cad_frame_file_name);
};

void PoseEstimator::
initCadFrame(const std::string& cad_frame_file_name)
{
    int rows = 0;
    int cols = 3;
    double buffer[MAXBUFFERSIZE];
    std::ifstream file(cad_frame_file_name);
    double x, y, z;
    while (file >> x >> y >> z)
    {
        buffer[rows * cols] = x;
        buffer[rows * cols + 1] = y;
        buffer[rows * cols + 2] = z;
        rows++;
    }

    Eigen::MatrixX3d matrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            matrix(i,j) = buffer[cols * i + j];


    cad_frame_keypoints_ = matrix;
    //cad_frame_keypoints_ = matrix.transpose();
}

int PoseEstimator::
solveTransformation(Eigen::MatrixX3d& keypoints_3D, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    if (!is_initialized_)
        return 0;

    try
    {
        solver_.solve(cad_frame_keypoints_.transpose(), keypoints_3D.transpose());
        auto solution = solver_.getSolution();
        R = solution.rotation;
        t = solution.translation;
        solver_.reset(params_);
        return 1;
    }
    catch (...)
    {
        return 0;
    }
}

void PoseEstimator::
transformCadFrame(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::MatrixX3d& transformed_cad_frame_keypoints){
    for (int i=0; i < cad_frame_keypoints_.rows(); i++)
    {
        // transformed_cad_frame_keypoints.row(i) = R * cad_frame_keypoints_.row(i).transpose() + t;
    }
    transformed_cad_frame_keypoints = cad_frame_keypoints_;
}


}