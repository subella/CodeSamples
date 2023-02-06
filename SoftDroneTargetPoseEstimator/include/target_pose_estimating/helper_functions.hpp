#ifndef HELPER_FUNCTIONS_INCLUDED
#define HELPER_FUNCTINOS_INCLUDED

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "softdrone_target_pose_estimator/Keypoints3D.h"

typedef softdrone_target_pose_estimator::Keypoint3D Keypoint3DMsg;
typedef softdrone_target_pose_estimator::Keypoints3D Keypoints3DMsg;
typedef geometry_msgs::PoseWithCovarianceStamped PoseWCov;

void keypoints3DMsgToEigenMat(const Keypoints3DMsg& keypoints_3D_msg, Eigen::MatrixX3d& keypoints_3D_mat);

void poseWCovToEigenMat(const PoseWCov& pose_cov, Eigen::Matrix3d& R, Eigen::Vector3d& t);

void eigenMatToCov(const Eigen::MatrixXd& cov, PoseWCov &pwc);

#endif