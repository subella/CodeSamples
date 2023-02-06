#include "helper_functions.hpp"


void keypoints3DMsgToEigenMat(const Keypoints3DMsg& keypoints_3D_msg, Eigen::MatrixX3d& keypoints_3D_mat)
{
  std::vector<Keypoint3DMsg> keypoints_3D = keypoints_3D_msg.keypoints_3D;
  for (int i=0; i < keypoints_3D.size(); i++)
  {
    double x = keypoints_3D[i].x;
    double y = keypoints_3D[i].y;
    double z = keypoints_3D[i].z;
    keypoints_3D_mat.row(i) << x, y, z;
  }
}

void poseWCovToEigenMat(const PoseWCov& pose_cov, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    t[0] = pose_cov.pose.pose.position.x;
    t[1] = pose_cov.pose.pose.position.y;
    t[2] = pose_cov.pose.pose.position.z;

    Eigen::Quaterniond q(pose_cov.pose.pose.orientation.w,
                         pose_cov.pose.pose.orientation.x,
                         pose_cov.pose.pose.orientation.y,
                         pose_cov.pose.pose.orientation.z);
    R = q.toRotationMatrix();

}

void eigenMatToCov(const Eigen::MatrixXd& cov, PoseWCov &pwc)
{
  const int map[6] = {0,1,2,5,4,3};
  for(int i=0; i<6; i++)
  {
    for(int j=0; j<6; j++)
    {
      pwc.pose.covariance[map[i]*6 + map[j]] = cov(i,j);
    }
  }
}
