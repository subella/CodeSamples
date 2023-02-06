// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    pose_estimator_node.hpp
 * @author  Samuel Ubellacker
 * 
 * @brief Given detected 3D keypoints, estimates the pose of the 
 *        target using point cloud registration with Teaser.
 */
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <target_pose_estimating/pose_estimator_ros.hpp>

std::string cad_frame_file_name_;
sdrone::TeaserParams params_;

void load_params(const ros::NodeHandle& nh)
{
  nh.getParam("cad_frame_file_name", cad_frame_file_name_);
  ROS_INFO_STREAM("Pose estimator cad frame name: " << cad_frame_file_name_);
  std::cout << "\n\ncad frame name: " << cad_frame_file_name_ << std::endl;
  nh.getParam("noise_bound", params_.noise_bound);
  nh.getParam("cbar2", params_.cbar2);
  nh.getParam("estimate_scaling", params_.estimate_scaling);
  int rotation_max_iterations;
  nh.param("rotation_max_iterations", rotation_max_iterations);
  params_.rotation_max_iterations = (unsigned long) rotation_max_iterations;
  nh.getParam("rotation_gnc_factor", params_.rotation_gnc_factor);
  nh.getParam("rotation_cost_threshold", params_.rotation_cost_threshold);
  // TODO: We can expose this to the yaml if needed.
  params_.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
}

int main(int argc, char **argv)
{
    std::cout << "\n\n\n Pose Estimator \n\n\n" << std::endl;
    ros::init(argc, argv, "pose_estimator_node");
    ros::NodeHandle nh("~");

    ROS_INFO("pose_estimator_node running...");

    load_params(nh);
    sdrone::PoseEstimatorROS pose_estimator(nh, cad_frame_file_name_, params_);
    ros::spin();

	return 0;
}
