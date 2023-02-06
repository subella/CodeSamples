// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    reproject_keypoints_ros.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/reproject_keypoints_ros.hpp>



namespace sdrone
{

ReprojectKeypointsROS::
ReprojectKeypointsROS(const ros::NodeHandle& nh)
  : nh_(nh), 
    it_(nh_),
    keypoints_2D_sub_(nh_, "keypoints_2d_in", 1),
    depth_img_sub_(it_, "depth_img_in", 1),
    sync_(SyncPolicy(100), keypoints_2D_sub_, depth_img_sub_)
{
  sync_.registerCallback(boost::bind(&ReprojectKeypointsROS::keypoints2DCallback, this, _1, _2));
  rgb_cam_info_sub_ = nh_.subscribe("rgb_cam_info_in", 1, &ReprojectKeypointsROS::rgbCamInfoCallback, this);
  keypoints_3D_sub_ = nh_.subscribe("keypoints_3d_in", 1, &ReprojectKeypointsROS::keypoints3DCallback, this);

  keypoints_2D_pub_ = nh_.advertise<Keypoints2DMsg>("keypoints_2d_out",  1);
  keypoints_3D_pub_ = nh_.advertise<Keypoints3DMsg>("keypoints_3d_out",  1);
  
};

void ReprojectKeypointsROS::
rgbCamInfoCallback(const CameraInfoMsg& camera_info_msg)
{
  if (is_initialized_)
    return;

  Eigen::Matrix3d camera_intrinsics;
  for (int i=0; i < camera_intrinsics.rows(); i++)
    for(int j=0; j < camera_intrinsics.cols(); j++)
      camera_intrinsics(i, j) = camera_info_msg.K[i*camera_intrinsics.cols()+j];

  init(camera_intrinsics);
  std::cout.precision(17);
  std::cout << camera_intrinsics << std::flush;
}


void ReprojectKeypointsROS::
keypoints2DCallback(const Keypoints2DMsg::ConstPtr& keypoints_2D_msg, const ImageMsg::ConstPtr& depth_img_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  time_stamp_ = keypoints_2D_msg->header.stamp;

  auto keypoints_2D = keypoints_2D_msg->keypoints_2D;
  int num_kpts = keypoints_2D.size();
  auto depth_img = cv_ptr->image;

  Eigen::MatrixX2i keypoints_2D_mat(num_kpts, 2);
  makeKeypoints2DMat(keypoints_2D, keypoints_2D_mat);

  Eigen::MatrixX3d keypoints_3D_mat(num_kpts, 3);
  int success = reprojectKeypoints(keypoints_2D_mat, depth_img, keypoints_3D_mat); 

  if (success)
  {
    Keypoints3DMsg keypoints_3D_msg;
    keypoints3DMatToKeypoints3DMsg(keypoints_3D_mat, keypoints_3D_msg);
    keypoints_3D_pub_.publish(keypoints_3D_msg);
  }

};

void ReprojectKeypointsROS::
keypoints3DCallback(const Keypoints3DMsg& keypoints_3D_msg)
{
  time_stamp_ = keypoints_3D_msg.header.stamp;
  auto keypoints_3D = keypoints_3D_msg.keypoints_3D;
  int num_kpts = keypoints_3D.size();

  Eigen::MatrixX3d keypoints_3D_mat(num_kpts, 3);
  makeKeypoints3DMat(keypoints_3D, keypoints_3D_mat);

  Eigen::MatrixX2i keypoints_2D_mat(num_kpts, 2);
  int success = projectKeypoints(keypoints_3D_mat, keypoints_2D_mat);

  if (success)
  {
    Keypoints2DMsg keypoints_2D_msg;
    keypoints2DMatToKeypoints2DMsg(keypoints_2D_mat, keypoints_2D_msg);
    keypoints_2D_pub_.publish(keypoints_2D_msg);
  }

}

void ReprojectKeypointsROS::
makeKeypoints3DMat(const std::vector<Keypoint3DMsg>& keypoints_3D, Eigen::MatrixX3d& keypoints_3D_mat)
{
  for (int i=0; i < keypoints_3D.size(); i++)
  {
    double x = keypoints_3D[i].x;
    double y = keypoints_3D[i].y;
    double z = keypoints_3D[i].z;
    keypoints_3D_mat.row(i) << x, y, z;
  }
}

void ReprojectKeypointsROS::
makeKeypoints2DMat(const std::vector<Keypoint2DMsg>& keypoints_2D, Eigen::MatrixX2i& keypoints_2D_mat)
{
  for (int i=0; i < keypoints_2D.size(); i++)
  {
    int px = keypoints_2D[i].x;
    int py = keypoints_2D[i].y;
    keypoints_2D_mat.row(i) << px, py;
  }
}

void ReprojectKeypointsROS::
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

void ReprojectKeypointsROS::
keypoints2DMatToKeypoints2DMsg(Eigen::MatrixX2i& keypoints_2D_mat, Keypoints2DMsg& keypoints_2D_msg)
{
  keypoints_2D_msg.header.stamp = time_stamp_;
  for (int i=0; i < keypoints_2D_mat.rows(); i++)
  {
    Keypoint2DMsg keypoint_2D_msg;
    keypoint_2D_msg.x = keypoints_2D_mat(i, 0);
    keypoint_2D_msg.y = keypoints_2D_mat(i, 1);
    keypoints_2D_msg.keypoints_2D.push_back(keypoint_2D_msg);
  }  

}

}; //namespace sdrone
