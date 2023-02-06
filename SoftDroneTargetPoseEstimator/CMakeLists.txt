cmake_minimum_required(VERSION 3.0.2)
project(softdrone_target_pose_estimator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)


add_message_files(
  FILES
  Keypoint2D.msg
  Keypoints2D.msg
  Keypoint3D.msg
  Keypoints3D.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS rospy
                  roscpp
                  std_msgs
                  sensor_msgs
                  cv_bridge
                  image_transport
                  message_runtime
                  tf2
                  tf2_ros
                  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# LIBRARIES
add_library(helper_functions include/target_pose_estimating/helper_functions.cpp)
target_link_libraries(helper_functions 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_library(keypoint_detector include/target_pose_estimating/keypoint_detector.cpp)
target_link_libraries(keypoint_detector 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TORCH_LIBRARIES}
  TorchVision::TorchVision
)

add_library(keypoint_detector_ros include/target_pose_estimating/keypoint_detector_ros.cpp)
target_link_libraries(keypoint_detector_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TORCH_LIBRARIES}
  TorchVision::TorchVision
  keypoint_detector
)

add_library(reproject_keypoints include/target_pose_estimating/reproject_keypoints.cpp)
target_link_libraries(reproject_keypoints
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_library(reproject_keypoints_ros include/target_pose_estimating/reproject_keypoints_ros.cpp)
target_link_libraries(reproject_keypoints_ros
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  reproject_keypoints
)

add_library(pose_estimator include/target_pose_estimating/pose_estimator.cpp)
target_link_libraries(pose_estimator
 ${catkin_LIBRARIES}
 Eigen3::Eigen
 teaserpp::teaser_registration
 teaserpp::teaser_io
 helper_functions
)

add_library(pose_estimator_ros include/target_pose_estimating/pose_estimator_ros.cpp)
target_link_libraries(pose_estimator_ros
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_io
  pose_estimator
)

add_library(plotter include/target_pose_estimating/plotter.cpp)
target_link_libraries(plotter
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  reproject_keypoints
)

add_library(plotter_ros include/target_pose_estimating/plotter_ros.cpp)
target_link_libraries(plotter_ros
  ${catkin_LIBRARIES}
  helper_functions
  plotter
)

# EXECUTABLES

add_dependencies(helper_functions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(keypoint_detector_node src/keypoint_detector_node.cpp)

add_dependencies(keypoint_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(keypoint_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${TORCH_LIBRARIES}
   TorchVision::TorchVision
   keypoint_detector_ros
 )

add_executable(reproject_keypoints_node src/reproject_keypoints_node.cpp)

add_dependencies(reproject_keypoints_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(reproject_keypoints_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  reproject_keypoints_ros
 )

add_executable(pose_estimator_node src/pose_estimator_node.cpp)

add_dependencies(pose_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_estimator_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_io
  pose_estimator_ros
 )

add_executable(plotter_node src/plotter_node.cpp)

add_dependencies(plotter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(plotter_node
  ${catkin_LIBRARIES}
  plotter_ros
 )

#TESTS
#add_executable(test_keypoint_detector tests/test_keypoint_detector.cpp)
catkin_add_gtest(test_keypoint_detector tests/test_keypoint_detector.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
target_link_libraries(test_keypoint_detector 
  ${GTEST_LIBRARIES} 
  keypoint_detector
  reproject_keypoints
  pose_estimator
)
