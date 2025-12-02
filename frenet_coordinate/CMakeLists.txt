cmake_minimum_required(VERSION 2.8.3)
project(frenet_coordinate)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  fsd_common_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  tf
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

add_message_files(  # Tell catkin which custom message files to include
  FILES
  RoadWidth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS fsd_common_msgs roscpp rospy std_msgs visualization_msgs tf message_runtime
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${roscpp_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN_DEFINITIONS})

add_executable(${PROJECT_NAME} 
  src/frenet_coordinate_handle.cpp
  src/main.cpp
  src/cubic_spline_interpolation.cpp
  src/fitted_curve.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

