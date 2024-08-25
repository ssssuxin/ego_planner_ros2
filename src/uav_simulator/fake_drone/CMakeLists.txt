cmake_minimum_required(VERSION 3.5)
project(poscmd_2_odom)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  nav_msgs
  quadrotor_msgs
  # PCL
)
# find_package(nav_msgs REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   nav_msgs
#   quadrotor_msgs
# )
find_package(Eigen3 REQUIRED)

# catkin_package()

include_directories(
# include
  # ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# add_executable (poscmd_2_odom src/poscmd_2_odom.cpp )
# target_link_libraries(poscmd_2_odom
#   ${catkin_LIBRARIES})

add_executable (poscmd_2_odom src/poscmd_2_odom.cpp )
ament_target_dependencies(poscmd_2_odom
${MY_DEPENDENCIES} 
)

install(TARGETS
  poscmd_2_odom
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/#把存yaml文件的文件夹config移到share
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
##1.自身工程的.h库
ament_package()