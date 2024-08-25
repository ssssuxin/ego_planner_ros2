cmake_minimum_required(VERSION 3.5)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(traj_utils REQUIRED)
# find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  visualization_msgs
#   plan_env
  cv_bridge
#   traj_utils
#   path_searching
  # PCL
  Eigen3
)

find_package(PCL 1.7 REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bspline_opt
#  CATKIN_DEPENDS plan_env 
# #  DEPENDS system_lib
# )
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
include_directories( 
    SYSTEM 
    include 
    # ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_ROOT}/path_searching/include
    ${PROJECT_ROOT}/plan_env/include
    ${PROJECT_ROOT}/traj_utils_so/include
)

add_library( bspline_opt SHARED
    src/uniform_bspline.cpp 
    src/bspline_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
ament_target_dependencies(bspline_opt
${MY_DEPENDENCIES} 
)
# target_link_libraries(traj_server 
# /home/suxin/English_Path/NEW_WORK_PATH/MY_FINAL_PJ/ROS2_ego_planner/install/bspline_opt/lib/libbspline_opt.so

# )




install(TARGETS
  bspline_opt
  DESTINATION lib/${PROJECT_NAME}
)
# install(
#   TARGETS bspline_opt
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
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