cmake_minimum_required(VERSION 3.5)
project(path_searching)

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
find_package(Eigen3 REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  visualization_msgs
#   plan_env
  cv_bridge
  Eigen3
  # PCL
)

find_package(PCL 1.7 REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_searching
#  CATKIN_DEPENDS plan_env
# #  DEPENDS system_lib
# )
#TAG 自制的没有main函数的几个cpp编译成库文件，然后如何被工程内的其他功能包使用，1.编译add_library编译好 2.这边引用那边的时候，在include_directories键入那边的src源文件路径即可
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
include_directories( 
    SYSTEM 
    include 
    # ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_ROOT}/plan_env/include
)

add_library( path_searching SHARED 
    src/dyn_a_star.cpp
    )
ament_target_dependencies(path_searching
${MY_DEPENDENCIES} 
)


# target_include_directories(path_searching PRIVATE "/home/suxin/English_Path/NEW_WORK_PATH/MY_FINAL_PJ/ROS2_ego_planner/src/planner/plan_env/include")
install(TARGETS
    path_searching
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