cmake_minimum_required(VERSION 3.5)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  nav_msgs
  # PCL
)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   visualization_msgs
#   cv_bridge
#   message_filters
# )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_env
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS OpenCV
# #  DEPENDS system_lib
# )

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
#TAG 这里非常有意思，三个没有main函数的cpp文件共同编译在一起，作为代码库（在ros2中被当做功能包如msg引入使用）使用
add_library( plan_env SHARED
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )
ament_target_dependencies(plan_env
${MY_DEPENDENCIES} 
)
target_link_libraries( plan_env
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
)
ament_target_dependencies(obj_generator
${MY_DEPENDENCIES} 
)

install(TARGETS
    plan_env
    obj_generator
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