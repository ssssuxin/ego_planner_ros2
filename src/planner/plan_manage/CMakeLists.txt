cmake_minimum_required(VERSION 3.5)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#TAG 这里有在Cmakelist.txt里面获取目录路径的方式
get_filename_component(SOURCE_CMAKELIST_DIR "${CMAKE_SOURCE_DIR}" ABSOLUTE)
# message(${SOURCE_CMAKELIST_DIR})
get_filename_component(PROJECT_ROOT_DIR "${SOURCE_CMAKELIST_DIR}/../../.." ABSOLUTE)
# message(${PROJECT_ROOT_DIR})
#TAG 在这里添加这行东西后，可通过find_package的方式来访问包  目前只是编译的msg包能够使用
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_ROOT_DIR}/install)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
# find_package(plan_env REQUIRED)
# find_package(path_searching REQUIRED)
# find_package(bspline_opt REQUIRED)
#TAG 这里很重要的点就是，消息类型用find_package引入，如果是so库那么用下面那个include_directories  所以这里bspline_opt就用下面那个，并且绝对不能用这个，可能会出现链接混乱然后 报错
find_package(traj_utils REQUIRED)
# find_package(message_generation REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   rclcpp
#   std_msgs
#   geometry_msgs
#   quadrotor_msgs
#   plan_env
#   path_searching
#   bspline_opt
  # traj_utils
#   message_generation
#   cv_bridge
# )
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  # plan_env
  # path_searching
  # bspline_opt
  traj_utils
  # message_generation
  cv_bridge
  # PCL
  Eigen3
  nav_msgs
  message_filters
)

find_package(PCL 1.7 REQUIRED)
# catkin_package(CATKIN_DEPENDS message_runtime)
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ego_planner
#  CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
# #  DEPENDS system_lib
# )
# TAG 小功能包获取其他功能包的路径  这里是planner这个大工程里面的一个小包，访问其他小包的方式(这种适用于ortool那种纯代码第三方库)
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

# TAG这个东西是给编译可执行文件时，.cpp或者.h文件引用其他包头文件时 引入路径用的
include_directories(
  include
  SYSTEM
  # ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  # ${PROJECT_ROOT_FA}/uav_simulator/quadrotor_msgs/include
  ${PROJECT_ROOT}/bspline_opt/include
  ${PROJECT_ROOT}/path_searching/include
  ${PROJECT_ROOT}/plan_env/include
  ${PROJECT_ROOT}/traj_utils_so/include
)
add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
ament_target_dependencies(ego_planner_node
${MY_DEPENDENCIES} 
)
target_link_libraries(ego_planner_node 
${PROJECT_ROOT_DIR}/install/bspline_opt/lib/bspline_opt/libbspline_opt.so
${PROJECT_ROOT_DIR}/install/path_searching/lib/path_searching/libpath_searching.so
${PROJECT_ROOT_DIR}/install/plan_env/lib/plan_env/libplan_env.so
${PROJECT_ROOT_DIR}/install/traj_utils_so/lib/traj_utils_so/libtraj_utils.so
)
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server
${MY_DEPENDENCIES} 
# quadrotor_msgs
# "plan_env"
# "path_searching"
# "bspline_opt"
# "message_generation"
)
#M?  这个说实话不知道怎么搞，只能权宜之计
target_link_libraries(traj_server 
${PROJECT_ROOT_DIR}/install/bspline_opt/lib/bspline_opt/libbspline_opt.so
${PROJECT_ROOT_DIR}/install/path_searching/lib/path_searching/libpath_searching.so
${PROJECT_ROOT_DIR}/install/plan_env/lib/plan_env/libplan_env.so
)
# target_link_libraries(bspline_opt 
# )
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
install(TARGETS
  traj_server
  ego_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/#把存yaml文件的文件夹config移到share
)
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

