PROJECT(local_sensing_node)
cmake_minimum_required(VERSION 3.5)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

#set(ENABLE_CUDA false)
set(ENABLE_CUDA false)

if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
  set(CUDA_NVCC_FLAGS 
#       -gencode arch=compute_20,code=sm_20;
#       -gencode arch=compute_20,code=sm_21;
#       -gencode arch=compute_30,code=sm_30;
#       -gencode arch=compute_35,code=sm_35;
#       -gencode arch=compute_50,code=sm_50;
#       -gencode arch=compute_52,code=sm_52; 
#       -gencode arch=compute_60,code=sm_60;
#       -gencode arch=compute_61,code=sm_61;
      -gencode arch=compute_75,code=sm_75;
  ) 

  SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)

  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)
  generate_dynamic_reconfigure_options(
    cfg/local_sensing_node.cfg
  )
  catkin_package(
      DEPENDS OpenCV Eigen Boost
      CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
  #INCLUDE_DIRS include
      LIBRARIES depth_render_cuda
  )

  include_directories(
    SYSTEM 
    #include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  CUDA_ADD_LIBRARY( depth_render_cuda
      src/depth_render.cu
  )

  add_executable(
    pcl_render_node
    src/pcl_render_node.cpp
  )
  target_link_libraries( pcl_render_node
    depth_render_cuda
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else(ENABLE_CUDA)
  
  # find_package(catkin REQUIRED COMPONENTS
  #     roscpp roslib cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs)
  
  # catkin_package(
  #     DEPENDS Eigen
  #     CATKIN_DEPENDS roscpp roslib pcl_ros
  # )
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(quadrotor_msgs REQUIRED)
  set(MY_DEPENDENCIES
    rclcpp
    pcl_ros
    sensor_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    # PCL
  )
  find_package(Eigen3 REQUIRED)


  include_directories(
    SYSTEM 
    # ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}.
  )

  add_executable(
    pcl_render_node
    src/pointcloud_render_node.cpp
  )
  ament_target_dependencies(pcl_render_node
  ${MY_DEPENDENCIES} 
  )
  target_link_libraries( pcl_render_node
    # ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  
  install(TARGETS
    pcl_render_node
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
endif(ENABLE_CUDA)
