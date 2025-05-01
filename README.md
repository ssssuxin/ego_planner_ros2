Ubuntu 20/04  ROS:Foxy  
使用方式：  
colcon build --symlink-install  
source  
ros2 launch ego_planner single_run_in_sim.launch.py   

路径不要有中文（自定义消息有中文会出bug）

# Humble版本
有位老哥fork了本工程做了22.04的https://github.com/Kaede-Rukawa/ego_planner_ros_humble 

-----------------  
只进行了这个单机demo的迁移，多机的没做迁移

