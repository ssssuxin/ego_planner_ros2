Ubuntu 20/04  ROS:Foxy  
使用方式：  
colcon build --symlink-install  
source  
ros2 launch ego_planner single_run_in_sim.launch.py   

路径不要有中文（自定义消息有中文会出bug）


-----------------  
只进行了这个单机demo的迁移，多机的没做迁移

