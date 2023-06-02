cmake_minimum_required(VERSION 3.8)
project(ros2_control_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)

######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
# Node
  scripts/Move_Step_Node.py
  scripts/Sub_gui_node.py
  scripts/move_base_node.py
  scripts/sub_GUI_for_move_step_node.py
  
  scripts/lidar_control.py
  scripts/motor_lidar_mux.py
  scripts/joy_control.py
  scripts/sub_nav2.py
  scripts/gui_node.py
  scripts/test_sub_pub.py
  scripts/joy.py



  scripts/a_sdk_group_bulk_read.py
  scripts/a_sdk_group_bulk_write.py
  scripts/a_sdk_group_sync_read.py
  scripts/a_sdk_group_sync_write.py
  scripts/a_sdk_packet_handler.py
  scripts/a_sdk_port_handler.py
  scripts/a_sdk_protocol1_packet_handler.py
  scripts/a_sdk_robotis_def.py

  SLAM/env.py
  SLAM/sensor.py
  SLAM/main.py
  
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 

  DESTINATION share/${PROJECT_NAME}
)

ament_package()
