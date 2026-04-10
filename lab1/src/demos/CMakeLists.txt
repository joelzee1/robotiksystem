cmake_minimum_required(VERSION 3.5)
project(demos)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install Python script directories for access as imports
install(DIRECTORY
  ${PROJECT_NAME}/services
  ${PROJECT_NAME}/topics
  ${PROJECT_NAME}/parameters
  ${PROJECT_NAME}/actions
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/services/service_client.py
  ${PROJECT_NAME}/services/service_server.py
  ${PROJECT_NAME}/topics/listener1.py
  ${PROJECT_NAME}/topics/talker1.py
  ${PROJECT_NAME}/topics/listener2.py
  ${PROJECT_NAME}/topics/talker2.py
  ${PROJECT_NAME}/topics/listener3.py
  ${PROJECT_NAME}/parameters/param_talker.py
  ${PROJECT_NAME}/parameters/config_reader.py
  ${PROJECT_NAME}/actions/action_client.py
  ${PROJECT_NAME}/actions/action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
