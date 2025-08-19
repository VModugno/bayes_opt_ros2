cmake_minimum_required(VERSION 3.8)
project(bayes_opt_ros2)

# Use ament_cmake for a simple Python-script package
find_package(ament_cmake REQUIRED)

# Install the Python node as an executable into lib/<pkg>
install(PROGRAMS
  scripts/ros2_bayes_opt_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
