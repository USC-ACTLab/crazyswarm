cmake_minimum_required(VERSION 3.8)
project(crazyflie_examples)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  config
  data
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
