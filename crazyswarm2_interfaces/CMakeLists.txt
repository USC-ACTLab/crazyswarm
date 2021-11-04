cmake_minimum_required(VERSION 3.8)
project(crazyswarm2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FullState.msg"
  "msg/GenericLogData.msg"
  "msg/Hover.msg"
  "msg/LogBlock.msg"
  "msg/Position.msg"
  "msg/TrajectoryPolynomialPiece.msg"
  "msg/VelocityWorld.msg"
  "srv/GoTo.srv"
  "srv/Land.srv"
  "srv/NotifySetpointsStop.srv"
  "srv/SetGroupMask.srv"
  "srv/StartTrajectory.srv"
  "srv/Stop.srv"
  "srv/Takeoff.srv"
  "srv/UpdateParams.srv"
  "srv/UploadTrajectory.srv"
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

ament_package()
