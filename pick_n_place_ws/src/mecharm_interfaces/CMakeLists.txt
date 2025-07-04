cmake_minimum_required(VERSION 3.5)
project(mecharm_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(msg_files
  "msg/MecharmAngles.msg"
  "msg/MecharmCoords.msg"
  "msg/MecharmSetAngles.msg"
  "msg/MecharmSetCoords.msg"
  "msg/MecharmGripperStatus.msg"
  "msg/MecharmPumpStatus.msg"
  "msg/MecharmErrorStatus.msg"
  "msg/MecharmCameraPose.msg"
)

set(srv_files
  "srv/GetAngles.srv"
  "srv/SetAngles.srv"
  "srv/GetCoords.srv"
  "srv/SetCoords.srv"
  "srv/GripperStatus.srv"
  "srv/PumpStatus.srv"
  "srv/PickObject.srv"
  "srv/CapturePose.srv"
  "srv/ArmGripperControl.srv"
  "srv/ArmLinearControl.srv"
  "srv/ArmMoveToPick.srv"
  "srv/SearchObject.srv"
  "srv/SetInitialPose.srv"
  "srv/GoToGoal.srv"
  "srv/TransformPose.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
