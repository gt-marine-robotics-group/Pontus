cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20) # C++ 20 is required for the blueview driver
project(pontus_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
find_package(pontus_msgs REQUIRED)


# C++
include_directories(
  include
  $(rclcpp_INCLUDE_DIRS)
  ${OpenCV_INCLUDE_DIRS}
  colormaps
  bvtsdk/include
)

add_library(bvtsdk SHARED IMPORTED GLOBAL)
set_property(TARGET bvtsdk PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bvtsdk/lib/libbvtsdk.so)

add_executable(ros_blueview_driver src/blueview_ros_driver.cpp src/blueview_wrapper.cpp)

target_link_libraries(ros_blueview_driver bvtsdk)

ament_target_dependencies(ros_blueview_driver rosidl_default_runtime rclcpp std_msgs sensor_msgs image_transport cv_bridge OpenCV pontus_msgs)


install(DIRECTORY
  launch
  pontus_sensors
  config
  bvtsdk/colormaps
  bvtsdk/data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  bvtsdk/lib/.
  DESTINATION lib
)

install (TARGETS ros_blueview_driver
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Python
ament_python_install_package(${PROJECT_NAME})

# Uncomment this to add new python nodes
install(PROGRAMS
  # add python executable file paths here
  pontus_sensors/imu_monitor.py
  pontus_sensors/camera_republish.py
  pontus_sensors/imu_republish.py
  pontus_sensors/depth_republish.py
  pontus_sensors/oak_imu_republish.py
  DESTINATION lib/${PROJECT_NAME}
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
