cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project(pontus_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandMode.msg"
  "msg/SemanticObject.msg"
  "msg/SemanticMap.msg"
  "msg/SemanticMetaGate.msg"
  "srv/GateSideInformation.srv"
  "srv/AddSemanticObject.srv"
  "srv/AddMetaGate.srv"
  "srv/SetEnum.srv"
  "action/GoToPose.action"
  "action/SearchRegion.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
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
