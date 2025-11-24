cmake_minimum_required(VERSION 3.8)
project(seda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_definitions(-DSEDA_CONTROLLER_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# === Dynamixel 노드 ===
add_executable(seda_dynamixel_node
  src/seda_dynamixel.cpp
)

ament_target_dependencies(seda_dynamixel_node
  rclcpp
  std_msgs
  dynamixel_sdk
)

# === 상위 컨트롤러 노드 ===
add_executable(seda_controller_node
  src/seda_controller.cpp
)

ament_target_dependencies(seda_controller_node
  rclcpp
  std_msgs
  ament_index_cpp
)

# === 설치 ===
install(TARGETS
  seda_dynamixel_node
  seda_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
