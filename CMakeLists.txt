cmake_minimum_required(VERSION 3.8)
project(turtlebot3_wall_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)
ament_auto_add_library(${PROJECT_NAME} SHARED
 src/wall_tracking.cpp include/${PROJECT_NAME}/wall_tracking.hpp)

ament_auto_add_executable(wall_tracking
  src/wall_tracking.cpp
  src/wall_tracking_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  models
  config
  launch
)
