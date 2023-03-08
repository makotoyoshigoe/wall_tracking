cmake_minimum_required(VERSION 3.8)
project(turtlebot3_wall)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)
ament_auto_add_library(${PROJECT_NAME} SHARED
 src/run_along_wall.cpp include/${PROJECT_NAME}/run_along_wall.hpp)

ament_auto_add_executable(run_along_wall
  src/run_along_wall.cpp
  src/run_along_wall_node.cpp
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
