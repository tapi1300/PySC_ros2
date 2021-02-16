cmake_minimum_required(VERSION 3.5)
project(scan_fake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
)

add_executable(node_pub src/node_pub.cpp)
ament_target_dependencies(node_pub ${dependencies})

add_executable(node_sub src/node_sub.cpp)
ament_target_dependencies(node_sub ${dependencies})


install(TARGETS
  node_pub
  node_sub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()


endif()


ament_export_dependencies(${dependencies})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  include
)


install(DIRECTORY include/
  DESTINATION include/
)


ament_package()