cmake_minimum_required(VERSION 3.5)
project(cognitive_arch)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(blackboard_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
#****#
find_package(gb_attention REQUIRED)
find_package(gb_attention_msgs REQUIRED)
#****#
# find_package(OpenCV 2 REQUIRED)


set(dependencies
    rclcpp
    std_msgs
    rclcpp_cascade_lifecycle
    lifecycle_msgs
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
    sensor_msgs
    blackboard_msgs
    cv_bridge
    image_transport
    tf2_ros
    #****#
    gb_attention
    gb_attention_msgs
    #****#
)

include_directories(include)
include_directories(fsm)
include_directories(OpenCV)

add_library(cognitive_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs cognitive_move_bt_node)

add_library(cognitive_pick_bt_node SHARED src/behavior_tree_nodes/Pick.cpp)
list(APPEND plugin_libs cognitive_pick_bt_node )

add_library(cognitive_drop_bt_node SHARED src/behavior_tree_nodes/Drop.cpp)
list(APPEND plugin_libs cognitive_drop_bt_node )

add_library(cognitive_search_bt_node SHARED src/behavior_tree_nodes/Search.cpp)
list(APPEND plugin_libs cognitive_search_bt_node )

add_library(blackboard SHARED
  src/blackboard/BlackBoard.cpp
  src/blackboard/BlackBoardNode.cpp
  src/blackboard/BlackBoardClient.cpp
)
ament_target_dependencies(blackboard ${dependencies})





foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()



add_executable(hfsm_main
  src/TestBT.cpp
  fsm/MyBT.cpp
)
ament_target_dependencies(hfsm_main ${dependencies})

target_link_libraries(hfsm_main blackboard)

install(DIRECTORY fsm launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})


install(TARGETS
  hfsm_main
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(${dependencies})

ament_package()
