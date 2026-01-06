cmake_minimum_required(VERSION 3.5)
project(elkapod_nav_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

find_package(behaviortree_cpp REQUIRED) 
find_package(nav2_behavior_tree REQUIRED)

nav2_package()

include_directories(include)

# --- 1. CONTROLLER DEPENDENCIES ---
set(controller_dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

# --- 2. BT NODE DEPENDENCIES ---
set(bt_dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  behaviortree_cpp    # <--- FIXED: Removed "_v3"
  nav2_behavior_tree  # <--- ADDED: Often needed for BT Action Node base classes
)

# ==========================================
# LIBRARY 1: ELKAPOD STRAIGHT CONTROLLER
# ==========================================
add_library(elkapod_straight_controller SHARED
  src/elkapod_straight_controller.cpp)

target_compile_definitions(elkapod_straight_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(elkapod_straight_controller
  ${controller_dependencies}
)

# ==========================================
# LIBRARY 2: ELKAPOD BT NODES
# ==========================================
add_library(elkapod_bt_nodes SHARED
  src/get_pose_from_path.cpp)

target_compile_definitions(elkapod_bt_nodes PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(elkapod_bt_nodes
  ${bt_dependencies}
)

# ==========================================
# INSTALLATION & EXPORT
# ==========================================
install(TARGETS 
  elkapod_straight_controller
  elkapod_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

# Export BOTH libraries
ament_export_libraries(elkapod_straight_controller elkapod_bt_nodes)

# Export ALL dependencies
ament_export_dependencies(
  ${controller_dependencies} 
  behaviortree_cpp     # <--- FIXED
  nav2_behavior_tree
)

# --- PLUGIN EXPORTS ---
# 1. Controller needs this (uses pluginlib)
pluginlib_export_plugin_description_file(nav2_core elkapod_straight_controller.xml)

# 2. BT Nodes do NOT need an XML export! 
# (They are loaded by library path in nav2_params.yaml)

ament_package()