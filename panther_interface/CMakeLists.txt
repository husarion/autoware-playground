cmake_minimum_required(VERSION 3.5)
project(panther_interface)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/panther_interface.cpp
  src/panther_interface_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})

set(NODE_NAME ${PROJECT_NAME}_exe)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "panther_interface::LgsvlInterfaceNode"
  EXECUTABLE ${NODE_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
