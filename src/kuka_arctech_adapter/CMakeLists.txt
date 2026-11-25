cmake_minimum_required(VERSION 3.12)
project(kuka_arctech_adapter)

find_package(ament_cmake REQUIRED)
find_package(atomic_adapter_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(adapter_node_component SHARED
  src/adapter_node.cpp
  src/job_validator.cpp
)
target_compile_features(adapter_node_component PRIVATE cxx_std_17)
target_include_directories(adapter_node_component PRIVATE include)
ament_target_dependencies(adapter_node_component
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)

rclcpp_components_register_node(adapter_node_component
  PLUGIN "AdapterNode"
  EXECUTABLE adapter_node
)
add_dependencies(adapter_node adapter_node_component)

install(TARGETS adapter_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
)

ament_package()
