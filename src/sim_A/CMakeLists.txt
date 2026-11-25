cmake_minimum_required(VERSION 3.12)
project(sim_A)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(atomic_adapter_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(sim_a_node_component SHARED
  src/sim_a_node.cpp
)
target_compile_features(sim_a_node_component PRIVATE cxx_std_17)
target_include_directories(sim_a_node_component PRIVATE include)
ament_target_dependencies(sim_a_node_component PUBLIC
  ament_index_cpp
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)
target_link_libraries(sim_a_node_component PRIVATE nlohmann_json::nlohmann_json)

rclcpp_components_register_node(sim_a_node_component
  PLUGIN "SimANode"
  EXECUTABLE sim_a_node
)
add_dependencies(sim_a_node sim_a_node_component)

install(TARGETS sim_a_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
)

ament_package()
