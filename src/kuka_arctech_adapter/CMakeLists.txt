cmake_minimum_required(VERSION 3.12)
project(kuka_arctech_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(atomic_adapter_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(adapter_node_component SHARED
  src/adapter_node.cpp
  src/job_validator.cpp
  src/manual_krc_transport.cpp
  src/mock_krc_transport.cpp
  src/program_executor.cpp
  src/semantic_mapper.cpp
  src/static_checker.cpp
  src/template_engine.cpp
)
target_compile_features(adapter_node_component PRIVATE cxx_std_17)
target_include_directories(adapter_node_component PRIVATE include)
ament_target_dependencies(adapter_node_component PUBLIC
  ament_index_cpp
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
)
target_link_libraries(adapter_node_component PRIVATE nlohmann_json::nlohmann_json)

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

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY templates/
  DESTINATION share/${PROJECT_NAME}/templates
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
  atomic_adapter_interfaces
  rclcpp
  rclcpp_action
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kuka_arctech_adapter
    test/test_kuka_arctech_adapter.cpp
  )
  if(TARGET test_kuka_arctech_adapter)
    target_compile_features(test_kuka_arctech_adapter PRIVATE cxx_std_17)
    target_include_directories(test_kuka_arctech_adapter PRIVATE include)
    target_compile_definitions(test_kuka_arctech_adapter PRIVATE
      KUKA_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(test_kuka_arctech_adapter
      adapter_node_component
      nlohmann_json::nlohmann_json
    )
  endif()
endif()

ament_package()
