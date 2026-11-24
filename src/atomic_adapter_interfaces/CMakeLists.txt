cmake_minimum_required(VERSION 3.12)
project(atomic_adapter_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB INTERFACE_FILES
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "msg/*.msg"
  "srv/*.srv"
  "action/*.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
