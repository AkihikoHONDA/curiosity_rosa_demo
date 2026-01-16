cmake_minimum_required(VERSION 3.8)
project(curiosity_rosa_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_package_xml()
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(DIRECTORY curiosity_rosa_demo
  DESTINATION ${PYTHON_INSTALL_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureAndScore.srv"
  DEPENDENCIES builtin_interfaces
)

install(DIRECTORY config launch resource srv
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/simulator_node
  scripts/adapter_node
  scripts/agent_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
