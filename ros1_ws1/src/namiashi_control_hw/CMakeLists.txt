cmake_minimum_required(VERSION 3.0.2)
project(namiashi_control_hw)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  geometry_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp hardware_interface controller_manager geometry_msgs
)

add_library(${PROJECT_NAME}
  src/namiashi_hardware_interface.cpp
  src/serial_motor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  /usr/include/asio
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} pthread
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES namiashi_hw_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(namiashi_control_hw_node src/main.cpp)
target_link_libraries(namiashi_control_hw_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS namiashi_control_hw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)