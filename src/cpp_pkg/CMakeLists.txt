cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catamaran_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(listener_velocity src/listener_velocity.cpp)
ament_target_dependencies(listener_velocity 
                          rclcpp 
                          catamaran_interfaces
                          )

add_executable(motor_brushless_node src/motor_brushless_node.cpp)
ament_target_dependencies(motor_brushless_node 
                          rclcpp
                          catamaran_interfaces
                          )

add_executable(control_velocity src/control_velocity.cpp)
ament_target_dependencies(control_velocity 
                          rclcpp
                          catamaran_interfaces
                          std_msgs
                          )

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp)
ament_target_dependencies(robot_news_station example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp)
ament_target_dependencies(smartphone std_msgs)

add_executable(listener_cmd_vel src/listener_cmd_vel.cpp)
ament_target_dependencies(listener_cmd_vel 
                          rclcpp
                          geometry_msgs
                          )

add_executable(mapping_cmd_to_override src/mapping_cmd_to_override.cpp)
ament_target_dependencies(mapping_cmd_to_override 
                          rclcpp
                          geometry_msgs
                          std_msgs
                          mavros_msgs
                          )   

# Aquí enlazamos wiringPi (librería del sistema), NO como dependencia ROS
find_library(WIRINGPI_LIB wiringPi)
if(NOT WIRINGPI_LIB)
  message(FATAL_ERROR "No se encontró la librería wiringPi. ¿Está instalada?")
endif()

target_link_libraries(motor_brushless_node
  ${WIRINGPI_LIB}
)
     

install(TARGETS
  control_velocity
  listener_velocity
  motor_brushless_node
  listener_cmd_vel
  smartphone
  mapping_cmd_to_override
  robot_news_station
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
