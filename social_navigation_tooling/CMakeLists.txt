cmake_minimum_required(VERSION 3.5)
project(social_navigation_tooling)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(social_navigation_msgs REQUIRED)

set(dependencies
    rclcpp
    nav_msgs
    pedsim_msgs
    social_navigation_msgs
)

add_executable(dummy_set_agent_action src/dummy_set_agent_action.cpp)
ament_target_dependencies(dummy_set_agent_action ${dependencies})

install(TARGETS
  dummy_set_agent_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
