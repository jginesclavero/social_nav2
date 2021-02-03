cmake_minimum_required(VERSION 3.5)
project(social_nav2_goal_updaters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(social_nav2_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_cascade_lifecycle
    nav2_msgs
    diagnostic_msgs
    nav2_costmap_2d
    social_nav2_msgs
)

include_directories(include)

set(SOCIAL_NAV2_GOAL_UPDATER_SRC
  src/social_goal_updater.cpp
)
add_library(${PROJECT_NAME} SHARED ${SOCIAL_NAV2_GOAL_UPDATER_SRC})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(hri_goal_updater_node src/hri_goal_updater.cpp)
ament_target_dependencies(hri_goal_updater_node ${dependencies})
target_link_libraries(hri_goal_updater_node ${PROJECT_NAME})

add_executable(escort_goal_updater_node src/escort_goal_updater.cpp)
ament_target_dependencies(escort_goal_updater_node ${dependencies})
target_link_libraries(escort_goal_updater_node ${PROJECT_NAME})

add_executable(follow_goal_updater_node src/follow_goal_updater.cpp)
ament_target_dependencies(follow_goal_updater_node ${dependencies})
target_link_libraries(follow_goal_updater_node ${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  hri_goal_updater_node
  escort_goal_updater_node
  follow_goal_updater_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()