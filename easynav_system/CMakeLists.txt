cmake_minimum_required(VERSION 3.20)
project(easynav_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_interfaces)
find_package(easynav_common)
find_package(easynav_controller)
find_package(easynav_localizer)
find_package(easynav_maps_manager)
find_package(easynav_planner)
find_package(easynav_sensors)
find_package(geometry_msgs)
find_package(tf2_geometry_msgs)
find_package(tf2)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_interfaces
  easynav_common
  easynav_controller
  easynav_localizer
  easynav_maps_manager
  easynav_planner
  easynav_sensors
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_system/SystemNode.cpp
  src/easynav_system/GoalManager.cpp
  src/easynav_system/GoalManagerClient.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(system_main src/system_main.cpp)
ament_target_dependencies(system_main ${dependencies})
target_link_libraries(system_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  system_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
