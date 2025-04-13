cmake_minimum_required(VERSION 3.8)
project(easynav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_core/EasyNavNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(easynav_main src/easynav_main.cpp)
ament_target_dependencies(easynav_main ${dependencies})
target_link_libraries(easynav_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  easynav_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
