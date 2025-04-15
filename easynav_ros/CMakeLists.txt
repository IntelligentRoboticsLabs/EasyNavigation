cmake_minimum_required(VERSION 3.20)
project(easynav_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(easynav_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  easynav_core
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_ros/EasyNavNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(easynav_main src/easynav_main.cpp)
ament_target_dependencies(easynav_main ${dependencies})
target_link_libraries(easynav_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
