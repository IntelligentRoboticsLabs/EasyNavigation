cmake_minimum_required(VERSION 3.20)
project(easynav_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pcl_ros REQUIRED)

set(dependencies
  rclcpp_lifecycle
  pcl_ros
)

add_library(easynav_configuration SHARED
  src/easynav_common/Configuration.cpp
)
target_include_directories(easynav_configuration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(easynav_types SHARED
  src/easynav_common/types/MapTypeBase.cpp
)
target_include_directories(easynav_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(easynav_types ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  easynav_configuration
  easynav_types
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  easynav_configuration
  easynav_types
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
