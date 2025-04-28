cmake_minimum_required(VERSION 3.20)
project(easynav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
  easynav_common
  rclcpp_lifecycle
)

add_library(easynav_core_methods INTERFACE)
target_include_directories(easynav_core_methods INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(easynav_core_methods INTERFACE
  ${dependencies}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  easynav_core_methods
  EXPORT easynav_core_methods
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
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_targets(easynav_core_methods)
ament_export_dependencies(${dependencies})
ament_package()
