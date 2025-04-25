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

set(dependencies
  easynav_common
)

# Localizer Base
add_library(localizer_method_base SHARED
src/easynav_core/LocalizerMethodBase.cpp
)
target_include_directories(localizer_method_base PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(localizer_method_base PUBLIC ${dependencies})

# Planner Base
add_library(planner_method_base SHARED
src/easynav_core/LocalizerMethodBase.cpp
)
target_include_directories(planner_method_base PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_method_base PUBLIC ${dependencies})

# Controller Base
add_library(controller_method_base SHARED
  src/easynav_core/ControllerMethodBase.cpp
)
target_include_directories(controller_method_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(controller_method_base PUBLIC ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

set(target_libs
  localizer_method_base
  planner_method_base
  controller_method_base
)

install(TARGETS
  ${target_libs}
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
  ${target_libs}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
