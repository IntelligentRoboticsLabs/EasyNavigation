ament_add_gtest(core_method_test core_method_test.cpp)
target_include_directories(core_method_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(core_method_test
  easynav_core_methods
)
ament_target_dependencies(core_method_test ${dependencies})
