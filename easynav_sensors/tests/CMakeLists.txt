ament_add_gtest(sensors_node_tests sensors_node_tests.cpp)
target_link_libraries(sensors_node_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(sensors_node_tests ${dependencies})
