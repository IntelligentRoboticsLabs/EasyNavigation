ament_add_gtest(perceptionsops_tests perceptionsops_tests.cpp)
target_link_libraries(perceptionsops_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(perceptionsops_tests ${dependencies})
