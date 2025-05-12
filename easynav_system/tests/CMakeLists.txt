ament_add_gtest(goalmanager_tests goalmanager_tests.cpp)
target_link_libraries(goalmanager_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(goalmanager_tests ${dependencies})
