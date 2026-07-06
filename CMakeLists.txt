cmake_minimum_required(VERSION 3.10)
project(sentry_decision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/profile.cpp
  src/fsm.cpp
  src/decision_node.cpp
)

# yaml-cpp is a plain CMake package (not ament); link its imported target
# explicitly — ament_auto does not propagate it.
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN sentry_decision::DecisionNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fsm_test test/fsm_test.cpp)
  target_link_libraries(fsm_test ${PROJECT_NAME})
endif()

ament_auto_package()
