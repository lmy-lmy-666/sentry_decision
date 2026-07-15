cmake_minimum_required(VERSION 3.22)
project(sentry_decision_sample)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- library -------------------------------------------------
add_library(sentry_decision_sample_lib SHARED
  src/profile.cpp
  src/fsm.cpp
  src/decision_node.cpp
)
target_include_directories(sentry_decision_sample_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(sentry_decision_sample_lib PUBLIC cxx_std_17)
ament_target_dependencies(sentry_decision_sample_lib
  rclcpp rclcpp_action rclcpp_components nav2_msgs geometry_msgs nav_msgs rm_interfaces
  tf2_ros tf2_geometry_msgs
)
target_link_libraries(sentry_decision_sample_lib yaml-cpp::yaml-cpp)
rclcpp_components_register_node(sentry_decision_sample_lib
  PLUGIN "sentry_decision_sample::DecisionNode"
  EXECUTABLE sentry_decision_sample_node
)

# ---- install -------------------------------------------------
install(TARGETS sentry_decision_sample_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# python helper scripts (mock referee + preflight check)
install(PROGRAMS
  scripts/mock_referee.py
  scripts/mock_referee_tui.py
  scripts/mock_referee_gui.py
  scripts/preflight_check.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- tests ---------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fsm_test test/fsm_test.cpp)
  target_link_libraries(fsm_test sentry_decision_sample_lib)
endif()

ament_package()
