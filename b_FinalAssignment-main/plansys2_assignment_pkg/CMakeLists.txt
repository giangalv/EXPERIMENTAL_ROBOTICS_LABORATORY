cmake_minimum_required(VERSION 3.5)
project(plansys2_assignment_pkg)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_python_install_package(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    plansys2_msgs
    plansys2_executor
)

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  plansys2_assignment_pkg/bug0.py
  plansys2_assignment_pkg/go_to_action_node.py
  plansys2_assignment_pkg/go_to_point_srv.py
  plansys2_assignment_pkg/wall_follow_srv.py
  plansys2_assignment_pkg/camera_check_node.py
  plansys2_assignment_pkg/marker_searcher_action.py
  plansys2_assignment_pkg/motor_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
