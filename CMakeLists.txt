cmake_minimum_required(VERSION 3.8)
project(turtlebot3_nav2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装所有文件目录
install(DIRECTORY
  launch
  maps
  rviz
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/multi_goal_patrol.py
  scripts/moving_obstacle_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()