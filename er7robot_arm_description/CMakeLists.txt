cmake_minimum_required(VERSION 3.8)
project(er7robot_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# 安装URDF文件
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装meshes文件夹
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install( # copy the document to where ros2 can find
  DIRECTORY launch rviz  # original place /src/project_name/
  DESTINATION share/${PROJECT_NAME} # destinate place install/share/project_name/
)

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

ament_package()

