cmake_minimum_required(VERSION 3.8)
project(gazebo_robot_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install( # copy the document to where ros2 can find
  DIRECTORY launch rviz urdf # original place /src/project_name/
  DESTINATION share/${PROJECT_NAME} # destinate place install/share/project_name/
)

ament_package()
