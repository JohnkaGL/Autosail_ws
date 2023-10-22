cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(sailboat_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(autosail_description REQUIRED)

xacro_add_files(
  urdf/sailboat_gazebo.urdf.xacro
  INSTALL DESTINATION urdf
)
# Install meshes and textures
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)



ament_package()
