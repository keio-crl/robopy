cmake_minimum_required(VERSION 3.0.2)
project(assembly_2)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch urdf meshes collision_meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
