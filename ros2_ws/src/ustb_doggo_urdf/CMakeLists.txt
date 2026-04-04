cmake_minimum_required(VERSION 3.5)
project(ustb_doggo_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

