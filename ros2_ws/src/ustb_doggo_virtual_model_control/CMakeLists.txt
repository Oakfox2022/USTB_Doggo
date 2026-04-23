cmake_minimum_required(VERSION 3.8)
project(ustb_doggo_virtual_model_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 节点 virtual_model_control
add_executable(virtual_model_control src/virtual_model_control.cpp)
target_include_directories(virtual_model_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(virtual_model_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  virtual_model_control
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
)
target_link_libraries(virtual_model_control 
  Eigen3::Eigen
)

# 节点 keyboard_control_vmc
add_executable(keyboard_control_vmc src/keyboard_control_vmc.cpp)
target_include_directories(keyboard_control_vmc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(keyboard_control_vmc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  keyboard_control_vmc
  "rclcpp"
  "std_msgs"
)

install(TARGETS 
  virtual_model_control
  keyboard_control_vmc
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/)

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
