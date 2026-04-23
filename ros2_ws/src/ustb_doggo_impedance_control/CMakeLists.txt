cmake_minimum_required(VERSION 3.8)
project(ustb_doggo_impedance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)

# 节点 joint_impedance
add_executable(joint_impedance src/joint_impedance.cpp)

target_include_directories(joint_impedance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(joint_impedance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  joint_impedance
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

# 节点 forward_kinematics
add_executable(forward_kinematics src/forward_kinematics.cpp)

target_include_directories(forward_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(forward_kinematics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  forward_kinematics
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

target_link_libraries(forward_kinematics Eigen3::Eigen)

# 节点 cartesian_impedance
add_executable(cartesian_impedance src/cartesian_impedance.cpp)

target_include_directories(cartesian_impedance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(cartesian_impedance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  cartesian_impedance
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "nav_msgs"
)

target_link_libraries(cartesian_impedance 
  Eigen3::Eigen
  pinocchio::pinocchio
  urdfdom_model
)

# 节点 trot_gait_generator
add_executable(trot_gait_generator src/trot_gait_generator.cpp)
target_include_directories(trot_gait_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trot_gait_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  trot_gait_generator
  "rclcpp"
  "std_msgs"
)

# 节点 keyboard_control
add_executable(keyboard_control src/keyboard_control.cpp)

target_include_directories(keyboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(keyboard_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  keyboard_control
  "rclcpp"
  "std_msgs"
)

# 节点 keyboard_control_trot
add_executable(keyboard_control_trot src/keyboard_control_trot.cpp)

target_include_directories(keyboard_control_trot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(keyboard_control_trot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  keyboard_control_trot
  "rclcpp"
  "std_msgs"
)

# 安装
install(TARGETS 
  joint_impedance
  forward_kinematics
  cartesian_impedance
  trot_gait_generator
  keyboard_control
  keyboard_control_trot
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
