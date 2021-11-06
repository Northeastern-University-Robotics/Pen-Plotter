cmake_minimum_required(VERSION 3.0.2)

# Declare the project to CMake
project(controls)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  roslaunch
)

# This macro sets up the Python modules (for message generation)
# by referencing the 'setup.py' file in the package
catkin_python_setup()

# Add and generate the message files for the controls package
add_message_files(
  FILES
  MotorOrientation.msg
  MotorState.msg
)
generate_messages(
  DEPENDENCIES
  controls
)

# Allow CMake to mark raspberry_controls.py to be executable
# instead of having everybody do 'chmod +x raspberry_controls.py'
catkin_install_python(PROGRAMS src/raspberry_controls.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Allow catkin_make to check *.launch files for errors
roslaunch_add_file_check(launch)

# ----- C++ Specific -----

# Describe the C++ compilation process to CMake 
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(nuc_motor_controls src/nuc_motor_controls.cpp)
target_link_libraries(nuc_motor_controls ${catkin_LIBRARIES})

# Declare the package as a catkin project
catkin_package()