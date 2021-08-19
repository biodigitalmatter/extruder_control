cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)


# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(stepper
  SRCS stepper.cpp ${ROS_LIB_DIR}/time.cpp 
  BOARD uno 
  PORT /dev/ttyACM0
)

generate_arduino_firmware(simple_stepper
  SRCS simple_stepper.cpp ${ROS_LIB_DIR}/time.cpp 
  BOARD uno 
  PORT /dev/ttyACM0
)

