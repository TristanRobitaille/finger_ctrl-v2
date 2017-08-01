cmake_minimum_required (VERSION 2.6)

# include directory containing header files
include_directories (${libRobot_IMU_Control_SOURCE_DIR}/include)

# set as shared library
add_library (Robot_IMU_Control SHARED Robot_IMU_Control.cpp )

set_target_properties(
 Robot_IMU_Control
  PROPERTIES
    SOVERSION ${libRobot_IMU_Control_VERSION_MAJOR}
    VERSION ${libRobot_IMU_Control_VERSION}
)

# link external libraries
target_link_libraries (Robot_IMU_Control ${LIBUSB_1_LIBRARIES})
target_link_libraries (Robot_IMU_Control ${Boost_LIBRARIES})

# set the install targets
install (TARGETS Robot_IMU_Control LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

file (GLOB Robot_IMU_Control_header_files ${libRobot_IMU_Control_SOURCE_DIR}/include/*.h)
install (FILES ${libRobot_IMU_Control_header_files} DESTINATION include/libRobot_IMU_Control)
