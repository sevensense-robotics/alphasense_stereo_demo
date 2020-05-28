cmake_minimum_required(VERSION 3.5)
project(alphasense_stereo)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
  src/alphasense_stereo/fisheye_camera_pair.cc
  src/alphasense_stereo/stereo_undistort.cc
  src/nodelets/stereo_undistort_nodelet.cc
)
target_link_libraries(${PROJECT_NAME}_lib )

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cs_install()
cs_export()
