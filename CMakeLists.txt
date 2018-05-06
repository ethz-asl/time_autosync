cmake_minimum_required(VERSION 2.8.3)
project(time_autosync)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV 3 REQUIRED COMPONENTS core calib3d video)

add_definitions(-std=c++11 -o0 -g)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/time_autosync.cpp
)

############
# BINARIES #
############
cs_add_executable(time_autosync_node
  src/time_autosync_node.cpp
)
target_link_libraries(time_autosync_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
