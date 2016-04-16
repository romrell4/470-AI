# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(apriltags_intrude_detector)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             cv_bridge
             image_transport
             std_msgs
             message_generation
             apriltags
            )

#find_package(OpenCV 2.4 REQUIRED COMPONENTS core highgui imgproc calib3d
#                                            ml features2d objdetect )
find_package(OpenCV REQUIRED)

add_message_files (
  FILES
  apriltags_intrude.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
   CATKIN_DEPENDS message_runtime
)


# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            /usr/local/include
                            /usr/include/eigen3
                            )

# Define a library target called hello_world
#add_library(hello_world src/hello_world.cpp)
#target_link_libraries(hello_world ${roscpp_LIBRARIES})

add_executable(apriltags_intrude_detector src/apriltags_intrude_detector.cpp
                                          src/apriltags_intrude_detector_demo.cpp
              )
target_link_libraries(apriltags_intrude_detector ${catkin_LIBRARIES} 
                                                 ${OpenCV_LIBRARIES}
                                                 apriltags
                     )


