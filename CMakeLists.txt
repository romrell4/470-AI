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
            )

#find_package(OpenCV 2.4 REQUIRED COMPONENTS core highgui imgproc calib3d
#                                            ml features2d objdetect )

find_package(cmake_modules)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
if( NOT OpenCV_FOUND ) 
  message( FATAL_ERROR "OpenCV not found\n" )
  return()
endif()



find_package(CGAL REQUIRED)
if( NOT CGAL_FOUND )
  message( FATAL_ERROR "CGAL not found\n" ) 
  return()
endif()

# Find package apriltags
find_package(PkgConfig)
pkg_check_modules(APRILTAGS apriltags)
if( NOT APRILTAGS_FOUND )
  message( "Apriltags not found\n" )
  return()
endif()
find_path(APRILTAGS_INCLUDE_DIR AprilTags/TagDetector.h
    HINTS ${APRILTAGS_INCLUDEDIR} ${APRILTAGS_INCLUDE_DIRS})

find_library(APRILTAGS_LIBRARY apriltags
    HINTS ${APRILTAGS_LIBDIR} ${APRILTAGS_LIBRARY_DIRS})

add_service_files(
  FILES
  apriltags_intrude.srv
)

generate_messages ()

# Declare this project as a catkin package
catkin_package(
   CATKIN_DEPENDS roscpp message_runtime
)

include( ${CGAL_USE_FILE} )

# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            ${CGAL_INCLUDE_DIR}
                            ${APRILTAGS_INCLUDE_DIRS}
                            )

add_executable(apriltags_intrude_detector src/apriltags_intrude_detector.cpp
                                          src/apriltags_intrude_detector_demo.cpp
              )
target_link_libraries(apriltags_intrude_detector ${catkin_LIBRARIES} 
                                                 ${OpenCV_LIBRARIES}
                                                 ${CGAL_LIBRARIES}
                                                 ${APRILTAGS_LIBRARY}
                     )

add_executable(intrude_client src/intrude_client.cpp
                              src/intrude_client_demo.cpp
              )

target_link_libraries(intrude_client ${catkin_LIBRARIES} 
                     )


