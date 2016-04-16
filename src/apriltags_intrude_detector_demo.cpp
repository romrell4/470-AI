#include <ros/ros.h>
#include "apriltags_intrude_detector.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "apriltags_intrude_detector" );
  for( unsigned int i=0; i < argc; i++ ) {
    std::cout << i << " " << argv[i] << std::endl;
  }
  AprilTagsIntrudeDetector detector;  
  ros::spin();
  return 0;
}
