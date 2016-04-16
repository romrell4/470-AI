#include <ros/ros.h>
#include "intrude_client.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "intrude_client" );
  IntrudeClient client;  
  ros::spin();
  return 0;
}
