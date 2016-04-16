#ifndef INTRUDE_CLIENT_H_
#define INTRUDE_CLIENT_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

class IntrudeClient {
public:
  IntrudeClient();
  virtual ~IntrudeClient();

  void position_callback( const geometry_msgs::Pose2D& msg );

  ros::NodeHandle    m_nh;
  ros::Subscriber    m_sub;
  ros::ServiceClient m_client;

};

#endif // INTRUDE_CLIENT_H_
