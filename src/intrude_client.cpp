#include "intrude_client.h"
#include "apriltags_intrude_detector/apriltags_intrude.h"

#define APRILTAGS_INTRUDE_MSG_NAME      "apriltags_intrude"

IntrudeClient::IntrudeClient() {
  m_client = m_nh.serviceClient<apriltags_intrude_detector::apriltags_intrude>(APRILTAGS_INTRUDE_MSG_NAME);
  m_sub = m_nh.subscribe("tracked_pos", 1, &IntrudeClient::position_callback, this);
}

IntrudeClient::~IntrudeClient() {

}

void IntrudeClient::position_callback( const geometry_msgs::Pose2D& msg ) {

  apriltags_intrude_detector::apriltags_intrude srv;
  srv.request.x = msg.x;
  srv.request.y = msg.y;
  if( m_client.call(srv) ) {
    int id = (int)srv.response.id;
    if( id >= 0 ) {
      ROS_INFO("Intrude ID: %d", id);
    }
  }
  else {
    ROS_ERROR("Failed to call service apriltags_intrude");
  }
}
