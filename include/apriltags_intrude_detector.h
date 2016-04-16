#ifndef APRILTAGS_INTRUDE_DETECTOR_H_
#define APRILTAGS_INTRUDE_DETECTOR_H_

#include <utility>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include "apriltags_intrude_detector/apriltags_intrude.h"
#include "geo_datatype.h"

class AprilTagsIntrudeDetector {
public:
  AprilTagsIntrudeDetector( AprilTags::TagCodes codes = AprilTags::tagCodes36h11 );
  virtual ~AprilTagsIntrudeDetector();

  void image_callback( const sensor_msgs::ImageConstPtr& msg);

  bool update_tags( cv::Mat& image );
  std::vector<AprilTags::TagDetection> extract_tags( cv::Mat& image);

 
  bool get_intrude( apriltags_intrude_detector::apriltags_intrude::Request& req,
                    apriltags_intrude_detector::apriltags_intrude::Response& res); 
  int get_tag_id( int x, int y );
  void visualize_tags( cv::Mat& image );
 
  ros::NodeHandle                 m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber     m_sub;

  ros::ServiceServer              m_intrude_srv;

  std::vector<std::pair<AprilTags::TagDetection, Polygon2D> >  m_tags;  

  AprilTags::TagDetector* mp_tag_detector;
  AprilTags::TagCodes     m_tag_codes;

  bool                    m_update_tags;
};

#endif // APRILTAGS_INTRUDE_DETECTOR_H_
