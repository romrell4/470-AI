#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "apriltags_intrude_detector.h"

using namespace std;
using namespace cv;

#define APRILTAGS_INTRUDE_DETECTOR_VIEW "AprilTags Intrude Detector"
#define APRILTAGS_INTRUDE_MSG_NAME      "apriltags_intrude"
#define APRILTAGS_INFO_MSG_NAME         "apriltags_info"


void AprilTagsIntrudeDetector::image_callback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
  
  if( m_update_tags ) {
    cout << "update tags " << endl;
    update_tags( cv_ptr->image );
    m_update_tags = false;
  }

  visualize_tags( cv_ptr->image );
  
  int key_value = cv::waitKey(30);
  if( key_value >= 0 ) {
    cout << "key value " << key_value << endl;
    if( key_value == 1048689 ) {
      // value of q
      ros::shutdown();
    }
    else if( key_value == 1048693 ) {
      // value of u
      cout << " U Pressed " << endl;
      m_update_tags = true;    
    }
  }
  if( false == ros::ok() ) {
    ros::shutdown();
  }
}

AprilTagsIntrudeDetector::AprilTagsIntrudeDetector( AprilTags::TagCodes codes  ) : m_it( m_nh ) , m_tag_codes( codes )  {
  
  cv::namedWindow(APRILTAGS_INTRUDE_DETECTOR_VIEW);
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &AprilTagsIntrudeDetector::image_callback, this);
  mp_tag_detector = new AprilTags::TagDetector( m_tag_codes ); 

  m_intrude_srv = m_nh.advertiseService( APRILTAGS_INTRUDE_MSG_NAME, &AprilTagsIntrudeDetector::get_intrude, this);
  m_info_srv = m_nh.advertiseService( APRILTAGS_INFO_MSG_NAME, &AprilTagsIntrudeDetector::get_info, this);

  m_update_tags = false;
}

AprilTagsIntrudeDetector::~AprilTagsIntrudeDetector() {
  if( mp_tag_detector ) {
    delete mp_tag_detector;
    mp_tag_detector = NULL;
  }
  cv::destroyWindow( APRILTAGS_INTRUDE_DETECTOR_VIEW );
}
  
std::vector<AprilTags::TagDetection> AprilTagsIntrudeDetector::extract_tags( cv::Mat& image) {
   cv::Mat gray_img;
   cvtColor( image, gray_img, CV_BGR2GRAY );
   return mp_tag_detector->extractTags( gray_img );
}

bool AprilTagsIntrudeDetector::get_intrude(apriltags_intrude_detector::apriltags_intrude::Request& req,
                                           apriltags_intrude_detector::apriltags_intrude::Response& res) {
   res.id = get_tag_id( req.x, req.y );
   return true;
}

bool AprilTagsIntrudeDetector::get_info(apriltags_intrude_detector::apriltags_info::Request& req,
                                        apriltags_intrude_detector::apriltags_info::Response& res) {
  for( vector< pair<AprilTags::TagDetection, Polygon2D> >::iterator it = m_tags.begin();
       it != m_tags.end(); it ++ ) {
    pair<AprilTags::TagDetection, Polygon2D> data = (*it);
    geometry_msgs::Polygon poly;
    for(unsigned int i=0; i<4;i++) {
      geometry_msgs::Point32 p;
      p.x = data.first.p[i].first;
      p.y = data.first.p[i].second;
      p.z = 0.0;
      poly.points.push_back(p);
    }
    res.polygons.push_back(poly);
    res.ids.push_back(data.first.id);
  }
  return true; 
}

int AprilTagsIntrudeDetector::get_tag_id(int x, int y) {
  Point2D point( x, y);
  for( unsigned int i=0; i<m_tags.size(); i++ ) {
    if( CGAL::ON_UNBOUNDED_SIDE != m_tags[i].second.bounded_side( point) ) {
      return m_tags[i].first.id;
    }
  }
  return -1;
}

bool AprilTagsIntrudeDetector::update_tags( cv::Mat& image ) {

  m_tags.clear();
  vector<AprilTags::TagDetection> tags = extract_tags( image ); 
  for( unsigned int i=0; i<tags.size(); i++ ){
    AprilTags::TagDetection tag = tags[i];

    Polygon2D polygon;
    polygon.push_back( Point2D( tag.p[0].first, tag.p[0].second ) );
    polygon.push_back( Point2D( tag.p[1].first, tag.p[1].second ) );
    polygon.push_back( Point2D( tag.p[2].first, tag.p[2].second ) );
    polygon.push_back( Point2D( tag.p[3].first, tag.p[3].second ) );

    m_tags.push_back( make_pair( tag, polygon ) );    
  }
  return true;
}

void AprilTagsIntrudeDetector::visualize_tags( cv::Mat& image ) {
    
  for( unsigned int i=0; i < m_tags.size(); i++ ) {
    AprilTags::TagDetection tag = m_tags[i].first;
    line( image, Point( tag.p[0].first, tag.p[0].second ), Point( tag.p[1].first, tag.p[1].second ), Scalar(0,255,0), 2 );
    line( image, Point( tag.p[1].first, tag.p[1].second ), Point( tag.p[2].first, tag.p[2].second ), Scalar(0,255,0), 2 );
    line( image, Point( tag.p[2].first, tag.p[2].second ), Point( tag.p[3].first, tag.p[3].second ), Scalar(0,255,0), 2 );
    line( image, Point( tag.p[3].first, tag.p[3].second ), Point( tag.p[0].first, tag.p[0].second ), Scalar(0,255,0), 2 );
  }
  cv::imshow(APRILTAGS_INTRUDE_DETECTOR_VIEW, image );
}
