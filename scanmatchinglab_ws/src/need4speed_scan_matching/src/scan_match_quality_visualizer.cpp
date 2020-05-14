#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "need4speed_scan_matching/ScanMatchError.h"

class scanMatchErrorVisualizer
{
  private:
    double movingAveragePositionError;
    double movingAverageAngleError;
    visualization_msgs::Marker scanMatchErrorMarker;

    ros::NodeHandle nh;
    ros::Subscriber error_sub;
    ros::Publisher error_viz_pub;

    void errorCallback(const need4speed_scan_matching::ScanMatchError::ConstPtr& msg);

  public:
    scanMatchErrorVisualizer();
    void publishErrorVisualization();
};

scanMatchErrorVisualizer::scanMatchErrorVisualizer() :
  nh(ros::NodeHandle()),
  error_sub(nh.subscribe("scan_match_quality", 100, &scanMatchErrorVisualizer::errorCallback, this))
{
  this->error_viz_pub = this->nh.advertise<visualization_msgs::Marker>("scan_match_error_visualization", 1);
  this->movingAverageAngleError = 0.0;
  this->movingAveragePositionError = 0.0;

  // Initialize all the markers with their positions so we only need to
  // change the text during callbacks
  this->scanMatchErrorMarker = visualization_msgs::Marker();
  this->scanMatchErrorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->scanMatchErrorMarker.scale.z = 0.4;
  this->scanMatchErrorMarker.color.a = 1;
  this->scanMatchErrorMarker.header.frame_id = "/laser";
  this->scanMatchErrorMarker.pose.position.y = -3.0;
  this->scanMatchErrorMarker.pose.position.x = 2.4;
}

void scanMatchErrorVisualizer::errorCallback(const need4speed_scan_matching::ScanMatchError::ConstPtr& msg)
{
  this->movingAveragePositionError = msg->average_squared_position_error;
  this->movingAverageAngleError = msg->average_squared_heading_error;
}

void scanMatchErrorVisualizer::publishErrorVisualization()
{

  // error state text output
  this->scanMatchErrorMarker.text = "Avg sqd position error: " + std::to_string(this->movingAveragePositionError) + "\nAvg sqd angle error: " + std::to_string(this->movingAverageAngleError);
  this->error_viz_pub.publish(this->scanMatchErrorMarker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "scan_match_quality_visualizer");
  scanMatchErrorVisualizer scan_match_quality_visualizer;
  ros::Rate rate(10);
  while(ros::ok())
  {
    scan_match_quality_visualizer.publishErrorVisualization();
    ros::spinOnce();
    rate.sleep();
  }
}

