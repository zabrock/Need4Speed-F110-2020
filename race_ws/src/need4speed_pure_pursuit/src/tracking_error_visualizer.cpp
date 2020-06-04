#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"
#include "race/drive_param.h"

class trackingErrorVisualizer
{
  private:
    double maxError;
    double sqdAverageError;
    visualization_msgs::Marker errorMarker;

    ros::NodeHandle nh;
    ros::Subscriber max_error_sub;
    ros::Subscriber sqd_error_sub;
    ros::Publisher viz_pub;

    void maxErrorCallback(const std_msgs::Float64::ConstPtr& msg);
    void sqdErrorCallback(const std_msgs::Float64::ConstPtr& msg);

  public:
    trackingErrorVisualizer();
    void publishVisualization();
};

trackingErrorVisualizer::trackingErrorVisualizer() :
  nh(ros::NodeHandle()),
  max_error_sub(nh.subscribe("max_tracking_error", 100, &trackingErrorVisualizer::maxErrorCallback, this)),
  sqd_error_sub(nh.subscribe("squared_average_tracking_error", 100, &trackingErrorVisualizer::sqdErrorCallback, this))
{
  this->viz_pub = this->nh.advertise<visualization_msgs::Marker>("tracking_error_visualization", 1);
  this->maxError = 0.0;
  this->sqdAverageError = 0.0;

  // Initialize all the markers with their positions so we only need to
  // change the text during callbacks
  this->errorMarker = visualization_msgs::Marker();
  this->errorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->errorMarker.scale.z = 0.4;
  this->errorMarker.color.a = 1;
  this->errorMarker.header.frame_id = "/laser";
  this->errorMarker.pose.position.y = 3.0;
  this->errorMarker.pose.position.x = 0.0;
}

void trackingErrorVisualizer::maxErrorCallback(const std_msgs::Float64::ConstPtr& msg)
{
  this->maxError = msg->data;
}

void trackingErrorVisualizer::sqdErrorCallback(const std_msgs::Float64::ConstPtr& msg)
{
  this->sqdAverageError = msg->data;
}

void trackingErrorVisualizer::publishVisualization()
{

  // error state text output
  this->errorMarker.text = "Max Tracking Error: " + std::to_string(this->maxError) + "\nAverage Squared Error: " + std::to_string(this->sqdAverageError);
  this->viz_pub.publish(this->errorMarker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "tracking_error_visualizer");
  trackingErrorVisualizer visualizer;
  ros::Rate rate(10);
  while(ros::ok())
  {
    visualizer.publishVisualization();
    ros::spinOnce();
    rate.sleep();
  }
}

