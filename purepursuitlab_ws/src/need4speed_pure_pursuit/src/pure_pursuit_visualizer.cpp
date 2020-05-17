#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"
#include "race/drive_param.h"

class purePursuitVisualizer
{
  private:
    double lookaheadDistance;
    double velocity;
    visualization_msgs::Marker purePursuitMarker;

    ros::NodeHandle nh;
    ros::Subscriber lookahead_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher viz_pub;

    void lookaheadCallback(const std_msgs::Float64::ConstPtr& msg);
    void velocityCallback(const race::drive_param::ConstPtr& msg);

  public:
    purePursuitVisualizer();
    void publishVisualization();
};

purePursuitVisualizer::purePursuitVisualizer() :
  nh(ros::NodeHandle()),
  lookahead_sub(nh.subscribe("lookahead_distance", 100, &purePursuitVisualizer::lookaheadCallback, this)),
  velocity_sub(nh.subscribe("drive_parameters", 100, &purePursuitVisualizer::velocityCallback, this))
{
  this->viz_pub = this->nh.advertise<visualization_msgs::Marker>("pure_pursuit_visualization", 1);
  this->lookaheadDistance = 0.0;
  this->velocity = 0.0;

  // Initialize all the markers with their positions so we only need to
  // change the text during callbacks
  this->purePursuitMarker = visualization_msgs::Marker();
  this->purePursuitMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->purePursuitMarker.scale.z = 0.4;
  this->purePursuitMarker.color.a = 1;
  this->purePursuitMarker.header.frame_id = "/laser";
  this->purePursuitMarker.pose.position.y = -3.0;
  this->purePursuitMarker.pose.position.x = 0.0;
}

void purePursuitVisualizer::lookaheadCallback(const std_msgs::Float64::ConstPtr& msg)
{
  this->lookaheadDistance = msg->data;
}

void purePursuitVisualizer::velocityCallback(const race::drive_param::ConstPtr& msg)
{
  this->velocity = msg->velocity;
}

void purePursuitVisualizer::publishVisualization()
{

  // error state text output
  this->purePursuitMarker.text = "Lookahead distance: " + std::to_string(this->lookaheadDistance) + "\nVelocity: " + std::to_string(this->velocity);
  this->viz_pub.publish(this->purePursuitMarker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pure_pursuit_visualizer");
  purePursuitVisualizer visualizer;
  ros::Rate rate(10);
  while(ros::ok())
  {
    visualizer.publishVisualization();
    ros::spinOnce();
    rate.sleep();
  }
}

