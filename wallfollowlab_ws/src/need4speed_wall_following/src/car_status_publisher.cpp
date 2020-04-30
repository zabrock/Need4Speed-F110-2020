#include "ros/ros.h"
#include <string>
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "need4speed_wall_following/ErrorAnalysis.h"
#include "need4speed_wall_following/TurnState.h"

class carStatusVisualizer
{
  private:
    std::string wallFollowMode;
    bool turnInitiated;
    bool turnCompleted;
    double runningAverageError;
    double runningMaxError;
    visualization_msgs::Marker wallFollowModeMarker;
    visualization_msgs::Marker turnStateMarker;
    visualization_msgs::Marker errorStateMarker;

    ros::NodeHandle nh;
    ros::Subscriber error_sub;
    ros::Subscriber turn_state_sub;
    ros::Subscriber follow_mode_sub;
    ros::Publisher error_viz_pub;
    ros::Publisher turn_state_viz_pub;
    ros::Publisher follow_mode_viz_pub;

    void errorCallback(const need4speed_wall_following::ErrorAnalysis::ConstPtr& msg);
    void turnStateCallback(const need4speed_wall_following::TurnState::ConstPtr& msg);
    void followModeCallback(const std_msgs::String::ConstPtr& msg);

  public:
    carStatusVisualizer();
    void publishCarStatusVisualization();
};

carStatusVisualizer::carStatusVisualizer() :
  nh(ros::NodeHandle()),
  error_sub(nh.subscribe("wall_following_analysis", 100, &carStatusVisualizer::errorCallback, this)),
  turn_state_sub(nh.subscribe("car_turn_state", 100, &carStatusVisualizer::turnStateCallback, this)),
  follow_mode_sub(nh.subscribe("car_follow_mode", 100, &carStatusVisualizer::followModeCallback, this))
{
  this->error_viz_pub = this->nh.advertise<visualization_msgs::Marker>("error_visualization", 1);
  this->turn_state_viz_pub = this->nh.advertise<visualization_msgs::Marker>("turn_state_visualization", 1);
  this->follow_mode_viz_pub = this->nh.advertise<visualization_msgs::Marker>("follow_mode_visualization", 1);
  this->wallFollowMode = "";
  this->turnInitiated = false;
  this->turnCompleted = false;
  this->runningAverageError = 0.0;
  this->runningMaxError = 0.0;

  this->wallFollowModeMarker = visualization_msgs::Marker();
  this->wallFollowModeMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->wallFollowModeMarker.scale.z = 0.4;
  this->wallFollowModeMarker.color.a = 1;
  this->wallFollowModeMarker.header.frame_id = "/laser";
  this->wallFollowModeMarker.pose.position.y = -3.0;
  this->wallFollowModeMarker.pose.position.x = 0.0;
  this->turnStateMarker = visualization_msgs::Marker();
  this->turnStateMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->turnStateMarker.scale.z = 0.4;
  this->turnStateMarker.color.a = 1;
  this->turnStateMarker.pose.position.y = -3.0;
  this->turnStateMarker.pose.position.x = -1.2;
  this->turnStateMarker.header.frame_id = "/laser";
  this->errorStateMarker = visualization_msgs::Marker();
  this->errorStateMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->errorStateMarker.scale.z = 0.4;
  this->errorStateMarker.color.a = 1;
  this->errorStateMarker.header.frame_id = "/laser";
  this->errorStateMarker.pose.position.y = -3.0;
  this->errorStateMarker.pose.position.x = 1.2;
}

void carStatusVisualizer::errorCallback(const need4speed_wall_following::ErrorAnalysis::ConstPtr& msg)
{
  this->runningAverageError = msg->running_avg_abs_error.data;
  this->runningMaxError = msg->running_max_abs_error.data;
}

void carStatusVisualizer::turnStateCallback(const need4speed_wall_following::TurnState::ConstPtr& msg)
{
  this->turnInitiated = msg->turn_initiated;
  this->turnCompleted = msg->turn_completed;
}

void carStatusVisualizer::followModeCallback(const std_msgs::String::ConstPtr& msg)
{
  this->wallFollowMode = msg->data;
}

void carStatusVisualizer::publishCarStatusVisualization()
{
  // turn state text output
  std::string turn_initiated;
  std::string turn_completed;
  if(this->turnInitiated)
    turn_initiated = "true";
  else
    turn_initiated = "false";
  if(this->turnCompleted)
    turn_completed = "true";
  else
    turn_completed = "false";
  this->turnStateMarker.text = "turn_initiated: " + turn_initiated + "\nturn_completed: " + turn_completed;
  this->turn_state_viz_pub.publish(this->turnStateMarker);
  
  // wall follow mode output
  this->wallFollowModeMarker.text = "Wall follow mode: " + this->wallFollowMode;
  this->follow_mode_viz_pub.publish(this->wallFollowModeMarker);

  // error state text output
  this->errorStateMarker.text = "Running average error: " + std::to_string(this->runningAverageError) + "\nRunning max error: " + std::to_string(this->runningMaxError);
  this->error_viz_pub.publish(this->errorStateMarker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "car_state_visualizer");
  carStatusVisualizer car_status_visualizer;
  ros::Rate rate(10);
  while(ros::ok())
  {
    car_status_visualizer.publishCarStatusVisualization();
    ros::spinOnce();
    rate.sleep();
  }
}

