#include "ros/ros.h"
#include <string>
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "need4speed_wall_following/ErrorAnalysis.h"
#include "need4speed_wall_following/TurnState.h"
#include "need4speed_wall_following/TurnsPossible.h"
#include "need4speed_wall_following/DriveCommand.h"

class carStatusVisualizer
{
  private:
    std::string wallFollowMode;
    bool turnInitiated;
    bool turnCompleted;
    double runningAverageError;
    double runningMaxError;
    int turnsPossible;
    std::string executingInstruction;
    visualization_msgs::Marker wallFollowModeMarker;
    visualization_msgs::Marker turnStateMarker;
    visualization_msgs::Marker errorStateMarker;
    visualization_msgs::Marker turnsPossibleMarker;
    visualization_msgs::Marker executingInstructionMarker;

    ros::NodeHandle nh;
    ros::Subscriber error_sub;
    ros::Subscriber turn_state_sub;
    ros::Subscriber follow_mode_sub;
    ros::Subscriber turn_possible_sub;
    ros::Subscriber executing_instruction_sub;
    ros::Publisher error_viz_pub;
    ros::Publisher turn_state_viz_pub;
    ros::Publisher follow_mode_viz_pub;
    ros::Publisher turns_possible_viz_pub;
    ros::Publisher executing_instruction_viz_pub;

    void errorCallback(const need4speed_wall_following::ErrorAnalysis::ConstPtr& msg);
    void turnStateCallback(const need4speed_wall_following::TurnState::ConstPtr& msg);
    void followModeCallback(const std_msgs::String::ConstPtr& msg);
    void turnsPossibleCallback(const need4speed_wall_following::TurnsPossible::ConstPtr& msg);
    void executingInstructionCallback(const need4speed_wall_following::DriveCommand::ConstPtr& msg);

  public:
    carStatusVisualizer();
    void publishCarStatusVisualization();
};

carStatusVisualizer::carStatusVisualizer() :
  nh(ros::NodeHandle()),
  error_sub(nh.subscribe("wall_following_analysis", 100, &carStatusVisualizer::errorCallback, this)),
  turn_state_sub(nh.subscribe("car_turn_state", 100, &carStatusVisualizer::turnStateCallback, this)),
  follow_mode_sub(nh.subscribe("car_follow_mode", 100, &carStatusVisualizer::followModeCallback, this)),
  turn_possible_sub(nh.subscribe("turns_possible", 100, &carStatusVisualizer::turnsPossibleCallback, this)),
  executing_instruction_sub(nh.subscribe("executing_instruction", 100, &carStatusVisualizer::executingInstructionCallback, this))
{
  this->error_viz_pub = this->nh.advertise<visualization_msgs::Marker>("error_visualization", 1);
  this->turn_state_viz_pub = this->nh.advertise<visualization_msgs::Marker>("turn_state_visualization", 1);
  this->follow_mode_viz_pub = this->nh.advertise<visualization_msgs::Marker>("follow_mode_visualization", 1);
  this->turns_possible_viz_pub = this->nh.advertise<visualization_msgs::Marker>("turns_possible_visualization", 1);
  this->executing_instruction_viz_pub = this->nh.advertise<visualization_msgs::Marker>("executing_instruction_visualization", 1);
  this->wallFollowMode = "";
  this->turnInitiated = false;
  this->turnCompleted = false;
  this->runningAverageError = 0.0;
  this->runningMaxError = 0.0;
  this->turnsPossible = 0;
  this->executingInstruction = "n/a";

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
  this->turnsPossibleMarker = visualization_msgs::Marker();
  this->turnsPossibleMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->turnsPossibleMarker.scale.z = 0.4;
  this->turnsPossibleMarker.color.a = 1;
  this->turnsPossibleMarker.header.frame_id = "/laser";
  this->turnsPossibleMarker.pose.position.y = 3.0;
  this->turnsPossibleMarker.pose.position.x = 0.0;
  this->executingInstructionMarker = visualization_msgs::Marker();
  this->executingInstructionMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  this->executingInstructionMarker.scale.z = 0.4;
  this->executingInstructionMarker.color.a = 1;
  this->executingInstructionMarker.header.frame_id = "/laser";
  this->executingInstructionMarker.pose.position.y = 3.0;
  this->executingInstructionMarker.pose.position.x = 1.2;
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

void carStatusVisualizer::turnsPossibleCallback(const need4speed_wall_following::TurnsPossible::ConstPtr& msg)
{
  this->turnsPossible = msg->turns_possible;
}

void carStatusVisualizer::executingInstructionCallback(const need4speed_wall_following::DriveCommand::ConstPtr& msg)
{
  if(msg->follow_method != need4speed_wall_following::DriveCommand::EMPTY_FOLLOW_METHOD && msg->velocity != need4speed_wall_following::DriveCommand::EMPTY_VELOCITY)
    this->executingInstruction = msg->follow_method + " " + std::to_string(msg->velocity);
  else
    this->executingInstruction = "n/a";
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

  
  // turns possible text output
  std::string turns_possible{"error"};
  if(this->turnsPossible == need4speed_wall_following::TurnsPossible::NONE)
    turns_possible = "none";
  else if(this->turnsPossible == need4speed_wall_following::TurnsPossible::LEFT)
    turns_possible = "left";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::RIGHT)
    turns_possible = "right";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::CENTER)
    turns_possible = "center";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::LEFT_AND_CENTER)
    turns_possible = "left and center";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::CENTER_AND_RIGHT)
    turns_possible = "center and right";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::LEFT_AND_RIGHT)
    turns_possible = "left and right";
  else if (this->turnsPossible == need4speed_wall_following::TurnsPossible::LEFT_AND_CENTER_AND_RIGHT)
    turns_possible = "left, center, and right";
  this->turnsPossibleMarker.text = "Routes possible: " + turns_possible;
  this->turns_possible_viz_pub.publish(this->turnsPossibleMarker);

  // executing instruction text output
  this->executingInstructionMarker.text = "Executing instruction: " + this->executingInstruction;
  this->executing_instruction_viz_pub.publish(this->executingInstructionMarker);
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

