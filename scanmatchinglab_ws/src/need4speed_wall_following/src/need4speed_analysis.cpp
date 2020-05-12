#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "need4speed_wall_following/ErrorAnalysis.h"
#include <cmath>

class dataAnalyzer
{
  private:
    long double running_sum;
    unsigned long long int running_count;
    double running_max;
    ros::NodeHandle node_handle;
    ros::Subscriber error_sub;
    ros::Publisher pub;
  public:
    dataAnalyzer();
    void callback(const std_msgs::Float64::ConstPtr& msg);
};

dataAnalyzer::dataAnalyzer() :
    node_handle(ros::NodeHandle()),
    error_sub(node_handle.subscribe("pid_error", 100, &dataAnalyzer::callback, this)),
    pub(node_handle.advertise<need4speed_wall_following::ErrorAnalysis>("wall_following_analysis", 100))
{
  this->running_sum = 0.0;
  this->running_count = 0;
  this->running_max = 0.0;
}

void dataAnalyzer::callback(const std_msgs::Float64::ConstPtr& msg)
{
  // Add new value to running sum and increment number of messages received
  this->running_sum += std::abs(msg->data);
  this->running_count++;
  // Check on status of absolute max
  if(std::abs(msg->data) > this->running_max)
    this->running_max = std::abs(msg->data);

  // Now package new data into the outgoing message and publish
  need4speed_wall_following::ErrorAnalysis analysis_msg;
  analysis_msg.running_avg_abs_error.data = this->running_sum/this->running_count;
  analysis_msg.running_max_abs_error.data = this->running_max;
  this->pub.publish(analysis_msg);
}


int main(int argc, char **argv)
{
  // Start an analysis node
  ros::init(argc, argv, "brock_analysis");
  dataAnalyzer data_analyzer;

  // Spin to let the algorithm run until ROS shut down
  ros::spin();

  return 0;
}
