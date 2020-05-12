#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <need4speed_scan_matching/ScanMatchError.h>
#include <vector>
#include <tf/transform_datatypes.h>

class matchErrorTracker
{
  private:
    ros::NodeHandle nh;
    //Topic names for publishers and subscribers
    const std::string& POSE_TOPIC = "/scan_match_location";
    const std::string& ERROR_TOPIC = "/scan_match_error";
    
    // Variables to handle sliding window
    int window_size;
    int window_idx;
    std::vector<double> angle_error;
    std::vector<double> position_error;
    int window_n;

    //Publisher
    ros::Publisher error_pub;

    //Subscriber
    ros::Subscriber pose_sub;

    //TF2 Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
  public:
    matchErrorTracker();
    ~matchErrorTracker();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void publishError();
};

matchErrorTracker::matchErrorTracker() :
  nh(ros::NodeHandle()), error_pub(nh.advertise<need4speed_scan_matching::ScanMatchError>(ERROR_TOPIC, 10)),
  pose_sub(nh.subscribe(POSE_TOPIC, 10, &matchErrorTracker::poseCallback, this))
{
  ros::param::get("/scan_match_quality_node/window_size",this->window_size);
  this->window_idx = 0;
  for(int i{0}; i < this->window_size; i++)
  {
    this->angle_error.push_back(0.0);
    this->position_error.push_back(0.0);
  }
  this->window_n = 0;
  this->tfListener = new tf2_ros::TransformListener(this->tfBuffer);
}

matchErrorTracker::~matchErrorTracker()
{
  delete this->tfListener;
}

void matchErrorTracker::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // Get the latest transform from map to base_link
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  // Extract X,Y,theta from the true position and scan match estimate
  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(transformStamped.transform.rotation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  double theta = yaw;
  ROS_INFO("True transform: %f %f %f", x, y, theta);

  double x_est = msg->pose.position.x;
  double y_est = msg->pose.position.y;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  double theta_est = yaw;
  ROS_INFO("Estimated transform: %f %f %f", x_est, y_est, theta_est);
  
  // Increase window count until max
  if(this->window_n < this->window_size)
    window_n++;

  //Save squared error in buffer
  this->angle_error[this->window_idx] = (theta_est - theta)*(theta_est - theta);
  this->position_error[this->window_idx] = (x_est - x)*(x_est - x) + (y_est - y)*(y_est - y);
  // Increment the window index for next save
  this->window_idx++;
  if(this->window_idx >= this->window_size)
    this->window_idx = 0;

  this->publishError();
}

void matchErrorTracker::publishError()
{
  double sum_angle_error{0.0};
  double sum_position_error{0.0};
  for(int i{0}; i < this->window_n; i++)
  {
    sum_angle_error += this->angle_error[i];
    sum_position_error += this->position_error[i];
  }
  need4speed_scan_matching::ScanMatchError msg;
  msg.average_squared_position_error = sum_position_error/this->window_n;
  msg.average_squared_heading_error = sum_angle_error/this->window_n;
  this->error_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_match_quality");
  matchErrorTracker error_tracker;
  ros::spin();
  return 0;
}
