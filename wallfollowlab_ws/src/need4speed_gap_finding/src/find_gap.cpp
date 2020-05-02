#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "need4speed_gap_finding/gaps.h"
#include <math.h>
#include <vector>
#include <utility> // Used for std::pair
#include <algorithm> // Used for std::max and std::min
#include <string>

class gapFinder
{
  private:
    double min_acceptable_gap_width;	// Minimum acceptable gap distance suitable for publishing
    double min_acceptable_distance; 	// Minimum acceptable distance to a potential gap
    double max_acceptable_distance; 	// Maximum acceptable distance to a potential gap
    double truncated_lidar_angle;	// Angular range in front of vehicle to take into account
    double max_adjacent_pct_diff;	// Comparative value to determine if lidar ranges are adjacent/part of same gap
    double truncated_start_angle;	// Tracks start angle of truncated angular range
    double angle_increment;		// Tracks how far apart angular measurements are in lidar data
    size_t truncated_start_idx;		// Tracks start index of truncated lidar range
    size_t truncated_end_idx;		// Tracks end index of truncated lidar range
    bool truncated{false};		// Tracks whether truncated indices have been calculated yet
    ros::NodeHandle node_handle;
    ros::Subscriber lidar_sub;
    ros::Publisher gaps_pub;
    ros::Publisher largest_gap_pub;
  public:
    gapFinder();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    std::vector<double> preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    need4speed_gap_finding::gaps get_lidar_gaps(const std::vector<double>& lidar_ranges);
    need4speed_gap_finding::gaps get_turn_gaps(const std::vector<double>& lidar_ranges);
    std::pair<geometry_msgs::Vector3,std_msgs::Float64> calculate_gap(const std::vector<double>& lidar_ranges, const size_t& start_idx, const size_t& end_idx);
    std::pair<geometry_msgs::Vector3, std_msgs::Float64> calculate_turn_gap(const std::vector<double>& lidar_ranges, const size_t& start_idx);
    std::vector<double> filter_ranges(const std::vector<double>& lidar_ranges);
    
};

gapFinder::gapFinder() :
    node_handle(ros::NodeHandle()),
    lidar_sub(node_handle.subscribe("scan", 100, &gapFinder::scanCallback, this)),
    gaps_pub(node_handle.advertise<need4speed_gap_finding::gaps>("lidar_gaps", 100)),
    largest_gap_pub(node_handle.advertise<geometry_msgs::Vector3>("gap_center",100))
{
  ros::param::get("/gap_finding_node/max_acceptable_scan_distance",this->max_acceptable_distance);
  ros::param::get("/gap_finding_node/min_acceptable_scan_distance",this->min_acceptable_distance);
  ros::param::get("/gap_finding_node/truncated_lidar_angle",this->truncated_lidar_angle);
  ros::param::get("/gap_finding_node/max_adjacent_scan_percentage_difference",this->max_adjacent_pct_diff);
  ros::param::get("/gap_finding_node/min_acceptable_gap_width",this->min_acceptable_gap_width);
}

void gapFinder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!this->truncated)
  {
    // Define the range of indices of the lidar scan we're interested in
    // and save the corresponding start angle and angle increment between ranges
    const auto truncated_range_size = static_cast<size_t>((this->truncated_lidar_angle/(msg->angle_max - msg->angle_min))*msg->ranges.size());
    this->truncated_start_idx = (msg->ranges.size()/2) - (truncated_range_size/2);
    this->truncated_end_idx = (msg->ranges.size()/2) + (truncated_range_size/2);
    this->truncated_start_angle = msg->angle_min + this->truncated_start_idx*msg->angle_increment;
    this->angle_increment = msg->angle_increment;
    this->truncated = true;
    ROS_INFO("truncated scan start angle = %f", this->truncated_start_angle);
    ROS_INFO("truncated scan start idx = %zu", this->truncated_start_idx);
    ROS_INFO("truncated scan end idx = %zu", this->truncated_end_idx);
  }

  // Preprocess the lidar readings to reduce to angles we care about and filter out NaN/Inf values
  std::vector<double> preprocessed_ranges = this->preprocess_lidar_scan(msg);
  // Use the preprocessed readings to identify gaps
  need4speed_gap_finding::gaps gap_msg = this->get_turn_gaps(preprocessed_ranges);
  std_msgs::Header h;
  h.stamp = ros::Time::now();
  gap_msg.header = h;
  this->gaps_pub.publish(gap_msg);

  // Find the largest gap and publish it
  double max_width{0.0};
  geometry_msgs::Vector3 largest_gap_loc;
  for(size_t i{0}; i < gap_msg.gap_widths.size(); i++)
  {
    if(gap_msg.gap_widths[i].data > max_width)
    {
      largest_gap_loc = gap_msg.gap_centers[i];
      max_width = gap_msg.gap_widths[i].data;
    }
  }
  this->largest_gap_pub.publish(largest_gap_loc);
}

std::vector<double> gapFinder::preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<double> preprocessed_ranges;
  for(size_t i{this->truncated_start_idx}; i < this->truncated_end_idx; i++)
  {
    // Ignore NaN values and values below minimum acceptable 
    // (zeroes won't be taken into account as gaps)
    if(std::isnan(msg->ranges[i]) || msg->ranges[i] < this->min_acceptable_distance)
      preprocessed_ranges.push_back(0.0);
    // Ranges above defined maximum interesting value or Inf values
    // are truncated to within maximum range
    else if(msg->ranges[i] > this->max_acceptable_distance || std::isinf(msg->ranges[i]))
      preprocessed_ranges.push_back(this->max_acceptable_distance);
    // All other ranges filled in as received
    else
      preprocessed_ranges.push_back(msg->ranges[i]);
  }
  return preprocessed_ranges;
}

need4speed_gap_finding::gaps gapFinder::get_turn_gaps(const std::vector<double>& lidar_ranges)
{
  need4speed_gap_finding::gaps gaps;

  // Tracking variables for where gaps begin and end
  size_t current_start{0};
  size_t current_size{0};
  size_t current_idx{0};
  while(current_idx < lidar_ranges.size())
  {
    // Find discontinuities in scan based on where range is maximum
    if(lidar_ranges[current_idx] >= this->max_acceptable_distance)
    {
      current_start = current_idx;
      while(current_idx < lidar_ranges.size() && lidar_ranges[current_idx] >= this->max_acceptable_distance)
        current_idx++;

      if(current_idx > current_start)
      {
        // A gap has been found, so find where and how wide it is
        std::pair<geometry_msgs::Vector3, std_msgs::Float64> gap_info = this->calculate_gap(lidar_ranges, current_start, current_idx-1);
        if(gap_info.second.data > this->min_acceptable_gap_width)
        {
          gaps.gap_centers.push_back(gap_info.first);
          gaps.gap_widths.push_back(gap_info.second);
        }
      }
      // Check if the place after maximum range shifts down should also be considered
      // as a gap
      if(current_idx < lidar_ranges.size() && std::abs(lidar_ranges[current_idx-1] - lidar_ranges[current_idx]) > this->max_adjacent_pct_diff*lidar_ranges[current_idx-1])
      {
        // A gap has been found, so find where and how wide it is
        std::pair<geometry_msgs::Vector3, std_msgs::Float64> gap_info = this->calculate_turn_gap(lidar_ranges, current_idx);
        if(gap_info.second.data > this->min_acceptable_gap_width)
        {
          gaps.gap_centers.push_back(gap_info.first);
          gaps.gap_widths.push_back(gap_info.second);
        }
      }
    }
    else if(current_idx < lidar_ranges.size()-1 && std::abs(lidar_ranges[current_idx] - lidar_ranges[current_idx+1]) > this->max_adjacent_pct_diff*lidar_ranges[current_idx])
    {
      // A gap has been found, so find where and how wide it is
      std::pair<geometry_msgs::Vector3, std_msgs::Float64> gap_info = this->calculate_turn_gap(lidar_ranges, current_idx);
      if(gap_info.second.data > this->min_acceptable_gap_width)
      {
        gaps.gap_centers.push_back(gap_info.first);
        gaps.gap_widths.push_back(gap_info.second);
      }
    }
    current_idx++;
  }
  return gaps;
}

std::pair<geometry_msgs::Vector3, std_msgs::Float64> gapFinder::calculate_turn_gap(const std::vector<double>& lidar_ranges, const size_t& start_idx)
{
  double angle1 = this->truncated_start_angle + this->angle_increment*start_idx;
  double angle2 = angle1 + this->angle_increment;

  double x1 = lidar_ranges[start_idx]*std::cos(angle1);
  double x2 = lidar_ranges[start_idx+1]*std::cos(angle2);
  double y1 = lidar_ranges[start_idx]*std::sin(angle1);
  double y2 = lidar_ranges[start_idx+1]*std::sin(angle2);

  geometry_msgs::Vector3 gap_loc;
  std_msgs::Float64 gap_width_msg;
  
  gap_loc.x = (x1+x2)/2;
  gap_loc.y = (y1+y2)/2;
  gap_loc.z = 0.1;
  gap_width_msg.data = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
  return std::pair<geometry_msgs::Vector3, std_msgs::Float64>(gap_loc,gap_width_msg);
}

std::pair<geometry_msgs::Vector3, std_msgs::Float64> gapFinder::calculate_gap(const std::vector<double>& lidar_ranges, const size_t& start_idx, const size_t& end_idx)
{
  /* Method is based on trigonometry. First the width of the gap is the length of the line 
   * from the start to the endpoint of the gap, calculated using the law of cosines.
   * Then another angle in the triangle formed by this line and the rays to the start and 
   * endpoints is identified to calculate the length of the vector from the lidar to the midpoint 
   * of the previously identified line. This then allows the calculation of the angle to the
   * midpoint of the line from the minimum angle of the reduced lidar range, which allows
   * for the identification of the location of the gap in space.
   */

  geometry_msgs::Vector3 gap_loc;
  std_msgs::Float64 gap_width_msg;

  // Calculate angular measurement between the start and endpoint of the gap
  double gap_angle = this->angle_increment*(end_idx-start_idx);
  // Calculate width of the gap using Law of Cosines
  double a = lidar_ranges[start_idx];
  double b = lidar_ranges[end_idx];
  double gap_width = std::sqrt(a*a + b*b - 2*a*b*std::cos(gap_angle));
  // Calculate angle opposite side b using Law of Cosines
  double beta = std::acos(1/(2*a*gap_width)*(a*a + gap_width*gap_width - b*b));
  // Calculate length of line to midpoint of gap using Law of Cosines
  double m = std::sqrt(a*a + gap_width*gap_width/4 - a*gap_width*std::cos(beta));
  // Now calculate angle between start of gap and the midpoint ray of the gap
  double phi = std::acos((1/(2*a*m))*(a*a + m*m -gap_width*gap_width/4));
  // Find where this angle projects to the actual distance measured in lidar_ranges for determining gap location
  const auto gap_center_idx = static_cast<size_t>(phi/this->angle_increment) + start_idx;
  double gap_distance = lidar_ranges[gap_center_idx];
  double gap_center_heading = this->truncated_start_angle + this->angle_increment*gap_center_idx;
  gap_loc.x = gap_distance*std::cos(gap_center_heading);
  gap_loc.y = gap_distance*std::sin(gap_center_heading);
  gap_loc.z = 0.1; // Puts the Vector3 above the zero plane
  gap_width_msg.data = gap_width;

  return std::pair<geometry_msgs::Vector3, std_msgs::Float64>(gap_loc,gap_width_msg);
}

int main(int argc, char **argv)
{
  // Start a gap finding node
  ros::init(argc, argv, "gap_finder");
  gapFinder gap_finder;

  // Spin to let the algorithm run until ROS shut down
  ros::spin();

  return 0;
}
