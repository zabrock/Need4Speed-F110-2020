#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Vector3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <utility>
#include <vector>
#include <random>
#include <limits>
#include <cmath>

// Constant parameters
int ITER_LIMIT = 2000;		    // Limit on number of iterations for RRT solver
double GOAL_RADIUS = 0.05; 	// Radius around goal acceptable for completion
double NEAR_RADIUS = 0.5;   // Radius around new node to search for rewiring candidates
double STEER_DELTA = 0.1;   // Maximum distance for steering toward random nodes
double GOAL_SAMPLE_RATE = 0.1;  // Rate at which goal is sampled in random node generation
double INFLATION_DISTANCE = 1.0;// Distance by which obstacles will be inflated
double INFLATION_RATE = 0.95;    // Percentage of neighboring obstacle value that will be 
                                // placed in the inflating cell
double GAMMA = 0.5;         // RRTStar specific constant for near radius calculation
int OBSTACLE_THRESHOLD = 75; // Threshold for determining whether cost of new node is too high

// Returns the Euclidean distance between two points.
double euclidean_distance(double x1, double y1, double x2, double y2)
{
  double dist_sqd = std::pow(x1-x2,2) + std::pow(y1-y2,2);
  return std::pow(dist_sqd,0.5);
}

// Node class for building RRT's.
class Node {
  private:
    double X;
    double Y;
    Node* parent;
    double cost;

  public:
    Node()
    {
      this->X = 0;
      this->Y = 0;
      this->parent = nullptr;
      this->cost = 0;
    }

    Node(double x, double y, Node* parent, double cost)
    {
      this->X = x;
      this->Y = y;
      this->parent = parent;
      this->cost = cost;
    }
    
    Node(Node* copy_node)
    {
      this->X = copy_node->getX();
      this->Y = copy_node->getY();
      this->parent = copy_node->getParent();
      this->cost = copy_node->getCost();
    }

    void setX(double x) {this->X = x;}
    void setY(double y) {this->Y = y;}
    void setParent(Node* parent){this->parent = parent;}
    void setCost(double cost){this->cost = cost;}

    double getX() {return this->X;}
    double getY() {return this->Y;}
    Node* getParent() {return this->parent;}
    double getCost() {return this->cost;}
};

class RRTstar {
  private:
    Node* root;
    Node* goal;
    double goalSampleRate;
    nav_msgs::MapMetaData mapInfo;
    std::vector<int8_t> mapData;
    std::vector<Node*> nodeList;
    bool mapReceived;

    // For random node generation
    std::default_random_engine generator;
    
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Publisher path_pub;
    ros::Publisher map_pub;

  public:
    RRTstar() {};

    RRTstar(ros::NodeHandle node_handle, double goal_sample_rate) :
      nh(node_handle),
      map_sub(nh.subscribe("/map", 100, &RRTstar::mapCallback, this))
    {
      this->root = nullptr;
      this->goal = nullptr;
      this->goalSampleRate = goal_sample_rate;
      this->x_min = 0;
      this->x_max = 0;
      this->y_min = 0;
      this->y_max = 0;
      
      this->path_pub = this->nh.advertise<nav_msgs::Path>("rrt_path", 10);
      this->map_pub = this->nh.advertise<nav_msgs::OccupancyGrid>("fake_map", 10);
    }
    
    virtual ~RRTstar()
    {
      this->resetTree();
    }

    void resetTree()
    {
      // Clear out the tree and goal node
      std::cout << "Tree size: " << this->nodeList.size() << std::endl;
      for(int i{0}; i < this->nodeList.size(); i++)
      {
        delete this->nodeList[i];
      }
      this->nodeList.clear();
      if (this->goal != nullptr)
        delete this->goal;
      this->root = nullptr;
      this->goal = nullptr;
    }

    void BuildRRT(double x_root, double y_root, double x_goal, double y_goal)
    {
      if(this->mapData.size() == 0)
      {
        // Wait until the map has been read in
        std::cout << "Skipping" << std::endl;
        return;
      }
      // Create root and goal nodes
      this->root = new Node(x_root, y_root, nullptr, 0);
      this->goal = new Node(x_goal, y_goal, nullptr, 0);
      // Add root to node list
      this->nodeList.push_back(this->root);
      
      bool foundPath = false;
      int best_node_to_goal_idx = -1;
      for(int i{0}; i < ITER_LIMIT; i++)
      {
        // Start by finding a random node in the space or sampling the goal
        std::uniform_real_distribution<double> distribution;
        double rand = distribution(this->generator);
        Node* random_node = new Node();
        if(rand < this->goalSampleRate)
        {
          random_node->setX(this->goal->getX());
          random_node->setY(this->goal->getY());
        }
        else
        {
          this->getPseudoRandomNode(random_node, x_root, y_root, x_goal, y_goal);
        }
        
        // Now steer toward the random node from the closest node
        int nearest_idx = this->getNearestNodeIndex(random_node);
        this->steer(random_node, nearest_idx);
        
        // Check if new node and path to it are obstacle free
        if(this->obstacleFree(random_node))
        {
          // Find the set of near nodes
          std::vector<int> near_indices = this->getNearNodeIndices(random_node);
          // Choose which of the near nodes should be the parent based
          // on cost (parent and cost are set here)
          this->chooseParent(random_node, near_indices);
          // Add to the tree
          this->nodeList.push_back(random_node);
          // Rewire as necessary
          this->rewire(random_node, near_indices);
          // Check if the new node gets us close enough to the goal
          if(this->closeToGoal(random_node))
          {
            // Check if this new node is better than a previous one
            // based on cost
            foundPath = true;
            if(best_node_to_goal_idx == -1 or this->nodeList[best_node_to_goal_idx]->getCost() > random_node->getCost())
            {
              // Set the best node to be the new node just added
              best_node_to_goal_idx = this->nodeList.size()-1;
            }
          }
        }
        else
        {
          // If not obstacle free, delete the new node
          delete random_node;
        }
      }
      // Once we're out of this loop, if we've found a path to the goal, return that path
      Node* temp_node;
      if(foundPath)
      {
        std::cout << "Path found" << std::endl;
        // Best node near goal has already been indicated
        temp_node = this->nodeList[best_node_to_goal_idx];
      }
      else
      {
        // If we didn't find a path to the goal, return the path that gets us to the 
        // node closest to the goal
        temp_node = this->nodeList[getNearestNodeIndex(this->goal)];
      }
      nav_msgs::Path path;
      path.header.frame_id = "/map";
      geometry_msgs::PoseStamped pose;
      while(temp_node != nullptr)
      {
        pose = geometry_msgs::PoseStamped();
        pose.pose.position.x = temp_node->getX();
        pose.pose.position.y = temp_node->getY();
        path.poses.insert(path.poses.begin(),pose);
        temp_node = temp_node->getParent();
      }
      this->path_pub.publish(path);
      this->resetTree();
      
    }
    
    std::vector<int> getNearNodeIndices(Node* new_node)
    {
      // Go through the list of nodes in the tree and determine which
      // are within the near search radius
      std::vector<int> near_indices;
      for(int i{0}; i < this->nodeList.size(); i++)
      {
        if(euclidean_distance(new_node->getX(), new_node->getY(), this->nodeList[i]->getX(), this->nodeList[i]->getY()) < NEAR_RADIUS)
        {
          near_indices.push_back(i);
        }
      }
      
      return near_indices;
    }
    
    void chooseParent(Node* new_node, std::vector<int> near_indices)
    {
      // Find which of the near nodes has the least cost to get to the new node
      double min_cost = std::numeric_limits<double>::max();
      int parent_idx{-1};
      for(int i{0}; i < near_indices.size(); i++)
      {
        //double cost = this->nodeList[near_indices[i]]->getCost() + euclidean_distance(new_node->getX(), new_node->getY(), this->nodeList[near_indices[i]]->getX(), this->nodeList[near_indices[i]]->getY());
        double cost = this->nodeList[near_indices[i]]->getCost() + edgeCost(new_node, this->nodeList[near_indices[i]]);
        if(cost < min_cost)
        {
          min_cost = cost;
          parent_idx = near_indices[i];
        }
      }
      // Now set the cost and parent for the new node
      new_node->setCost(min_cost);
      new_node->setParent(this->nodeList[parent_idx]);
    }
    
    double edgeCost(Node* new_node, Node* near_node)
    {
      // Determines the cost between new_node and near_node as the 
      // sum of the euclidean distance with the "integrated" cost through the cost map.
      
      // Start with euclidean distance cost
      double cost = euclidean_distance(new_node->getX(), new_node->getY(), near_node->getX(), near_node->getY());
      // Now add the "integrated" cost through the cost map; this is approximated
      // by averaging the cost at the two nodes in the cost map and multiplying by the
      // distance between them
      cost += cost*(this->mapData[getMapIndex(new_node->getX(),new_node->getY())]+this->mapData[getMapIndex(near_node->getX(),near_node->getY())])/2.0;
      
      return cost;
      
    }
    
    void rewire(Node* new_node, std::vector<int> near_indices)
    {
      // Go through all the near nodes and see if their cost would be reduced
      // by going through the new node
      for(int i{0}; i < near_indices.size(); i++)
      {
        //double new_cost = euclidean_distance(new_node->getX(), new_node->getY(), this->nodeList[near_indices[i]]->getX(), this->nodeList[near_indices[i]]->getY());
        double new_cost = edgeCost(new_node, this->nodeList[near_indices[i]]);
        if(new_node->getCost() + new_cost < this->nodeList[near_indices[i]]->getCost())
        {
          // Reset the neighbor node's parent to be the new node and update the cost
          this->nodeList[near_indices[i]]->setParent(new_node);
          this->nodeList[near_indices[i]]->setCost(new_node->getCost() + new_cost);
        }
      }
    }
    
    bool closeToGoal(Node* new_node)
    {
      // Get distance from the new_node to the goal node and check if it's within goal
      // radius
      double dist = euclidean_distance(new_node->getX(), new_node->getY(), this->goal->getX(), this->goal->getY());
      if(dist < GOAL_RADIUS)
        return true;
      else
        return false;
    }

    void getRandomNode(Node* rand)
    {
      std::uniform_real_distribution<double> distribution;
      rand->setX(this->x_min + distribution(this->generator)*(this->x_max-this->x_min));
      rand->setY(this->y_min + distribution(this->generator)*(this->y_max-this->y_min));
    }

    void getPseudoRandomNode(Node* rand, double x_root, double y_root, double x_goal, double y_goal)
    {
      std::uniform_real_distribution<double> distribution;
      // Find center between root and goal
      double x_ctr = (x_root + x_goal)/2.0;
      double y_ctr = (y_root + y_goal)/2.0;
      // Take radius of search area to be the distance between the root and goal so 
      // that we search a circle of diameter 2*distance centered between the root and goal
      double search_radius = euclidean_distance(x_root, y_root, x_goal, y_goal);
      // Now randomize heading and sub-radius within the search circle to find a new 
      // point in the smaller search area
      double heading = distribution(this->generator)*2*M_PI;
      double radius = distribution(this->generator)*search_radius;
      double x_new = x_ctr + radius*std::cos(heading);
      double y_new = y_ctr + radius*std::sin(heading);
      
      rand->setX(x_new);
      rand->setY(y_new);
      
    }

    int getNearestNodeIndex(Node* rand)
    {
      if(this->nodeList.empty())
        return -1;

      double min_dist_sqd = std::numeric_limits<double>::max();
      double dist_sqd{};
      int nearest_node_idx = -1;
      for(int i{0}; i < nodeList.size(); i++)
      {
        dist_sqd = std::pow(rand->getX()-nodeList[i]->getX(),2) + std::pow(rand->getY()-nodeList[i]->getY(),2);
        if(dist_sqd < min_dist_sqd)
        {
          min_dist_sqd = dist_sqd;
          nearest_node_idx = i;
        }
      }
      return nearest_node_idx;
    }
    
    void steer(Node* rand, int near_idx)
    {
      // Find distance between nearest and random node
      double dist = euclidean_distance(rand->getX(), rand->getY(), nodeList[near_idx]->getX(), nodeList[near_idx]->getY());
      // If already within steer limit, just set parent appropriately and return
      // to be added to the tree
      if(dist < STEER_DELTA)
      {
        rand->setParent(nodeList[near_idx]);
        return;
      }

      // Otherwise, find normal vector from nearest to random node and 
      // travel to steer limit along that normal vector to create new node
      double norm_x = (rand->getX()-nodeList[near_idx]->getX())/dist;
      double norm_y = (rand->getY()-nodeList[near_idx]->getY())/dist;
      double new_x = nodeList[near_idx]->getX() + STEER_DELTA*norm_x;
      double new_y = nodeList[near_idx]->getY() + STEER_DELTA*norm_y;
      
      rand->setX(new_x);
      rand->setY(new_y);
      rand->setParent(nodeList[near_idx]);
    }
    
    bool obstacleFree(Node* new_node)
    {
      double x_new = new_node->getX();
      double y_new = new_node->getY();
      
      // Parent must have been collision free to be added to the graph, so check the
      // new node to see if it's collision free
      if(this->mapData[getMapIndex(x_new,y_new)] > OBSTACLE_THRESHOLD || this->mapData[getMapIndex(x_new,y_new)] < 0)
        return false;
      
      // Now check along the path between the parent and the new node to make sure the
      // space between them is collision free. Because this is discrete, there's a chance
      // that there could be an obstacle in the spaces we're not checking, but it should
      // be pretty small.
      double x_parent = new_node->getParent()->getX();
      double y_parent = new_node->getParent()->getY();
      double x_check = (x_new+x_parent)/2.0;
      double y_check = (y_new+y_parent)/2.0;
      if(this->mapData[getMapIndex(x_check,y_check)] > OBSTACLE_THRESHOLD || this->mapData[getMapIndex(x_check,y_check)] < 0)
        return false;
      
      // If it passed all these checks, assume path is obstacle free
      return true;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
      std::cout << "Saving map" << std::endl;
      this->mapInfo = msg->info;
      this->mapData = msg->data;
      std::cout << "Map saved" << std::endl;

      this->x_min = msg->info.origin.position.x;
      this->y_min = msg->info.origin.position.y;
      this->x_max = this->x_min + msg->info.width*msg->info.resolution;
      this->y_max = this->y_min + msg->info.height*msg->info.resolution;
      std::cout << "x limits: " << this->x_min << " " << this->x_max << std::endl;
      std::cout << "y limits: " << this->y_min << " " << this->y_max << std::endl;
      this->getCostMap();
    }
    
    void getCostMap()
    {
      // This function works by "inflating" the obstacles to add new values to the
      // costmap in the free space.
      std::vector<int8_t>* cost_map = new std::vector<int8_t>{this->mapData};
      std::vector<int8_t>* temp_map = new std::vector<int8_t>{this->mapData};
      
      // Find the indices which are free space so we can ignore everything else
      std::vector<int> free_indices;
      for(int i{0}; i < this->mapData.size(); i++)
      {
        if(this->mapData[i] == 0)
          free_indices.push_back(i);
      }
      
      // Keep inflating until we've reached the set inflation distance
      for(int i{0}; i < (int)std::round(INFLATION_DISTANCE/this->mapInfo.resolution); i++)
      {
        std::vector<int> temp_indices;
        for(int j{0}; j < free_indices.size(); j++)
        {
          // Check all neighboring cells for an obstacle
          int left = free_indices[j] - 1;
          int right = free_indices[j] + 1;
          int top = free_indices[j] - this->mapInfo.width;
          int bottom = free_indices[j] + this->mapInfo.width;
          if(left > 0 && temp_map[0][left] != 0)
          {
            cost_map[0][free_indices[j]] = 0.9*temp_map[0][left];
          }
          else if(right < temp_map[0].size() && temp_map[0][right] != 0)
          {
            cost_map[0][free_indices[j]] = 0.9*temp_map[0][right];
          }
          else if(top > 0 && temp_map[0][top] != 0)
          {
            cost_map[0][free_indices[j]] = 0.9*temp_map[0][top];
          }
          else if(bottom < temp_map[0].size() && temp_map[0][bottom] != 0)
          {
            cost_map[0][free_indices[j]] = 0.9*temp_map[0][bottom];
          }
          else
          {
            temp_indices.push_back(free_indices[j]);
          }
        }
        // Moving new cost map to temp map for next cycle
        // and getting new set of free space indices
        temp_map[0] = cost_map[0];
        free_indices = temp_indices;
      }
      this->mapData = cost_map[0];
      std::cout << "Costmap modified" << std::endl;
      nav_msgs::OccupancyGrid map;
      map.info = this->mapInfo;
      map.data = this->mapData;
      map_pub.publish(map);
      delete cost_map;
      delete temp_map;
    }

    int getMapIndex(double x, double y)
    {
      // Find and return the row-major order map index for this position in the map
      double delta_x = x - this->mapInfo.origin.position.x;
      double delta_y = y - this->mapInfo.origin.position.y;
      int x_grid = (int)std::round(delta_x/this->mapInfo.resolution);
      int y_grid = (int)std::round(delta_y/this->mapInfo.resolution);
      if(x_grid < 0)
        x_grid = 0;
      else if(x_grid >= this->mapInfo.width)
        x_grid = this->mapInfo.width - 1;
      if(y_grid < 0)
        y_grid = 0;
      else if(y_grid >= this->mapInfo.height)
        y_grid = this->mapInfo.height - 1;

      return x_grid + y_grid*(this->mapInfo.width);
    }
};

class RRTPlanner {
  private:
    RRTstar* rrt_tree;
    double x_odom;
    double y_odom;
    double x_goal;
    double y_goal;
    double yaw_odom;

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Subscriber gap_sub;
  public:
    RRTPlanner(ros::NodeHandle node_handle) :
      nh(node_handle),
      pose_sub(nh.subscribe("/pf/pose/odom", 1, &RRTPlanner::odomCallback, this)),
      gap_sub(nh.subscribe("/gap_center", 1, &RRTPlanner::gapCallback, this))
    {
      this->rrt_tree = new RRTstar(node_handle, GOAL_SAMPLE_RATE);
      this->x_odom = 0;
      this->y_odom = 0;
      this->x_goal = 0;
      this->y_odom = 0;
      this->yaw_odom = 0;
    }

    ~RRTPlanner()
    {
      delete this->rrt_tree;
    }

    void plan()
    {
      this->rrt_tree->BuildRRT(this->x_odom, this->y_odom, this->x_goal, this->y_goal);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      this->x_odom = msg->pose.pose.position.x;
      this->y_odom = msg->pose.pose.position.y;
      double roll, pitch, yaw;      
      tf::Quaternion qt(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
      tf::Matrix3x3 rpyMat(qt);
      rpyMat.getRPY(roll, pitch, yaw);
      this->yaw_odom = yaw;
    }

    void gapCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
      // Gap is relative to the car so we need to transform it to the world frame
      // for planning
      this->x_goal = this->x_odom + std::cos(this->yaw_odom)*msg->x - std::sin(this->yaw_odom)*msg->y;
      this->y_goal = this->y_odom + std::sin(this->yaw_odom)*msg->x + std::cos(this->yaw_odom)*msg->y;
    }
};



int main( int argc, char** argv )
{
  ros::init(argc, argv, "rrt_planner");
  ros::NodeHandle nh;
  RRTPlanner rrt_planner(nh);
  ros::Rate rate(10);
  while(ros::ok())
  {
    rrt_planner.plan();
    ros::spinOnce();
    rate.sleep();
  }
}
