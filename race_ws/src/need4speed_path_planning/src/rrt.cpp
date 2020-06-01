#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <utility>
#include <vector>
#include <random>
#include <limits>

int ITER_LIMIT = 500;		// Limit on number of iterations for RRT solver
double GOAL_RADIUS = 0.1; 	// Radius around goal acceptable for completion
double STEER_DELTA = 0.1;

class Node {
  private:
    double X;
    double Y;
    Node* parent;

  public:
    Node()
    {
      this->X = 0;
      this->Y = 0;
      this->parent = nullptr;
    }

    Node(double x, double y, Node* parent)
    {
      this->X = x;
      this->Y = y;
      this->parent = parent;
    }

    void setX(double x) {this->X = x;}
    void setY(double y) {this->Y = y;}
    void setParent(Node* parent){this->parent = parent;}

    double getX() {return this->X;}
    double getY() {return this->Y;}
};

class RRTree {
  private:
    Node* root;
    Node* goal;
    double goalSampleRate;
    nav_msgs::MapMetaData mapInfo;
    std::vector<int8_t>* mapData;
    std::vector<Node*> nodeList;
    bool mapReceived;

    // Variables for random node generation
    std::default_random_engine generator;
    
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    
    ros::NodeHandle nh;
    ros::Subscriber map_sub;

  public:
    RRTree() {};

    RRTree(ros::NodeHandle node_handle, double goal_sample_rate) :
      nh(node_handle),
      map_sub(nh.subscribe("/map", 100, &RRTree::mapCallback, this))
    {
      this->root = nullptr;
      this->goal = nullptr;
      this->goalSampleRate = goal_sample_rate;
      this->mapData = nullptr;
    }
    
    virtual ~RRTree()
    {
      this->resetTree();
      delete this->mapData;
    }

    void resetTree()
    {
      for(int i{0}; i < this->nodeList.size(); i++)
        {
          delete this->nodeList[i];
        }
      this->nodeList.clear();
      if (this->goal != nullptr)
        delete this->goal;
      if (this->root != nullptr)
        delete this->root;
      this->root = nullptr;
      this->goal = nullptr;
    }

    void BuildRRT(double x_root, double y_root, double x_goal, double y_goal)
    {
      if(this->mapData == nullptr)
      {
        std::cout << "Skipping" << std::endl;
        return;
      }

      std::cout << "x_root: " << x_root << " y_root: " << y_root << std::endl;
      std::cout << "x_goal: " << x_goal << " y_goal: " << y_goal << std::endl;
      std::pair<int, int> map_coords = getMapIndex(x_root, y_root);
      std::cout << "Map coordinates of root: " << map_coords.first << " " << map_coords.second << std::endl;
      map_coords = getMapIndex(x_goal, y_goal);
      std::cout << "Map coordinates of goal: " << map_coords.first << " " << map_coords.second << std::endl;

      // Create root and goal nodes
      this->root = new Node(x_root, y_root, nullptr);
      this->goal = new Node(x_goal, y_goal, nullptr);
      // Add root to node list
      this->nodeList.push_back(this->root);
      
      for(i{0}; i < ITER_LIMIT; i++)
      {
        Node* random_node = this->getRandomNode();
        int nearest_idx = this->getNearestNodeIndex(random_node);
        Node* new_node = this->steer(random_node, nearest_idx);
        if(this->obstacleFree(new_node))
        {
        }
        else
          delete new_node;
      }
    }

    Node* getRandomNode()
    {
      std::uniform_real_distribution<double> distribution;
      double x = this->x_min + distribution(this->generator)*(this->x_max-this->x_min);
      double y = this->y_min + distribution(this->generator)*(this->y_max-this->y_min);
      Node* rand = new Node(x,y,nullptr);
      return rand;
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
    
    Node* steer(Node* rand, int near_idx)
    {
      double dist_sqd = std::pow(rand->getX()-nodeList[near_idx]->getX(),2) + std::pow(rand->getY()-nodeList[near_idx]->getY(),2);
      if(dist_sqd < std::pow(STEER_DELTA,2))
      {
        rand->setParent(nodeList[near_idx]);
        return rand;
      }

      double norm_x = (rand->getX()-nodeList[near_idx]->getX())/pow(dist_sqd,0.5);
      double norm_y = (rand->getY()-nodeList[near_idx]->getY())/pow(dist_sqd,0.5);
      double new_x = nodeList[near_idx]->getX() + STEER_DELTA*norm_x;
      double new_y = nodeList[near_idx]->getX() + STEER_DELTA*norm_x;
      delete rand;
      Node* new_node = new Node(new_x, new_y, nodeList[near_idx]);
      return new_node;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
      std::cout << "Saving map" << std::endl;
      this->mapInfo = msg->info;
      this->mapData = new std::vector<int8_t>;
      *(this->mapData) = msg->data;
      std::cout << this->mapData << std::endl;
      std::cout << "Map saved" << std::endl;

      this->x_min = msg->info.origin.position.x;
      this->y_min = msg->info.origin.position.y;
      this->x_max = this->x_min + msg->info.width*msg->info.resolution;
      this->y_max = this->y_min + msg->info.height*msg->info.resolution;
    }

    std::pair<int, int> getMapIndex(double x, double y)
    {
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

      return std::make_pair(x_grid, y_grid);
    }
};

class RRTPlanner {
  private:
    RRTree* rrt_tree;
    double x_odom;
    double y_odom;

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
  public:
    RRTPlanner(ros::NodeHandle node_handle) :
      nh(node_handle),
      pose_sub(nh.subscribe("/pf/pose/odom", 100, &RRTPlanner::odomCallback, this))
    {
      this->rrt_tree = new RRTree(node_handle, 0.1);
    }

    ~RRTPlanner()
    {
      delete this->rrt_tree;
    }

    void plan(double goal_x, double goal_y)
    {
      this->rrt_tree->BuildRRT(this->x_odom, this->y_odom, goal_x, goal_y);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      this->x_odom = msg->pose.pose.position.x;
      this->y_odom = msg->pose.pose.position.y;
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
    rrt_planner.plan(10, 10);
    ros::spinOnce();
    rate.sleep();
  }
}
