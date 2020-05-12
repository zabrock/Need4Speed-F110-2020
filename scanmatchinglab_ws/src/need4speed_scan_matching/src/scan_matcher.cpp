#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;

// myCode start -----------------------------------------------------
#include <eigen3/Eigen/Dense>
#include <complex.h>

int scanDataSize = 108;              // determine number of data points in use
std::vector<float> scanRaw;
float anglFacing = 0.0;
float posX = 0.0;
float posY = 0.0;
// myCode end -------------------------------------------------------


class ScanMatching {

  private:
    ros::NodeHandle nh_;
    vector<geometry_msgs::Point> points_;
    vector<geometry_msgs::Point> transformed_points_;
    vector<geometry_msgs::Point> prev_points_;
    tf::TransformBroadcaster br_;
    tf::Transform tr_;
    geometry_msgs::PoseStamped estimated_pose_;
    const int MAX_ITERATIONS = 100;
    //Topic names for publishers and subscribers
    const string& SCAN_TOPIC  = "/scan";
    const string& POSE_TOPIC = "/scan_match_location";
    const string& ODOM_TOPIC = "/odom";
    const string& FAKE_SCAN_TOPIC = "/fake_scan_match";
    const string& DRIVE_TOPIC = "/nav";

    //Publishers
    ros::Publisher pose_pub_;
    ros::Publisher fake_scan_pub_;
    ros::Publisher drive_pub_;

    //Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

  public:

    ScanMatching(ros::NodeHandle& nh) :nh_(nh) {
      cout<<"Node started..\n";
      //Publishers : Add others as per your need
      drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 1);
      fake_scan_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(FAKE_SCAN_TOPIC, 1);
      pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
      //Subscriber for odometry and laserscan
      odom_sub_ = nh_.subscribe(ODOM_TOPIC, 10, &ScanMatching::odom_callback, this);
      scan_sub_ = nh_.subscribe(SCAN_TOPIC, 1, &ScanMatching::scan_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
      //get odometry information from odometry topic
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
      //get laserscan data from the scan topic
      // myCode start ---------------------------------------------
      scanRaw = msg->ranges;
      // myCode end -----------------------------------------------
    }




    void do_scan_matching() {
      //The following steps are inspired from the Censi's PL-ICP paper. 
      //Try to modularize your code for each step to make it more readable and debuggable.
      //Use Eigen math library for linear algebra, matrix and vector operations, geometrical transformations.
      //1.Compute the coordinates of the second scan’s points in the first scan’s frame of reference, 
      //  according to the roto-translation obtained from odometry update.

      // myCode Start -----------------------------------------------
      // Pre-processing 1: get subset of data
      float anglStart = -0.75 * M_PI;
      float anglInc   = 1.5 * M_PI / 1080.0;
      int indexStart = 1080 % scanDataSize / 2;
      int indexInc   = 1080 / scanDataSize;
      float scanAngl[scanDataSize];
      float scanDist[scanDataSize];
      for (int i=0; i<scanDataSize; i++)
      {
        scanAngl[i] = anglStart + anglInc * (i*indexInc+indexStart);
        scanDist[i] = scanRaw[i*indexInc+indexStart];
      }

      // Pre-processing 2: convert RTheta coord to XY coord
      float scanXCarFrame[scanDataSize];
      float scanYCarFrame[scanDataSize];
      for (int i=0; i<scanDataSize; i++)
      {
        scanXCarFrame[i] = scanDist[i] * cos(scanAngl[i]);
        scanYCarFrame[i] = scanDist[i] * sin(scanAngl[i]);
      }

      // Compute 2nd scan's pts in 1st scan's frame
      float scanXWorldFrame[scanDataSize];
      float scanYWorldFrame[scanDataSize]; 
      for (int i=0; i<scanDataSize; i++)
      {
          scanXWorldFrame[i] = scanXCarFrame[i]*cos(anglFacing) - scanYCarFrame[i]*sin(anglFacing) + posX;
          scanYWorldFrame[i] = scanYCarFrame[i]*sin(anglFacing) + scanYCarFrame[i]*cos(anglFacing) + posY;
      }
      // myCode End -------------------------------------------------

      //2.Find correspondence between points of the current and previous frame. You can use naive way of looking 
      //  through all points in sequence or use radial ordering of laser points to speed up the search.

      // myCode Start -----------------------------------------------
      float normX[scanDataSize];
      float normY[scanDataSize];
      float tanX[scanDataSize];
      float tanY[scanDataSize];
      for (int i=1; i<scanDataSize-1; i++)
      {
        float j1X = scanXWorldFrame[i-1];
        float j1Y = scanYWorldFrame[i-1];
        float j2X = scanXWorldFrame[i+1];
        float j2Y = scanYWorldFrame[i+1];

        float tanSlope = (j2Y-j1Y) / (j2X-j1X);
        float tanLeng  = sqrt(1 + tanSlope*tanSlope);
        tanX[i] = 1.0      / tanLeng;
        tanY[i] = tanSlope / tanLeng;

        float normSlope = -1 / tanSlope;
        float normLeng  = sqrt(1 + normSlope*normSlope);
        normX[i] = 1.0       / normLeng;
        normY[i] = normSlope / normLeng;
      }
      // myCode End -------------------------------------------------

      //3. Based on the correspondences, find the necessary tranform.
      //3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.

      // myCode start -----------------------------------------------
      Eigen::MatrixXf M(4,4);
      M << 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0;
      Eigen::VectorXf g(4);
      g << 0.0, 0.0, 0.0, 0.0;
      for (int i=0; i<scanDataSize; i++)
      {
        Eigen::MatrixXf Mi(2,4);
        Mi << 1.0, 0.0, scanXWorldFrame[i], -scanYWorldFrame[i], 0.0, 1.0, scanYWorldFrame[i], scanXWorldFrame[i];
        float weight = 1.0;

        Eigen::Matrix2f Ci;
        Ci << normX[i]*normX[i],  normX[i]*normY[i],  normX[i]*normY[i],  normY[i]*normY[i];
        Ci = Ci * weight;
        Eigen::Vector2f PIi;

        float projLeng = scanXWorldFrame[i]*tanX[i] + scanYWorldFrame[i]*tanY[i]; 
        PIi << tanX[i], tanY[i];
        PIi = PIi * projLeng;
        
        M = M + Mi.transpose() * Ci * Mi;
        g = g + (-2.0 * PIi.transpose() * Ci * Mi).transpose();
      }
      Eigen::Matrix4f W;
      W << 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 1.0, 0.0,   0.0, 0.0, 0.0, 1.0;

      Eigen::Matrix2f A;
      A << M.block(0,0,2,2) * 2.0;

      Eigen::Matrix2f B;
      B << M.block(0,2,2,2) * 2.0;

      Eigen::Matrix2f D;
      D << M.block(2,2,2,2) * 2.0;

      Eigen::Matrix2f S;
      S = D - B.transpose() * A.inverse() * B;

      Eigen::Matrix2f SA;
      SA = S.determinant() * S.inverse();
      // myCode end -------------------------------------------------

      //3.b. You should get a fourth order polynomial in lamda which you can solve to get value
      //     (hint:greatest real root of polynomial equn) of lamda.

      // myCode start -----------------------------------------------
      float s00 = S(0,0);
      float s01 = S(0,1);
      float s10 = S(1,0);
      float s11 = S(1,1);

      // get matrix expression on left-hand-side of eqn (31) in Censi's paper
      Eigen::Matrix4f matInCoeff2;
      matInCoeff2 << A.inverse() * B * B.transpose() * A.inverse().transpose(),
                     -A.inverse()*B,
                     (-A.inverse()*B).transpose(),
                     Eigen::MatrixXf::Identity(2,2);
      Eigen::Matrix4f matInCoeff1;
      matInCoeff1 << A.inverse() * B * SA * B.transpose() * A.inverse().transpose(),
                     -A.inverse() * B * SA,
                     (-A.inverse() * B * SA).transpose(),
                     SA;
      Eigen::Matrix4f matInCoeff0;
      matInCoeff0 << A.inverse() * B * SA.transpose() * SA * B.transpose() * A.inverse().transpose(),
                     -A.inverse() * B * SA.transpose() * SA,   
                     (-A.inverse() * B * SA.transpose() * SA).transpose(),
                     SA.transpose() * SA;

      // get the coefficients of 4th-order polynomial
      float coeff4 = -16.0;
      float coeff3 = -16.0 * (s00+s11);
      float coeff2 = 4.0*g.transpose()*matInCoeff2*g - 8*(s00*s11-s01*s10) - 4*(s00+s11)*(s00+s11);
      float coeff1 = 4.0*g.transpose()*matInCoeff1*g - 4*(s00+s11)*(s00*s11-s01*s10);
      float coeff0 =     g.transpose()*matInCoeff0*g - (s00*s11-s01*s10)*(s00*s11-s01*s10);

      // solve the 4th-order polynomial
      float polyP = (8*coeff4*coeff2 - 3*coeff3*coeff3) / (8*coeff4*coeff4);
      float polyQ = (coeff3*coeff3*coeff3 - 4*coeff4*coeff3*coeff2 + 8*coeff4*coeff4*coeff1) / (8*coeff4*coeff4*coeff4);
      float polyD0 = coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0;
      float polyD1 = 2*coeff2*coeff2*coeff2 - 9*coeff3*coeff2*coeff1 + 27*coeff3*coeff3*coeff0
                     + 27*coeff4*coeff1*coeff1 - 72*coeff4*coeff2*coeff0;
      //float polyQQ = std::cbrt( (polyD1 + csqrt(polyD1*polyD1 - 4*polyD0*polyD0*polyD0)) / 2 );
      // STUCK HERE!!
      // myCode end -------------------------------------------------

      //3.c. Use the calcualted value of lamda to estimate the transform using equation 24 in the Censi's paper.


      //4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.


      //5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it.
      //Ideally, this should coincide with your actual current laserscan message.
    }

    ~ScanMatching() {}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle nh;
  ScanMatching scan_matching(nh);
  scan_matching.do_scan_matching();
  ros::spin();
  return 0;
}
