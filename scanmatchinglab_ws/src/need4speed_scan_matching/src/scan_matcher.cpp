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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std;

#include <eigen3/Eigen/Dense>    // for matrix computation

int numOfIter = 3;    // num of iteration of algorithm

int scanDataSize = 108;                           // num of laserscan pts to use
std::vector<double> scanXCurr   (scanDataSize, 0.0);  // laserscan
std::vector<double> scanYCurr   (scanDataSize, 0.0);
std::vector<double> scanAnglCurr(scanDataSize, 0.0);
std::vector<double> scanDistCurr(scanDataSize, 0.0);
std::vector<double> scanXPrev   (scanDataSize, 0.0);
std::vector<double> scanYPrev   (scanDataSize, 0.0);
std::vector<double> scanAnglPrev(scanDataSize, 0.0);
std::vector<double> scanDistPrev(scanDataSize, 0.0);

double odomPosXCurr   = 0.0;                       // odometry
double odomPosYCurr   = 0.0;
double odomOrienZCurr = 0.0;
double odomPosXPrev   = 0.0;
double odomPosYPrev   = 0.0;
double odomOrienZPrev = 0.0;

double globalPosXCurr   = 0.0;
double globalPosYCurr   = 0.0;
double globalOrienZCurr = 0.0;
double globalPosXPrev   = 0.0;
double globalPosYPrev   = 0.0;
double globalOrienZPrev = 0.0;


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

    double range_max;
    double range_min;
    double posConverge;
    double angleConverge;

    bool init;

    int max_iter;

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
      
      // Get values for range_min, range_max for lidar readings
      ros::param::get("/scan_matcher_node/range_max",this->range_max);
      ros::param::get("/scan_matcher_node/range_min",this->range_min);
      ros::param::get("/scan_matcher_node/max_iterations",this->max_iter);
      ros::param::get("/scan_matcher_node/position_convergence_error",this->posConverge);
      ros::param::get("/scan_matcher_node/angle_convergence_error",this->angleConverge);
      this->init = true;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
      //get odometry information from odometry topic

      // update old data
      odomPosXPrev   = odomPosXCurr;
      odomPosYPrev   = odomPosYCurr;
      odomOrienZPrev = odomOrienZCurr;
      // store posX, posY, and orientZ data from odometry
      odomPosXCurr = odom_msg->pose.pose.position.x;
      odomPosYCurr = odom_msg->pose.pose.position.y;
      double roll, pitch, yaw;      
      tf::Quaternion qt(
          odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y,
          odom_msg->pose.pose.orientation.z,
          odom_msg->pose.pose.orientation.w);
      tf::Matrix3x3 rpyMat(qt);
      rpyMat.getRPY(roll, pitch, yaw);
      odomOrienZCurr = yaw;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
      //get laserscan data from the scan topic

      // update old data
      scanXPrev = scanXCurr;
      scanYPrev = scanYCurr;
      scanAnglPrev = scanAnglCurr;
      scanDistPrev = scanDistCurr;
      // pick subset of data and store the angle and distance data
      double anglStart = -0.75 * M_PI;
      double anglInc   = 1.5 * M_PI / 1080.0;
      int indexStart = 1080 % scanDataSize / 2;
      int indexInc   = 1080 / scanDataSize;
      //std::cout << "ranges: ";
      for (int i=0; i<scanDataSize; i++)
      {
        scanAnglCurr[i] = anglStart + anglInc * (i*indexInc+indexStart);
        double range_i = msg->ranges[i*indexInc+indexStart];
        //if(range_i > this->range_max)
        //  range_i = this->range_max;
        //else if(range_i < this->range_min)
        //  range_i = this->range_min;
        scanDistCurr[i] = range_i;
        //std::cout << range_i << " ";
      }
      //std::cout << std::endl;
      //std::cout << "Got scan\n";
      // convert polar coord to XY coord
      for (int i=0; i<scanDataSize; i++)
      {
        scanXCurr[i] = scanDistCurr[i] * cos(scanAnglCurr[i]);
        scanYCurr[i] = scanDistCurr[i] * sin(scanAnglCurr[i]);
      }
      
      // run scan matching if we have a previous scan
      if(init)
        init = false;
      else
        do_scan_matching();
    }

    void do_scan_matching() {
      //The following steps are inspired from the Censi's PL-ICP paper. 
      //Try to modularize your code for each step to make it more readable and debuggable.
      //Use Eigen math library for linear algebra, matrix and vector operations, geometrical transformations.
      //1.Compute the coordinates of the second scan’s points in the first scan’s frame of reference, 
      //  according to the roto-translation obtained from odometry update.

      // calculate coord transform from last coord, using odometry data
      double dxGlobal = odomPosXCurr - odomPosXPrev;
      double dyGlobal = odomPosYCurr - odomPosYPrev;
      double unitVecXx =  cos(odomOrienZPrev);
      double unitVecXy =  sin(odomOrienZPrev);
      double unitVecYx = -sin(odomOrienZPrev);
      double unitVecYy =  cos(odomOrienZPrev);
      // use the coord transform as the first guess
      double posX   = (dxGlobal*unitVecYy - dyGlobal*unitVecYx) / (unitVecXx*unitVecYy - unitVecXy*unitVecYx);
      double posY   = (dyGlobal*unitVecXx - dxGlobal*unitVecXy) / (unitVecXx*unitVecYy - unitVecXy*unitVecYx);
      double orienZ = odomOrienZCurr - odomOrienZPrev;
      double posXPrev   = posX;
      double posYPrev   = posY;
      double orienZPrev = orienZ;
      int num_iter{0};
      // === repeat the entire algorithm k times ===
      for (int k=0; k<this->max_iter; k++)
      {
        num_iter++;
        // Compute 2nd scan's pts in 1st scan's frame
        std::vector<double> scanXCurr_PrevFrame(scanDataSize, 0.0);
        std::vector<double> scanYCurr_PrevFrame(scanDataSize, 0.0); 
        for (int i=0; i<scanDataSize; i++)
        {
          scanXCurr_PrevFrame[i] = scanXCurr[i]*cos(orienZ) - scanYCurr[i]*sin(orienZ) + posX;
          scanYCurr_PrevFrame[i] = scanXCurr[i]*sin(orienZ) + scanYCurr[i]*cos(orienZ) + posY;
        }

        //2.Find correspondence between points of the current and previous frame. You can use naive way of looking 
        //  through all points in sequence or use radial ordering of laser points to speed up the search.

        std::vector<double> normX(scanDataSize, 0.0);      // norm: unit norm vector
        std::vector<double> normY(scanDataSize, 0.0);
        std::vector<int> normNeighborIndex(scanDataSize, 0.0);  // (=j1). index of nearest scanPrev, where 
                                                                  //   scanCurr_PrevFrame is located
        // For each point in current scan, find closest line segment in previous surface
        for(int i{0}; i < scanDataSize; i++)
        {
          std::vector<double> dist_sqd;
          for(int j{0}; j < scanDataSize; j++)
          {
            dist_sqd.push_back( pow(scanXCurr_PrevFrame[i]-scanXPrev[j],2) + pow(scanYCurr_PrevFrame[i]-scanYPrev[j],2) );
          }
          normNeighborIndex[i] = std::min_element(dist_sqd.begin(),dist_sqd.end()) - dist_sqd.begin();

          if(normNeighborIndex[i] == 0 || (normNeighborIndex[i] < scanDataSize-1 && dist_sqd[normNeighborIndex[i]+1] < dist_sqd[normNeighborIndex[i]-1]))
          {
            // Calculate normal vector to line segment after closest point
            double segment_length = sqrt(pow(scanXPrev[normNeighborIndex[i]+1]-scanXPrev[normNeighborIndex[i]],2) + pow(scanYPrev[normNeighborIndex[i]+1]-scanYPrev[normNeighborIndex[i]],2));
            normX[i] = -(scanYPrev[normNeighborIndex[i]+1]-scanYPrev[normNeighborIndex[i]])/segment_length;
            normY[i] = (scanXPrev[normNeighborIndex[i]+1]-scanXPrev[normNeighborIndex[i]])/segment_length;
          }
          else
          {
            // Calculate normal vector to line segment before closest point
            double segment_length = sqrt(pow(scanXPrev[normNeighborIndex[i]]-scanXPrev[normNeighborIndex[i]-1],2) + pow(scanYPrev[normNeighborIndex[i]]-scanYPrev[normNeighborIndex[i]-1],2));
            normX[i] = -(scanYPrev[normNeighborIndex[i]]-scanYPrev[normNeighborIndex[i]-1])/segment_length;
            normY[i] = (scanXPrev[normNeighborIndex[i]]-scanXPrev[normNeighborIndex[i]-1])/segment_length;
          }
        }

        //3. Based on the correspondences, find the necessary tranform.
        //3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.

        // compute matrix M and vector g
        Eigen::Matrix4f M;
        M << 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0;
        Eigen::Vector4f g;
        g << 0.0, 0.0, 0.0, 0.0;
        for (int i=0; i<scanDataSize; i++)
        {
          if(scanDistCurr[i] > this->range_max)
          {
            continue;
          }
          Eigen::MatrixXf Mi(2,4);
          Mi << 1.0, 0.0, scanXCurr[i], -scanYCurr[i],    0.0, 1.0, scanYCurr[i], scanXCurr[i];

          Eigen::Matrix2f Ci;
          double weight = 1.0;
          Ci << normX[i]*normX[i],  normX[i]*normY[i],  normY[i]*normX[i],  normY[i]*normY[i];
          Ci = Ci * weight;

          Eigen::Vector2f PIi;
          double PIiX = scanXPrev[normNeighborIndex[i]];
          double PIiY = scanYPrev[normNeighborIndex[i]];
          PIi << PIiX, PIiY;
        
          M = M + (Mi.transpose() * Ci * Mi);
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
        SA = S.determinant() * S.inverse();\

        //3.b. You should get a fourth order polynomial in lamda which you can solve to get value
        //     (hint:greatest real root of polynomial equn) of lamda.

        // get matrix expression on left-hand-side of eqn (31) in Censi's paper
        Eigen::Matrix4f matInCoeff2;
        matInCoeff2 << A.inverse() * B * B.transpose() * A.inverse().transpose(),
                      -A.inverse() * B,
                     (-A.inverse() * B).transpose(),
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
        double coeff4 = -16.0;
        double coeff3 = -16.0*(S(0,0)+S(1,1));
        double coeff2 = 4.0*g.transpose()*matInCoeff2*g - 8.0*(S(0,0)*S(1,1)-S(0,1)*S(1,0)) - 4.0*(S(0,0)+S(1,1))*(S(0,0)+S(1,1));
        double coeff1 = 4.0*g.transpose()*matInCoeff1*g - 4.0*(S(0,0)+S(1,1))*(S(0,0)*S(1,1)-S(0,1)*S(1,0));
        double coeff0 =     g.transpose()*matInCoeff0*g - (S(0,0)*S(1,1)-S(0,1)*S(1,0))*(S(0,0)*S(1,1)-S(0,1)*S(1,0));

        // solve the 4th-order polynomial
        double lambda =  0.0;
        double lambda0 = 0.0;
        double lambda1 = 0.0;
        double lambda2 = 0.0;
        double lambda3 = 0.0;
        bool isLambda0Real = false;
        bool isLambda1Real = false;
        bool isLambda2Real = false;
        bool isLambda3Real = false;

/*        //method 1: from https://math.stackexchange.com/questions/785/is-there-a-general-formula-for-solving-4th-degree-equations-quartic
        double p1 = 2*coeff2*coeff2*coeff2 - 9*coeff3*coeff2*coeff1 + 27*coeff4*coeff1*coeff1
                   + 27*coeff3*coeff3*coeff0 - 72*coeff4*coeff2*coeff0;
        if (-4 * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0) 
               * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0)
               * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0) 
             + p1*p1 >=0 )
        {
          double p2 = p1 + sqrt(-4 
                     * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0) 
                     * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0)
                     * (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0) 
                     + p1*p1);
          double p3 = (coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0) / (3*coeff4*cbrt(p2/2))
                     + cbrt(p2/2) / (3*coeff4);
          if ( (coeff3*coeff3) / (4*coeff4*coeff4) - (2*coeff2) / (3*coeff4) + p3 >= 0 )
          {
            double p4 = sqrt( (coeff3*coeff3)/(4*coeff4*coeff4) - (2*coeff2)/(3*coeff4) + p3 );
            double p5 = (coeff3*coeff3)/(2*coeff4*coeff4) - (4*coeff2)/(3*coeff4) - p3;
            double p6 = (-coeff3*coeff3*coeff3/coeff4/coeff4/coeff4 
                        + 4*coeff3*coeff2/coeff4/coeff4 - 8*coeff1/coeff4) / (4*p4);
            if (p5 - p6 >= 0)
            {
              lambda0 = -coeff3/4/coeff4 - p4/2 + sqrt(p5-p6)/2;
              isLambda0Real = true;
            }
            if (p5 + p6 >= 0)
            {
              lambda2 = -coeff3/4/coeff4 + p4/2 + sqrt(p5+p6)/2;
              isLambda2Real = true;
            }
          }
        }
*/
      // Method 2: from https://en.wikipedia.org/wiki/Quartic_function
        double polyP = (8*coeff4*coeff2 - 3*coeff3*coeff3) / (8*coeff4*coeff4);
        double polyQ = (coeff3*coeff3*coeff3 - 4*coeff4*coeff3*coeff2 + 8*coeff4*coeff4*coeff1) / (8*coeff4*coeff4*coeff4);
        double polyDel0 = coeff2*coeff2 - 3*coeff3*coeff1 + 12*coeff4*coeff0;
        double polyDel1 = 2*coeff2*coeff2*coeff2 - 9*coeff3*coeff2*coeff1 + 27*coeff3*coeff3*coeff0
                       + 27*coeff4*coeff1*coeff1 - 72*coeff4*coeff2*coeff0;
        if (polyDel1*polyDel1 - 4*polyDel0*polyDel0*polyDel0 >= 0)  // to avoid complx root
        {
          //printf("pass QQ, ");
          double polyQQ = cbrt( (polyDel1 + sqrt(polyDel1*polyDel1 - 4*polyDel0*polyDel0*polyDel0)) / 2 );
          if (-2*polyP + 1/coeff4 * (polyQQ + polyDel0/polyQQ) >= 0)  // to avoid complex root
          {
            //printf("pass SS, ");
            double polySS = 1/2 * sqrt( -2/3*polyP + 1/(3*coeff4) * (polyQQ + polyDel0/polyQQ) );
            if (-4*polySS*polySS - 2*polyP + polyQ/polySS >= 0)  // to avoid complex root, get lambda 0,1
            {
              //printf("pass lam01, ");
              lambda0 = -coeff3/(4*coeff4) - polySS + 1/2*sqrt(-4*polySS*polySS - 2*polyP + polyQ/polySS );
              lambda1 = -coeff3/(4*coeff4) - polySS - 1/2*sqrt(-4*polySS*polySS - 2*polyP + polyQ/polySS );
              isLambda0Real = true;
              isLambda1Real = true;
            }
            if (-4*polySS*polySS - 2*polyP - polyQ/polySS >= 0)  // to avoid complex root, get lambda 2,3
            {
              //printf("pass lam23, ");
              lambda2 = -coeff3/(4*coeff4) + polySS + 1/2*sqrt(-4*polySS*polySS - 2*polyP - polyQ/polySS );
              lambda3 = -coeff3/(4*coeff4) + polySS - 1/2*sqrt(-4*polySS*polySS - 2*polyP - polyQ/polySS );
              isLambda2Real = true;
              isLambda3Real = true;
            }
          } // end 'if -2*polyP'
        } // end 'if polyDel1'


        // compare and select the largest lambda value (lambda0 vs lambda2)
        if (isLambda0Real && isLambda2Real)
        {
          if (lambda0 - lambda2 >= 0)
          {
            lambda = lambda0;
          } 
          else
          {
            lambda = lambda2;
          }
        }
        else if (isLambda0Real && ! isLambda2Real)
        { 
          lambda = lambda0;
        }
        else if (! isLambda0Real && isLambda2Real)
        {
          lambda = lambda2;
        }
        else if (! isLambda0Real && ! isLambda2Real)
        {
          //printf("No real lambda exist.\n")
        }
        // ----------end my code ------------------------------------

        //3.c. Use the calcualted value of lamda to estimate the transform using equation 24 in the Censi's paper.

        // ---------- start my code ---------------------------------
        // calculate the vector x
        Eigen::Vector4f x;
        if(std::isnan(lambda))
          lambda = 0.01;
        //std::cout << "lambda: " << lambda << std::endl;
        x = -(2*M + 2*lambda*W).inverse().transpose() * g;
        if(x(3) > 1)
          x(3) = 1;
        else if(x(3) < -1)
          x(3) = -1;
        if(x(2) > 1)
          x(2) = 1;
        else if(x(2) < -1)
          x(2) = -1;
        //std::cout << "x: " << std::endl << x << std::endl;

        // update posX, posY, and orienZ
        posX = x(0);
        posY = x(1);
        // calculate theta, considering the quadrant.
        //    Remember that x(2) = cos(th),  x(3) = sin(th).
        //    Also, note that 0< acos() <pi,  -pi/2< asin() <+pi/2)
        if (x(2)>0 && x(3)>0)    // quadrant 1
        {
          orienZ = (acos(x(2)) + asin(x(3)) ) / 2;
        }
        else if (x(2)<0 && x(3)>0)    // quadrant 2
        {
          orienZ = (acos(x(2)) + M_PI-asin(x(3)) ) / 2;
        }
        else if (x(2)<0 && x(3)<0)    // quadrant 3
        {
          orienZ = (-acos(x(2)) + (-M_PI-asin(x(3))) ) / 2;
        }
        else if (x(2)>0  && x(3)<0)    // quadrant 4
        {
          orienZ = (-acos(x(2)) + asin(x(3)) ) / 2;
        }

        // Determine if convergence has been reached
        double delta_pos = std::sqrt(pow(posX - posXPrev,2) + pow(posY - posYPrev,2));
        double delta_angle = std::abs(orienZ - orienZPrev);
        if(delta_pos < this->posConverge && delta_angle < this->angleConverge)
          break;
        // Prepare for next iteration
        posXPrev = posX;
        posYPrev = posY;
        orienZPrev = orienZ;
      } // end 'for k' (iteration of entire algorithm)
      std::cout << "Iterations: " << num_iter << std::endl;
      // ---------- end my code -------------------------------------

      //4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.

      // ---------- start my code -----------------------------------
      // store new data into old data
      globalPosXPrev   = globalPosXCurr;
      globalPosYPrev   = globalPosYCurr;
      globalOrienZPrev = globalOrienZCurr;
      //std::cout << "globalXPrev: " << globalPosXPrev << " globalYPrev: " << globalPosYPrev << " globalThetaPrev: " << globalOrienZPrev << std::endl;

      // get unit vector of prev coord in global frame
      double globalUnit_XPrev_xComp =  cos(globalOrienZPrev);
      double globalUnit_XPrev_yComp =  sin(globalOrienZPrev);
      double globalUnit_YPrev_xComp = -cos(globalOrienZPrev);
      double globalUnit_YPrev_yComp =  sin(globalOrienZPrev);

      // compute new global pos and orien
      globalPosXCurr = globalPosXPrev + posX*globalUnit_XPrev_xComp + posY*globalUnit_YPrev_xComp;
      globalPosYCurr = globalPosYPrev + posX*globalUnit_XPrev_yComp + posY*globalUnit_YPrev_yComp;
      globalOrienZCurr = globalOrienZPrev + orienZ;
      std::cout << "globalX: " << globalPosXCurr << " globalY: " << globalPosYCurr << " globalTheta: " << globalOrienZCurr << std::endl;
      
      geometry_msgs::PoseStamped new_pose;
      new_pose.header.frame_id = "/map";
      new_pose.header.stamp = ros::Time::now();
      new_pose.pose.position.x = globalPosXCurr;
      new_pose.pose.position.y = globalPosYCurr;
      
      tf2::Quaternion quat;
      quat.setRPY(0,0,globalOrienZCurr);
      new_pose.pose.orientation = tf2::toMsg(quat);

      this->pose_pub_.publish(new_pose);
      // ---------- end my code -------------------------------------

      //5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it.
      //Ideally, this should coincide with your actual current laserscan message.

    
    }
    ~ScanMatching() {}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle nh;
  ScanMatching scan_matching(nh);
  ros::spin();
  return 0;
}
