#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "vesc_msgs/VescStateStamped.h"

class velocityAverager {
  private:
    int bufferIdx;	// Keeps track of where in the buffer the next value should be stored
    int n;		// Keeps track of the number of values that should compose the average; max 10
    double buffer[10];	// Array to keep track of last 10 measurements
  public:
    velocityAverager();
    void cmdVelCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg);
    double getAverageVelocity();
};

velocityAverager::velocityAverager() {
  // Initialize buffer index to 0 so we start at front
  this->bufferIdx = 0;
  // No values saved in buffer yet
  this->n = 0;
  // Initialize buffer values to zero for completeness
  for(int i{0}; i < 10; i++)
    this->buffer[i] = 0;
}

void velocityAverager::cmdVelCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg) {
  // Increment the number of values that compose average until 10 is reached
  if(this->n < 10)
    this->n++;
  // Save the linear velocity from the current measurement in the buffer
  this->buffer[this->bufferIdx] = msg->state.speed;
  // Increment the buffer index for the next save, restarting once 10 has been passed
  this->bufferIdx++;
  if(this->bufferIdx > 9)
    this->bufferIdx = 0;

  // Print out the new average velocity
  ROS_INFO("New speed received: [%lf]; new average speed: [%lf]", msg->state.speed, this->getAverageVelocity());
}

double velocityAverager::getAverageVelocity() {
  // Calculate the average currently in the buffer
  double sum{0};
  for(int i{0}; i < this->n; i++)
    sum += this->buffer[i];
  return sum/this->n;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "brock_average");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  velocityAverager averager;
  ros::Subscriber sub = n.subscribe("/sensors/core", 1000, &velocityAverager::cmdVelCallback, &averager);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher avg_pub = n.advertise<std_msgs::Float64>("/average_velocity", 1000);

  // Sleep at a rate of 5 Hz
  ros::Rate loop_rate(5);

  while(ros::ok())
  {
    // Compose and publish the average velocity message
    std_msgs::Float64 msg;
    msg.data = averager.getAverageVelocity();
    avg_pub.publish(msg);
    
    // Spin and sleep to publish at 5 Hz rate
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
