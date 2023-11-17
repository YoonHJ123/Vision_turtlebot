/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "../include/image_viewer_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
image_transport::Subscriber Cam_sub;
ros::Publisher pub;

namespace image_viewer_qt
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "image_viewer_qt");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.

  image_transport::ImageTransport it(n);
  Cam_sub = it.subscribe("/camera/image", 1, &QNode::Cam_callback, this);

  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    geometry_msgs::Twist turtlebot_msg;
    turtlebot_msg.linear.x = X;
    turtlebot_msg.angular.z = Z;
    pub.publish(turtlebot_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::Cam_callback(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (Cam_img == NULL && !isRecved)
  {
    Cam_img = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);
    if (Cam_img != NULL)
    {
      Q_EMIT Cam_SIGNAL();
      isRecved = true;
    }
  }
}

}  // namespace image_viewer_qt
