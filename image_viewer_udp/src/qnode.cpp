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
#include <sstream>
#include "../include/image_viewer_udp/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_udp
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
  ros::init(init_argc, init_argv, "image_viewer_udp");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.

  subImage = n.subscribe("/camera/image", 1, &QNode::callbackImage, this);
  // 로봇 제어 명령을 보낼 퍼블리셔 생성
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

//이미지 메시지를 처리 함수
void QNode::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  imgRaw = cv_ptr->image;
  cv::resize(cv::Mat(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image), imgView, cv::Size(360, 240),
             0, 0, CV_INTER_LINEAR);
  Q_EMIT sigRcvImg();
}



}  // namespace image_viewer_udp
