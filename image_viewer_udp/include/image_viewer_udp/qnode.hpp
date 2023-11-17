/**
 * @file /include/image_viewer_udp/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef image_viewer_udp_QNODE_HPP_
#define image_viewer_udp_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_udp
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  cv::Mat imgRaw;
  cv::Mat imgView;
  geometry_msgs::Twist twist;
  ros::Publisher cmd_vel_pub;

Q_SIGNALS:
  void rosShutdown();
  void sigRcvImg();

private:
  int init_argc;
  char** init_argv;

  ros::Subscriber subImage;
  
  void callbackImage(const sensor_msgs::ImageConstPtr& msg);
  
};

}  // namespace image_viewer_udp

#endif /* image_viewer_udp_QNODE_HPP_ */
