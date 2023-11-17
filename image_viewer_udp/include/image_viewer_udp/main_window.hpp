/**
 * @file /include/image_viewer_udp/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef image_viewer_udp_MAIN_WINDOW_H
#define image_viewer_udp_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>
#include <QImage>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace image_viewer_udp
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void slotUpdateImg();
  cv::Mat canny_edge(cv::Mat image);
  void bi(cv::Mat& cloneImage);
  void whitelane(cv::Mat& cloneImage);
 
  cv::Mat white_region_of_interest(cv::Mat& cloneImage);
  cv::Mat yellow_region_of_interest(cv::Mat& cloneImage);

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace image_viewer_udp

#endif  // image_viewer_udp_MAIN_WINDOW_H
