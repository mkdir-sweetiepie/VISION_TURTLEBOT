/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/image_viewer_udp/main_window.hpp"
#include <opencv2/opencv.hpp>
#include <QImage>
#include <geometry_msgs/Twist.h>
#include <vector>
/*****************************************************************************
** Namespaces
*****************************************************************************/

int value1;  // low h
int value2;  // low s
int value3;  // low v
int value4;  // high h
int value5;  // high s
int value6;  // high v

namespace image_viewer_udp
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  // subscribe한 이미지의 callback에서 시그널을 발생시켜 위 함수를 부른다.
  QObject::connect(&qnode, SIGNAL(sigRcvImg()), this, SLOT(slotUpdateImg()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

//이미지 처리 및 UI 업데이트를 수행하며 이후 UDP를 통한 이미지 전송을 위한 시그널을 발생
void MainWindow::slotUpdateImg()
{
  //원본 이미지
  QImage qImage((const unsigned char*)(qnode.imgView.data), qnode.imgView.cols, qnode.imgView.rows,
                QImage::Format_RGB888);
  ui.origin->setPixmap(QPixmap::fromImage(qImage.rgbSwapped()));
  //복사 이미지
  cv::Mat cloneImage = qnode.imgView.clone();

  //이진화 찾는 이미지
  bi(cloneImage);

  // whitelane
  whitelane(cloneImage);
}

//이진화 찾는 함수
void MainWindow::bi(cv::Mat& cloneImage)
{
  // ocam1_image를 HSV 색상 공간으로 변환하여 hsvImage에 저장
  cv::Mat hsvImage;
  cv::cvtColor(cloneImage, hsvImage, cv::COLOR_BGR2HSV);

  //반사를 줄이기 위해 가우시안 블러 적용
  cv::Mat blurredImage;
  cv::GaussianBlur(hsvImage, blurredImage, cv::Size(5, 5), 0);

  //이미 HSV 이미지로 변환한 변수를 사용
  cv::Scalar lower(value1, value2, value3);
  cv::Scalar upper(value4, value5, value6);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Mat binaryImage;
  cv::inRange(blurredImage, lower, upper, binaryImage);

  //이진 이미지를 표시
  QImage binaryQImage(binaryImage.data, binaryImage.cols, binaryImage.rows, binaryImage.step,
                      QImage::Format_Grayscale8);
  ui.test->setPixmap(QPixmap::fromImage(binaryQImage));

  value1 = ui.horizontalSlider_1->value();  // low h
  value2 = ui.horizontalSlider_2->value();  // low s
  value3 = ui.horizontalSlider_3->value();  // low v
  value4 = ui.horizontalSlider_4->value();  // high h
  value5 = ui.horizontalSlider_5->value();  // high s
  value6 = ui.horizontalSlider_6->value();  // high v

  ui.label_1->setText(QString::number(value1));
  ui.label_2->setText(QString::number(value2));
  ui.label_3->setText(QString::number(value3));
  ui.label_4->setText(QString::number(value4));
  ui.label_5->setText(QString::number(value5));
  ui.label_6->setText(QString::number(value6));
}

//흰색 선 이진화
void MainWindow::whitelane(cv::Mat& cloneImage)
{

  
  //||||||||||||||||||||| 선 이진화 & 엣지 검출 & 관심 영역 설정 업데이트 ||||||||||||||||||||||||||
  // 하얀색 이진화 찾기 (선)
  cv::Mat white_img;
  cv::cvtColor(cloneImage, white_img, cv::COLOR_BGR2HSV);
  // Apply Gaussian blur to reduce reflection
  cv::Mat blurredImage;
  cv::GaussianBlur(white_img, blurredImage, cv::Size(5, 5), 0);
  // 이미 HSV 이미지로 변환한 변수를 사용
  cv::Scalar lower_white(0, 0, 193);
  cv::Scalar upper_white(180, 55, 255);
  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Mat white;
  cv::inRange(blurredImage, lower_white, upper_white, white);
  // Canny 엣지 검출 적용
  cv::Mat wedge = canny_edge(white);
  // 관심 영역 설정
  cv::Mat wroi_conversion = white_region_of_interest(wedge);
  // UI에 이미지 업데이트
  QImage white_QImage(wroi_conversion.data, wroi_conversion.cols, wroi_conversion.rows, wroi_conversion.step,
                      QImage::Format_Grayscale8);
  ui.whitelane->setPixmap(QPixmap::fromImage(white_QImage));

  // 노란색 이진화 찾기 (선)
  cv::Mat yellow_img;
  cv::cvtColor(cloneImage, yellow_img, cv::COLOR_BGR2HSV);
  // 이미 HSV 이미지로 변환한 변수를 사용
  cv::Scalar lower_yellow(10, 10, 10);
  cv::Scalar upper_yellow(180, 255, 255);
  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Mat yellow;
  cv::inRange(blurredImage, lower_yellow, upper_yellow, yellow);
  // Canny 엣지 검출 적용
  cv::Mat yedge = canny_edge(yellow);
  // 관심 영역 설정
  cv::Mat yroi_conversion = yellow_region_of_interest(yedge);
  // UI에 이미지 업데이트
  QImage yellow_QImage(yroi_conversion.data, yroi_conversion.cols, yroi_conversion.rows, yroi_conversion.step,
                       QImage::Format_Grayscale8);
  ui.yellowlane->setPixmap(QPixmap::fromImage(yellow_QImage));



  //||||||||||||||||||||| 허프 변환 & 선 추출  업데이트 ||||||||||||||||||||||||||
  // 하프 변환 적용
  cv::Mat whalf_conversion;
  cv::resize(wroi_conversion, whalf_conversion, cv::Size(wroi_conversion.cols / 2, wroi_conversion.rows / 2));
  std::vector<cv::Vec4i> wlines;
  cv::HoughLinesP(whalf_conversion, wlines, 1, CV_PI / 180, 50, 30, 60);
  // 선을 그릴 원본 이미지 복사
  cv::Mat woutputImage = cloneImage.clone();
  // 추출된 선을 이미지에 그리기
  for (size_t i = 0; i < wlines.size(); i++)
  {
    cv::Vec4i wline = wlines[i];
    cv::line(woutputImage, cv::Point(wline[0] * 2, wline[1] * 2), cv::Point(wline[2] * 2, wline[3] * 2),
             cv::Scalar(255, 0, 0), 2);
  }

  // 노란색 선 그리기
  // 하얀색 선을 그린 이미지를 복사하여 사용
  cv::Mat youtputImage = woutputImage.clone();
  // 하프 변환 적용
  cv::Mat yhalf_conversion;
  cv::resize(yroi_conversion, yhalf_conversion, cv::Size(yroi_conversion.cols / 2, yroi_conversion.rows / 2));
  std::vector<cv::Vec4i> ylines;
  cv::HoughLinesP(yhalf_conversion, ylines, 1, CV_PI / 180, 50, 30, 60);
  // 추출된 선을 이미지에 그리기
  for (size_t i = 0; i < ylines.size(); i++)
  {
    cv::Vec4i yline = ylines[i];
    cv::line(youtputImage, cv::Point(yline[0] * 2, yline[1] * 2), cv::Point(yline[2] * 2, yline[3] * 2),
             cv::Scalar(0, 0, 255), 2);
  }

  // UI에 이미지 업데이트
  QImage yellowQImage(youtputImage.data, youtputImage.cols, youtputImage.rows, youtputImage.step,
                      QImage::Format_RGB888);
  ui.result->setPixmap(QPixmap::fromImage(yellowQImage.rgbSwapped()));



  //||||||||||||||||||||| 선 x좌표 중점 평균 계산  ||||||||||||||||||||||||||
  // 흰색 선의 중점 계산
  std::vector<cv::Point2f> wmidpoints;
  for (size_t i = 0; i < wlines.size(); i++)
  {
    cv::Vec4i wline = wlines[i];
    cv::Point2f wmidpoint((wline[0] + wline[2]) * 2 / 2.0, (wline[1] + wline[3]) * 2 / 2.0);
    wmidpoints.push_back(wmidpoint);
  }
  // 노란색 선의 중점 계산
  std::vector<cv::Point2f> ymidpoints;
  for (size_t i = 0; i < ylines.size(); i++)
  {
    cv::Vec4i yline = ylines[i];
    cv::Point2f ymidpoint((yline[0] + yline[2]) * 2 / 2.0, (yline[1] + yline[3]) * 2 / 2.0);
    ymidpoints.push_back(ymidpoint);
  }

  // 흰색 중점의 x-좌표 평균 계산
  double waverageX = 0;
  for (const auto& wmidpoint : wmidpoints)
  {
    waverageX += wmidpoint.x;
  }
  waverageX /= wmidpoints.size();
  // 노란색 중점의 x-좌표 평균 계산
  double yaverageX = 0;
  for (const auto& ymidpoint : ymidpoints)
  {
    yaverageX += ymidpoint.x;
  }
  yaverageX /= ymidpoints.size();

  int imageWidth = 360;
  double angular_velocity = 0.0;                // 각속도 초기화
  double linear_velocity = 0.0;                 // 선속도 초기화
  double center = (waverageX + yaverageX) / 2;  // 흰색과 노란색 중점 계산

  //||||||||||||||||||||| 비례제어  -> 두 선이 있을때와 없을떄 한 선만 있을때  ||||||||||||||||||||||||||
  // 중앙을 기준으로 비례 제어
  double error = std::abs(center - imageWidth / 2);
  double proportional_gain = 0.03;

  // 속도 조절 로직 두 선 모두 존재 시 중심에서의 오차에 따른 비레제어
  if (!wlines.empty() && !ylines.empty())
  {
    if (center < imageWidth / 2)
    {
      // 오른쪽으로 회전
      angular_velocity = proportional_gain * error;
      angular_velocity = std::min(std::max(angular_velocity, -0.7), 0.7);
      linear_velocity = 0.7;
    }
    else if (center > imageWidth / 2)
    {
      // 왼쪽으로 회전
      angular_velocity = -proportional_gain * error;
      angular_velocity = std::min(std::max(angular_velocity, -0.7), 0.7);
      linear_velocity = 0.7;
    }
    else
    {
      // 전진 (중앙 정렬)
      linear_velocity = 0.7;
      angular_velocity = 0.0;
    }
  }
  else
  {
    // 어떤 선도 감지되지 않았을 때
    if (wlines.empty() && ylines.empty())
    {
      // 양쪽 선이 없으면 먼저 없어진 선을 기준으로 90도 회전 후 전진
      angular_velocity = 0.7;
      linear_velocity = 0.77;
    }

    //한쪽 선만 있을때 각각의 남아있는 선 기준으로 비례제어
    else if (wlines.empty() || ylines.empty())
    {
      // 흰색 선이 없으면 왼쪽으로 회전 (비례 제어 적용)
      if (wlines.empty())
      {
        // 오른쪽으로 향하는 노란색 선을 기준으로 오차 계산
        double ycenter = yaverageX;
        double yerror = std::abs(ycenter - imageWidth / 2);
        angular_velocity = proportional_gain * yerror;
        angular_velocity = std::min(std::max(angular_velocity, -1.0), 1.0);
        linear_velocity = 0.7;
      }
      else
      {
        // 왼쪽으로 향하는 흰색 선을 기준으로 오차 계산
        double wcenter = waverageX;
        double werror = std::abs(wcenter - imageWidth / 2);
        angular_velocity = -proportional_gain * werror;
        angular_velocity = std::min(std::max(angular_velocity, -1.0), 1.0);
        linear_velocity = 0.7;
      }
    }
  }

  ui.label_7->setText(QString::number(center));
  ui.label_8->setText(QString::number(error));
  ui.label_9->setText(QString::number(angular_velocity));
  ui.label_10->setText(QString::number(linear_velocity));

  // 속도 적용 및 명령 발행
  qnode.twist.linear.x = linear_velocity;
  qnode.twist.angular.z = angular_velocity;
  qnode.cmd_vel_pub.publish(qnode.twist);
}

//이미지에 Canny 엣지 검출을 적용하는 함수
cv::Mat MainWindow::canny_edge(cv::Mat image)
{
  //이미지를 가우시안 블러로 먼저 전처리
  cv::Mat blur_conversion;
  cv::GaussianBlur(image, blur_conversion, cv::Size(5, 5), 0);

  // 가우시안 블러된 이미지에 Canny 엣지 검출 적용
  cv::Mat canny_conversion;
  cv::Canny(blur_conversion, canny_conversion, 50, 200);  // 7,8

  return canny_conversion;
}

//흰색 선 관심 영역 설정 함수
cv::Mat MainWindow::white_region_of_interest(cv::Mat& cloneImage)
{
  //입력 이미지의 높이와 너비 가져오기
  int imageHeight = cloneImage.rows;
  int imageWidth = cloneImage.cols;
  //이미지 크기와 같은 빈 이미지 생성
  cv::Mat imageMask = cv::Mat::zeros(cloneImage.size(), cloneImage.type());

  cv::Point vertices[4];              //관심 영역의 꼭지점 좌표 설정
  vertices[0] = cv::Point(0, 180);    //좌측 하단 꼭지점
  vertices[1] = cv::Point(100, 180);  //우측 하단 꼭지점
  vertices[2] = cv::Point(100, 50);   //우측 상단 꼭지점
  vertices[3] = cv::Point(0, 50);     //좌측 상단 꼭지점

  cv::Point roi_pts[1][4];  //꼭지점을 사용하여 다각형 정의
  roi_pts[0][0] = vertices[0];
  roi_pts[0][1] = vertices[1];
  roi_pts[0][2] = vertices[2];
  roi_pts[0][3] = vertices[3];

  const cv::Point* ppt[1] = { roi_pts[0] };  //꼭지점 배열을 포인터
  int npt[] = { 4 };                         //꼭지점 수

  //다각형이 채워질 대상 이미지, 꼭지점을 지정하는 포인터 배열, 꼭지점 수, 다각형의 수, 색상
  cv::fillPoly(imageMask, ppt, npt, 1, cv::Scalar(255, 255, 255));
  cv::Mat maskingImage;                                  //결과이미지
  cv::bitwise_and(cloneImage, imageMask, maskingImage);  //이미지와 마스크를 AND 연산하여 관심 영역만 추출

  return maskingImage;
}

//노란색 선 관심 영역 설정 함수
cv::Mat MainWindow::yellow_region_of_interest(cv::Mat& cloneImage)
{
  //입력 이미지의 높이와 너비 가져오기
  int imageHeight = cloneImage.rows;
  int imageWidth = cloneImage.cols;
  //이미지 크기와 같은 빈 이미지 생성
  cv::Mat imageMask = cv::Mat::zeros(cloneImage.size(), cloneImage.type());

  cv::Point vertices[4];              //관심 영역의 꼭지점 좌표 설정
  vertices[0] = cv::Point(241, 180);  //좌측 하단 꼭지점
  vertices[1] = cv::Point(360, 180);  //우측 하단 꼭지점
  vertices[2] = cv::Point(360, 50);   //우측 상단 꼭지점
  vertices[3] = cv::Point(241, 50);   //좌측 상단 꼭지점

  cv::Point roi_pts[1][4];  //꼭지점을 사용하여 다각형 정의
  roi_pts[0][0] = vertices[0];
  roi_pts[0][1] = vertices[1];
  roi_pts[0][2] = vertices[2];
  roi_pts[0][3] = vertices[3];

  const cv::Point* ppt[1] = { roi_pts[0] };  //꼭지점 배열을 포인터
  int npt[] = { 4 };                         //꼭지점 수

  //다각형이 채워질 대상 이미지, 꼭지점을 지정하는 포인터 배열, 꼭지점 수, 다각형의 수, 색상
  cv::fillPoly(imageMask, ppt, npt, 1, cv::Scalar(255, 255, 255));
  cv::Mat maskingImage;                                  //결과이미지
  cv::bitwise_and(cloneImage, imageMask, maskingImage);  //이미지와 마스크를 AND 연산하여 관심 영역만 추출

  return maskingImage;
}

}  // namespace image_viewer_udp