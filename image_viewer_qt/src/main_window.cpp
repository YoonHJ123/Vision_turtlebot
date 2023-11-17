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
#include "../include/image_viewer_qt/main_window.hpp"
#include <vector>
#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_qt
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

  QObject::connect(ui.pushButton_1, SIGNAL(clicked()), this, SLOT(Button1()));
  QObject::connect(ui.pushButton_2, SIGNAL(clicked()), this, SLOT(Button2()));
  QObject::connect(ui.Slider_1, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(ui.Slider_2, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(ui.Slider_3, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(ui.Slider_4, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(ui.Slider_5, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(ui.Slider_6, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChanged(int)));
  QObject::connect(&qnode, SIGNAL(Cam_SIGNAL()), this, SLOT(ImgRec()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::ImgRec()
{
  if (qnode.Cam_img != nullptr)
  {
    // OpenCV 이미지를 Qt 이미지로 변환
    IMG = *(qnode.Cam_img);
    cv::resize(IMG, IMG, cv::Size(430, 230));

    IMG2 = IMG;

    cv::GaussianBlur(IMG, IMG, cv::Size(3, 3), 0);

    Binary(IMG);
    YellowLine(IMG);
    WhiteLine(IMG);

    GreenLine(IMG);
    MintLine(IMG);
    RedLine(IMG);

    LineCheck();

    judge();

    Move();
    
    ui.lcdNumber_11->display(qnode.X);

    qnode.Cam_img = NULL;
    qnode.isRecved = 0;
  }
}

void MainWindow::Binary(cv::Mat& IMG)
{
  cv::Mat BIN = IMG;
  cv::cvtColor(IMG, BIN, cv::COLOR_RGB2HSV);
  cv::inRange(BIN, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), BIN);

  QImage QImage(BIN.data, BIN.cols, BIN.rows, BIN.step, QImage::Format_Indexed8);
  ui.Cam_4->setPixmap(QPixmap::fromImage(QImage));
}

void MainWindow::YellowLine(cv::Mat& IMG)
{
  cv::Mat CAN = IMG;

  cv::inRange(CAN, cv::Scalar(0, 0, 230), cv::Scalar(62, 255, 255), CAN);

  //윤곽선 이미지 CAN
  cv::Canny(CAN, CAN, 50, 200, 3);

  points[0] = cv::Point(240, 230);
  points[1] = cv::Point(240, 190);
  points[2] = cv::Point(370, 190);
  points[3] = cv::Point(430, 230);

  CAN = Region_of_interest(CAN, points);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(CAN, lines, 1, CV_PI / 180, 30, 30, 100);
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::Vec4i l = lines[i];
    cv::line(IMG2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    if (i == 0)
    {
      p[0] = (-1) * (l[0] - 410);
      p[1] = (-1) * (l[2] - 410);
    }
    else if (i == 1)
    {
      p[2] = (-1) * (l[0] - 410);
      p[3] = (-1) * (l[2] - 410);
    }
  }

  ui.lcdNumber->display(p[0]);
  ui.lcdNumber_2->display(p[1]);
  ui.lcdNumber_3->display(p[2]);
  ui.lcdNumber_4->display(p[3]);

  cv::resize(CAN, CAN, cv::Size(221, 121));

  QImage QImage(CAN.data, CAN.cols, CAN.rows, CAN.step, QImage::Format_Indexed8);
  ui.Cam_5->setPixmap(QPixmap::fromImage(QImage));
}

void MainWindow::WhiteLine(cv::Mat& IMG)
{
  cv::Mat CAN = IMG;

  cv::inRange(CAN, cv::Scalar(0, 0, 230), cv::Scalar(255, 127, 255), CAN);

  //윤곽선 이미지 CAN
  cv::Canny(CAN, CAN, 50, 200, 3);

  points[0] = cv::Point(0, 230);
  points[1] = cv::Point(60, 190);
  points[2] = cv::Point(190, 190);
  points[3] = cv::Point(190, 230);

  CAN = Region_of_interest(CAN, points);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(CAN, lines, 1, CV_PI / 180, 50, 30, 80);
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::Vec4i l = lines[i];
    cv::line(IMG2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    if (i == 0)
    {
      p[4] = l[0];
      p[5] = l[2];
    }
    else if (i == 1)
    {
      p[6] = l[0];
      p[7] = l[2];
    }
  }
  ui.lcdNumber_5->display(p[4]);
  ui.lcdNumber_6->display(p[5]);
  ui.lcdNumber_7->display(p[6]);
  ui.lcdNumber_8->display(p[7]);

  cv::resize(CAN, CAN, cv::Size(221, 121));

  QImage QImage(CAN.data, CAN.cols, CAN.rows, CAN.step, QImage::Format_Indexed8);
  ui.Cam_7->setPixmap(QPixmap::fromImage(QImage));
}

cv::Mat MainWindow::Region_of_interest(cv::Mat& image, cv::Point* points)
{
  cv::Mat img_mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
  const cv::Point* ppt[1] = { points };
  int npt[] = { 4 };

  cv::fillPoly(img_mask, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);

  cv::Mat img_masked;
  bitwise_and(image, img_mask, img_masked);

  return img_masked;
}

void MainWindow::LineCheck()
{
  cv::cvtColor(IMG2, IMG2, cv::COLOR_HSV2RGB);

  QImage QImage(IMG2.data, IMG2.cols, IMG2.rows, IMG2.step, QImage::Format_RGB888);
  ui.Cam_1->setPixmap(QPixmap::fromImage(QImage));
}

void MainWindow::SliderValueChanged(int value)
{
  minH = ui.Slider_1->sliderPosition();
  minS = ui.Slider_2->sliderPosition();
  minV = ui.Slider_3->sliderPosition();
  maxH = ui.Slider_4->sliderPosition();
  maxS = ui.Slider_5->sliderPosition();
  maxV = ui.Slider_6->sliderPosition();

  ui.Snum_1->display(minH);
  ui.Snum_2->display(minS);
  ui.Snum_3->display(minV);
  ui.Snum_4->display(maxH);
  ui.Snum_5->display(maxS);
  ui.Snum_6->display(maxV);
}

void MainWindow::Move()
{
  int sum1 = 0, sum2 = 0;
  for (int i = 0; i < 4; i++)
  {
    sum1 += p[i];
    sum2 += p[4 + i];
  }

  ui.lcdNumber_9->display(sum1);
  ui.lcdNumber_10->display(sum2);

  if (flag == 1)
  {
    if(initvelo == 0){
      qnode.X = 0.2;
      initvelo = 1;
    }

    if (0 < abs(sum1 - sum2) && abs(sum1 - sum2) < 100)
    {
      qnode.Z = 0;
    }
    else if (100 < abs(sum1 - sum2) && abs(sum1 - sum2) < 200)
    {
      qnode.Z = 0.2;
    }
    else if (200 < abs(sum1 - sum2) && abs(sum1 - sum2) < 300)
    {
      qnode.Z = 0.4;
    }
    else if (300 < abs(sum1 - sum2) && abs(sum1 - sum2) < 400)
    {
      qnode.Z = 0.6;
    }
    else if (400 < abs(sum1 - sum2) && abs(sum1 - sum2) < 500)
    {
      qnode.Z = 0.8;
    }
    else
    {
      qnode.Z = 1;
    }

    if (sum1 - sum2 < 0)
    {
      qnode.Z *= -1;
    }
  }
  else if(flag == 0)
  {
    qnode.X = 0;
    qnode.Z = 0;
  }
}

void MainWindow::MintLine(cv::Mat& IMG)
{
  cv::Mat MINT = IMG;
  cv::inRange(MINT, cv::Scalar(40, 255, 0), cv::Scalar(90, 255, 255), MINT);
  cv::erode(MINT, MINT, cv::Mat(), cv::Point(-1, -1), 3);

  QImage QImage(MINT.data, MINT.cols, MINT.rows, MINT.step, QImage::Format_Indexed8);
  ui.Cam_2->setPixmap(QPixmap::fromImage(QImage));

  cv::Mat stats, centroids;

  int num = cv::connectedComponentsWithStats(MINT, MINT, stats, centroids, 8, CV_32S);

  for (int i = 1; i < num; i++)
  {
    int x = static_cast<int>(stats.at<int>(i, cv::CC_STAT_LEFT));
    mintY = static_cast<int>(stats.at<int>(i, cv::CC_STAT_TOP));
    int width = static_cast<int>(stats.at<int>(i, cv::CC_STAT_WIDTH));
    int height = static_cast<int>(stats.at<int>(i, cv::CC_STAT_HEIGHT));

    cv::rectangle(IMG2, cv::Rect(x, mintY, width, height), cv::Scalar(100, 255, 255), 4);
  }
}

void MainWindow::RedLine(cv::Mat& IMG)
{
  cv::Mat RED = IMG;
  cv::inRange(RED, cv::Scalar(0, 127, 0), cv::Scalar(0, 255, 255), RED);

  QImage QImage(RED.data, RED.cols, RED.rows, RED.step, QImage::Format_Indexed8);
  ui.Cam_3->setPixmap(QPixmap::fromImage(QImage));

  cv::Mat stats, centroids;

  int num = cv::connectedComponentsWithStats(RED, RED, stats, centroids, 8, CV_32S);

  for (int i = 1; i < num; i++)
  {
    int x = static_cast<int>(stats.at<int>(i, cv::CC_STAT_LEFT));
    redY = static_cast<int>(stats.at<int>(i, cv::CC_STAT_TOP));
    int width = static_cast<int>(stats.at<int>(i, cv::CC_STAT_WIDTH));
    int height = static_cast<int>(stats.at<int>(i, cv::CC_STAT_HEIGHT));

    cv::rectangle(IMG2, cv::Rect(x, redY, width, height), cv::Scalar(255, 0, 0), 4);
  }
}

void MainWindow::GreenLine(cv::Mat& IMG)
{
  cv::Mat Green = IMG;
  cv::inRange(Green, cv::Scalar(95, 0, 0), cv::Scalar(100, 255, 255), Green);

  QImage QImage(Green.data, Green.cols, Green.rows, Green.step, QImage::Format_Indexed8);
  ui.Cam_6->setPixmap(QPixmap::fromImage(QImage));

  cv::Mat stats, centroids;

  int num = cv::connectedComponentsWithStats(Green, Green, stats, centroids, 8, CV_32S);

  for (int i = 1; i < num; i++)
  {
    int x = static_cast<int>(stats.at<int>(i, cv::CC_STAT_LEFT));
    greenY = static_cast<int>(stats.at<int>(i, cv::CC_STAT_TOP));
    int width = static_cast<int>(stats.at<int>(i, cv::CC_STAT_WIDTH));
    int height = static_cast<int>(stats.at<int>(i, cv::CC_STAT_HEIGHT));

    cv::rectangle(IMG2, cv::Rect(x, greenY, width, height), cv::Scalar(255, 0, 0), 4);
  }
}

void MainWindow::judge()
{
  if(redY > 220){
    flag = 0;
  }
  if(mintY > 220){
    veloflag = 1;
    mintY = 0;
  }
  if(greenY > 220){
    veloflag = 2;
    greenY = 0;
  }

  if(veloflag == 1){
    if(qnode.X <= 0.5){
      qnode.X += 0.005;
    }
  }
  else if(veloflag == 2){
    if(qnode.X >= 0.2){
      qnode.X -= 0.005;
    }
  }
}

void MainWindow::Button1()
{
  flag = 1;
  initvelo = 0;
}
void MainWindow::Button2()
{
  flag = 0;
}
}  // namespace image_viewer_qt