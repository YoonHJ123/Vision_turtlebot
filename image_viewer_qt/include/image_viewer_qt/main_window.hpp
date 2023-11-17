/**
 * @file /include/image_viewer_qt/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef image_viewer_qt_MAIN_WINDOW_H
#define image_viewer_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <geometry_msgs/Twist.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace image_viewer_qt
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
    void Binary(cv::Mat &IMG);
    void Line(cv::Mat &IMG);
    void WhiteLine(cv::Mat &IMG);
    cv::Mat Region_of_interest(cv::Mat& image, cv::Point* points);
    void YellowLine(cv::Mat &IMG);
    void Move();
    void LineCheck();
    void MintLine(cv::Mat &IMG);
    void RedLine(cv::Mat &IMG);
    void GreenLine(cv::Mat &IMG);
    void judge();

    cv::Mat IMG, IMG2;
    int minH=0, minS=0, minV=0, maxH=0, maxS=0, maxV=0;
    int flag = 0, veloflag = 0, initvelo = 0;
    int mintY = 0, redY = 0, greenY = 0;

    cv::Point points[4];

    int p[8];

  public Q_SLOTS:
    void ImgRec();
    void SliderValueChanged(int value);
    void Button1();
    void Button2();

  private:
    Ui::MainWindowDesign ui;
    QNode qnode;

  };

}  // namespace s

#endif  // image_viewer_qt_MAIN_WINDOW_H
