/**
 * @file /include/image_viewer_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef image_viewer_qt_QNODE_HPP_
#define image_viewer_qt_QNODE_HPP_

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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_qt
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
    void Cam_callback(const sensor_msgs::ImageConstPtr &msg_img);
    cv::Mat* Cam_img = NULL;
    bool isRecved;
    float X = 0, Z = 0;

  Q_SIGNALS:
    void rosShutdown();
    void Cam_SIGNAL();

  private:
    int init_argc;
    char** init_argv;
  };

}  // namespace s

#endif /* image_viewer_qt_QNODE_HPP_ */
