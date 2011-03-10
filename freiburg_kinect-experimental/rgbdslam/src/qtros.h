/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include "ros/ros.h"
#include <QThread>
#include <QObject>

class QtROS : public QThread {
  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtROS(int argc, char *argv[], const char* node_name);
    ros::NodeHandle getNodeHandle(){ return *n; }
    /// This method contains the ROS event loop. Feel free to modify 
    void run();
  public slots:
    ///Connect to aboutToQuit signals, to stop the thread
    void quitNow();
  signals:
    ///Triggered if ros::ok() != true
    void rosQuits();
  private:
    bool quitfromgui;
    ros::NodeHandle* n;
};
#endif
