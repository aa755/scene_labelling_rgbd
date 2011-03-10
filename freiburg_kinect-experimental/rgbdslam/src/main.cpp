/* registration main:
 * Create 
 * - a Qt Application 
 * - a ROS Node, 
 * - a ROS listener, listening to and processing kinect data
 */
#include "openni_listener.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qtcv.h"


int main(int argc, char** argv)
{
  QApplication application(argc,argv);
  QtROS qtRos(argc, argv,"rgbdslam"); //Thread object, to run the ros event processing loop in parallel to the qt loop
  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&application, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &application, SLOT(quit()));

  MainWindow window;
  window.show();
  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(qtRos.getNodeHandle(), 
                                 "/camera/rgb/image_mono",  
                                 "/camera/depth/image", 
                                 "/camera/rgb/camera_info", 
                                 "/camera/rgb/points",
                                 "SURF",
                                 "SURF");//,  "/camera/depth/image_raw");
  //Route every incoming image to the GUI
  QObject::connect(&kinect_listener, SIGNAL(newVisualImage(QImage)), &window, SLOT(setVisualImage(QImage)));
  //QObject::connect(&kinect_listener, SIGNAL(newDepthImage(QImage)), &window, SLOT(setDepthImage(QImage)));
  QObject::connect(&(kinect_listener.graph_mgr), SIGNAL(newTransformationMatrix(QString)), &window, SLOT(setTransformation(QString)));
  QObject::connect(&window, SIGNAL(reset()), &(kinect_listener.graph_mgr), SLOT(reset()));
  QObject::connect(&window, SIGNAL(togglePause()), &kinect_listener, SLOT(togglePause()));
  QObject::connect(&window, SIGNAL(sendAllClouds()), &(kinect_listener.graph_mgr), SLOT(sendAllClouds()));
  QObject::connect(&(kinect_listener.graph_mgr), SIGNAL(sendFinished()), &window, SLOT(sendFinished()));

  // Run main loop.
  qtRos.start();
  application.exec();
  //exit(0); // hack to mask something is wrong if return is called on application.exec(). Couldn't find out what's wrong. This was only b/c of qglviewer I think
}


