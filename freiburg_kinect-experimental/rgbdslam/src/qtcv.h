/* This is the main widget of the application.
 * It sets up some not yet useful menus and
 * three qlabels in the layout of the central widget
 * that can be used to show qimages via the slots
 * setDepthImage and setVisualImage.
 */
 #ifndef QTCV_H
 #define QTCV_H

 #include <QMainWindow>
 #include <QGridLayout>
#include "ros/ros.h"

 class QAction;
 class QActionGroup;
 class QLabel;
 class QMenu;

 /** Small GUI Class to visualize and control rgbdslam
  * See Help->About for a short description */
 class MainWindow : public QMainWindow
 {
     Q_OBJECT

 public:
     MainWindow();
     void setNewWidget(QWidget* new_widget);
 signals:
     void reset(); 
     void togglePause();
     void sendAllClouds(); ///< Signifies the sending of the whole model
      
 public slots:
     void setVisualImage(QImage);
     void setDepthImage(QImage);
     void setTransformation(QString);
     void sendFinished(); ///< Call to display, that sending finished


 public slots:
     void resetCmd();
     void sendAll();
     void pause();
     void about();
    void finishTimerCallback() {
        sendAll();
        std::cerr<<"model sent on timer event\n";
    }

 private:
     void createActions();
     void createMenus();

     QString *helpText;
     QMenu *fileMenu;
     QMenu *helpMenu;
     QAction *newAct;
     QAction *saveAct;
     QAction *pauseAct;
     QAction *exitAct;
     QAction *aboutAct;
     QLabel *infoLabel;
     QLabel *visual_image_label;
     QLabel *depth_image_label;
     QLabel *stats_image_label;
     QLabel *transform_label;
     QGridLayout* gridlayout;
     bool pause_on;
 };

 #endif
