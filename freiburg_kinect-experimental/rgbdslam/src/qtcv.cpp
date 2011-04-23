/* This is the main widget of the application.
 * It sets up some not yet useful menus and
 * three qlabels in the layout of the central widget
 * that can be used to show qimages via the slots
 * setDepthImage and setVisualImage.
 */
 #include <QtGui>
 #include <QPixmap>
 #include <QFont>

 #include "qtcv.h"
#include "ros/node_handle.h"

 MainWindow::MainWindow() : pause_on(true)
 {
     QWidget *widget = new QWidget;
     setCentralWidget(widget);

     //QWidget *topFiller = new QWidget;
     //topFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
     helpText = new QString(tr(
                 "<p><b>RGBD-SLAM</b> uses visual features to identify corresponding 3D locations "
                 "in RGBD data. The correspondences are used to reconstruct the camera motion. "
                 "The SLAM-backend HOG-MAN is used to integrate the transformations between"
                 "the RGBD-images and compute a globally consistent 6D trajectory.</p>"
                 "<b>Usage</b><ul>"
                 "<li><i>File->Pause/Unpause</i> starts/stops processing</li>"
                 "<li><i>File->Send Model and Reset</i> to clear the collected information.</li>"
                 "<li><i>Help->About RGBD-SLAM</i> displays this text</li></ul>"));
     infoLabel = new QLabel(tr("<i>Kinect Images</i>"));
     infoLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
     infoLabel->setAlignment(Qt::AlignCenter);
     //infoLabel->resize(10,10);
     visual_image_label = new QLabel(*helpText);
     visual_image_label->setWordWrap(true);
     visual_image_label->setMargin(10);
     visual_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
     visual_image_label->setAlignment(Qt::AlignCenter);
     visual_image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
     //depth_image_label = new QLabel(tr("<i>Waiting for depth data...</i>"));
     //depth_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
     //depth_image_label->setAlignment(Qt::AlignCenter);
     transform_label = new QLabel(tr("<i>Waiting for transformation matrix...</i>"));
     transform_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
     transform_label->setAlignment(Qt::AlignCenter);
     //QFont typewriter_font;
     //typewriter_font.setStyleHint(QFont::TypeWriter);
     //transform_label->setFont(typewriter_font);

     //QWidget *bottomFiller = new QWidget;
     //bottomFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

     gridlayout = new QGridLayout;
     gridlayout->setMargin(5);
     gridlayout->addWidget(infoLabel, 0,0);
     gridlayout->addWidget(transform_label, 1,0);
     gridlayout->addWidget(visual_image_label, 2,0);
     //gridlayout->addWidget(depth_image_label, 1,1);
     widget->setLayout(gridlayout);

     createActions();
     createMenus();

     //TODO: Initial message for statusbar
     //QString message = tr("A context menu is available by right-clicking");
     //statusBar()->showMessage(message);

     setWindowTitle(tr("RGBD SLAM"));
     setMinimumSize(640, 480);
     resize(850, 700);
      ros::NodeHandle n;
//     ros::Timer finishTimer = n.createTimer<MainWindow>(ros::Duration(120.0), &MainWindow::finishTimerCallback,this,true);
     QTimer::singleShot(1080000, this, SLOT(finishTimerCallback()));
    // ros::spin();
 }

 void MainWindow::setNewWidget(QWidget* new_widget){
   this->gridlayout->addWidget(new_widget, 0,1,3,1);
 }

 void MainWindow::setVisualImage(QImage qimage){
   visual_image_label->setPixmap(QPixmap::fromImage(qimage));}

 void MainWindow::setDepthImage(QImage qimage){
   depth_image_label->setPixmap(QPixmap::fromImage(qimage));}

 void MainWindow::setTransformation(QString transf){
   transform_label->setText(transf);}

 void MainWindow::resetCmd()
 {
     sendAll();
     emit reset();
     infoLabel->setText(tr("Model sent and Graph Reset."));
     if(pause_on) // if paused resume
         pause();
 }


 void MainWindow::sendAll()
 {
     emit sendAllClouds();
     infoLabel->setText(tr("Sending Whole Model</b>"));
 }
 void MainWindow::sendFinished()
 {
     infoLabel->setText(tr("Finished Sending</b>"));
 }

 void MainWindow::pause()
 {
     emit togglePause();
     pause_on = !pause_on;
     if(pause_on) infoLabel->setText(tr("Paused processing."));
     else infoLabel->setText(tr("Resumed processing."));
 }

 void MainWindow::about()
 {
     QMessageBox::about(this, tr("About Menu"), *helpText);
 }


 void MainWindow::createActions()
 {
     newAct = new QAction(tr("&Reset"), this);
     newAct->setShortcuts(QKeySequence::New);
     newAct->setStatusTip(tr("Reset the graph, clear all data collected"));
     connect(newAct, SIGNAL(triggered()), this, SLOT(resetCmd()));


     saveAct = new QAction(tr("&Send Whole Model"), this);
     saveAct->setShortcuts(QKeySequence::Save);
     saveAct->setStatusTip(tr("Send out all stored point clouds with corrected transform"));
     connect(saveAct, SIGNAL(triggered()), this, SLOT(sendAll()));

     pauseAct = new QAction(tr("&Pause/Unpause"), this);
     pauseAct->setShortcuts(QKeySequence::Print);
     pauseAct->setStatusTip(tr("Pause/unpause processing of frames"));
     connect(pauseAct, SIGNAL(triggered()), this, SLOT(pause()));

     exitAct = new QAction(tr("E&xit"), this);
     exitAct->setShortcuts(QKeySequence::Quit);
     exitAct->setStatusTip(tr("Exit the application"));
     connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

     aboutAct = new QAction(tr("&About RGBD-SLAM"), this);
     aboutAct->setStatusTip(tr("Show information about RGBD-SLAM"));
     connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

 }

 void MainWindow::createMenus()
 {
     fileMenu = menuBar()->addMenu(tr("&File"));
     fileMenu->addAction(newAct);
     fileMenu->addAction(saveAct);
     fileMenu->addAction(pauseAct);
     fileMenu->addSeparator();
     fileMenu->addAction(exitAct);

     helpMenu = menuBar()->addMenu(tr("&Help"));
     helpMenu->addAction(aboutAct);
 }
