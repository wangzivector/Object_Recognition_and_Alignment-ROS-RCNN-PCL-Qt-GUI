/*
 * mainwindow.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cpp finishes the tasks msg publish, display and
 * point cloud process.
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  //
  // Set up the QVTK window and all stuff of render thing :)
  //
  qvtkWidgetObj = new qvtk(this);
  ui->verticalLayout->addWidget(qvtkWidgetObj);
//  qvtkWidgetObj->addPointCloudExample();
}

MainWindow::~MainWindow() { delete ui; }

//===================================================
//  on_pushButton_pc_clicked
//  an example for testing pcl and qt vision
//  update the test process of adding pc.
//===================================================
void MainWindow::on_pushButton_pc_clicked()
{
//  /// VTK Fun basic check
//  qvtkWidgetObj->addPointCloudExample();

//  /// ROS Msg publish check
//  qrosObj->qrosPublish("wangzi test!");

  /// pcd read check
  QString path_read = "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/world/world3.pcd";
  PointCloud::Ptr cloud_read = PointCloud::Ptr (new PointCloud());
  std::cout << "start read test ...\n";
  if(pcd_ioObj->pcdRead(path_read.toStdString().c_str(), cloud_read))
  {
    if (qvtkWidgetObj->showPointCloud(cloud_read, "cloud"))
      std::cout << "show finished \n";
  }
  else std::cout << "read failed from : " << path_read.toStdString().c_str() << std::endl;
}

//===================================================
//  on_pushButton_quit_clicked
//===================================================
void MainWindow::on_pushButton_quit_clicked() { this->~MainWindow(); }
