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
  addTextBrowser("start to read text ..");
  QString path_read =
      "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/world/world3.pcd";
  std::cout << "start read test ...\n";
  PointCloud::Ptr cloud_read = PointCloud::Ptr(new PointCloud());

  if (pcd_ioObj->pcdRead(path_read.toStdString().c_str(), cloud_read))
  {
    if (qvtkWidgetObj->showPointCloud(cloud_read, "cloud_read"))
      std::cout << "show finished \n";
  }
  else
    std::cout << "read failed from : " << path_read.toStdString().c_str()
              << std::endl;
  addTextBrowser("finish read pcd from " + path_read);
}

//===================================================
//  on_pushButton_quit_clicked
//===================================================
void MainWindow::on_pushButton_quit_clicked() { this->~MainWindow(); }

//===================================================
//  on_tabWidget_tabBarClicked
//===================================================
void MainWindow::on_tabWidget_tabBarClicked(int index)
{
  ui->textBrowser->append("This is for param editting. index : " +
                          QString::number(index));
  ui->textBrowser->moveCursor(ui->textBrowser->textCursor().End);
}

//===================================================
//  on_pushButton_ex_clicked
//===================================================
void MainWindow::on_pushButton_ex_clicked()
{
  QPixmap ico(":/image/icon.jpg");
  ui->label_pic->setScaledContents(true);
  ui->label_pic->setPixmap(ico);
//  QPixmap scaledPixmap = ico.scaled(picSize, Qt::KeepAspectRatio);
  qvtkWidgetObj->addPointCloudExample();
  addTextBrowser("example show complished.");
}

//===================================================
//  addTextBrowser
//===================================================
void MainWindow::addTextBrowser(QString text)
{
  ui->textBrowser->append(text);
  ui->textBrowser->moveCursor(ui->textBrowser->textCursor().End);
}
