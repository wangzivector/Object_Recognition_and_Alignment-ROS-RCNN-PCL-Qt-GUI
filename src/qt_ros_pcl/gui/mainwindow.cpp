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
  ObjectRecognition = new ObjReco();
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
      "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/model/"+ ui->comboBox_mo->currentText() +".pcd";
  std::cout << "start read test ...\n";
  PointCloud::Ptr cloud = PointCloud::Ptr(new PointCloud());
  if (ObjectRecognition->pcdRead(path_read.toStdString().c_str(),
                                 ObjectRecognition->cloud_world))
  {
       addTextBrowser("finish read pcd from " + path_read + "showing it ...");
    if (qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world, "cloud_read"))
     addTextBrowser("qvtkWidgetObj show finished ");
  }
  else
    addTextBrowser("read failed from : " + path_read);
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
//  eaxample of pointcloud
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

void MainWindow::reloadParamWidget()
{
  //
  // change the widget saved values.
  //
}

void MainWindow::on_pushButton_co_clicked()
{
  if (ObjectRecognition->cloud_world->size() == 0)
  {
    addTextBrowser("start to read pcd ...");
    ObjectRecognition->pcdRead(
        "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/world/world3.pcd",
        ObjectRecognition->cloud_world);
  }
  ObjectRecognition->gridFilter(ObjectRecognition->cloud_world,
                                ObjectRecognition->cloud_world, 0.004f);
 addTextBrowser("start to compute reconstruction ...");
  ObjectRecognition->checkReconstruction();
  addTextBrowser("finish compute, start to visualize ..");
  qvtkWidgetObj->showPointNormal(ObjectRecognition->cloud_pointRGBNormal,
                                 "reconstruction");
}

void MainWindow::on_pushButton_lo_clicked()
{
  if (ObjectRecognition->loadIni())
    addTextBrowser("Params loaded successivefully.");
  reloadParamWidget();
}

void MainWindow::on_pushButton_sa_clicked()
{
  if (ObjectRecognition->saveIni())
    addTextBrowser("Params saved successivefully.");
}

void MainWindow::on_actionReset_Params_triggered()
{
  if (ObjectRecognition->loadIni())
    addTextBrowser("base Params loaded successivefully.");
  reloadParamWidget();
}

void MainWindow::on_comboBox_wo_currentIndexChanged(const QString& arg1)
{
  QString path =
      "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/world/" + arg1 + ".pcd";
  if (ObjectRecognition->pcdRead(path.toStdString().c_str(),
                                 ObjectRecognition->cloud_world))
   addTextBrowser("finished load pcd " + arg1);
  else
    addTextBrowser("failed load pcd " + path);
  qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world, "cloud_world");
}

void MainWindow::on_pushButton_cl_clicked()
{
    qvtkWidgetObj->vtkRemovePointCloud("all", true);
}
