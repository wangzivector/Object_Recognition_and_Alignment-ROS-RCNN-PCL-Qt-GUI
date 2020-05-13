#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  //
  // Set up the QVTK window and all stuff of render thing :)
  //
  qvtkWidgetPointer = new qvtk(this);
  ui->verticalLayout->addWidget(qvtkWidgetPointer);
  qvtkWidgetPointer->addPointCloudExample();
//  qvtkWidgetPointer->update();
}

MainWindow::~MainWindow() { delete ui; }

//===================================================
//  on_pushButton_pc_clicked
//  an example for testing pcl and qt vision
//  update the test process of adding pc.
//===================================================
void MainWindow::on_pushButton_pc_clicked()
{
  qvtkWidgetPointer->addPointCloudExample();
}

//===================================================
//  on_pushButton_quit_clicked
//===================================================
void MainWindow::on_pushButton_quit_clicked() { this->~MainWindow(); }
