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
  qvtkWidgetPointer->SetRenderWindow(
      qvtkWidgetPointer->viewer->getRenderWindow());
  qvtkWidgetPointer->viewer->setupInteractor(
      qvtkWidgetPointer->GetInteractor(), qvtkWidgetPointer->GetRenderWindow());
  qvtkWidgetPointer->update();
}

MainWindow::~MainWindow() { delete ui; }

//===================================================
//  on_pushButton_pc_clicked
// an example for testing pcl and qt vision
//===================================================
void MainWindow::on_pushButton_pc_clicked()
{

  //
  // update the test process of adding pc.
  //
  qvtkWidgetPointer->addPointCloudExample();
}
