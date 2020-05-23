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
  current_time = QTime::currentTime();
  ObjectRecognition->loadIni();
  reloadParamWidget();
  ui->verticalLayout->addWidget(qvtkWidgetObj);
  table = new QStandardItemModel();
  ui->tableView_si->setModel(table);
  tableDisplay();
}

MainWindow::~MainWindow() { delete ui; }

//===================================================
//  on_pushButton_sh_clicked
//  show feature histagrams
//===================================================
void MainWindow::on_pushButton_sh_clicked()
{
  switch (ui->feature_choose->currentIndex())
  {
  case 0:
    qvtkWidgetObj->addPlotterExample(
        ObjectRecognition->cloud_descr_shot352_world, "# world ");
    qvtkWidgetObj->addPlotterExample(
        ObjectRecognition->cloud_descr_shot352_object, "# object ");
    break;
  case 1:
    break;
  case 2:
    qvtkWidgetObj->addPlotterExample(ObjectRecognition->cloud_descr_fpfh_world,
                                     "# world ");
    qvtkWidgetObj->addPlotterExample(ObjectRecognition->cloud_descr_fpfh_object,
                                     "# object ");
    break;

  default:
    addTextBrowser("Plot", "wrong chose in feature choose box.");
  }
}

//===================================================
//  on_pushButton_quit_clicked
//===================================================
void MainWindow::on_pushButton_quit_clicked() { this->~MainWindow(); }

//===================================================
//  on_pushButton_ex_clicked
//  eaxample of pointcloud
//===================================================
void MainWindow::on_pushButton_ex_clicked()
{
  //  QPixmap ico(":/image/icon.jpg");
  //  QPixmap scaledPixmap = ico.scaled(100, 100, Qt::KeepAspectRatio);
  //  ui->label_pic->setScaledContents(true);
  //  ui->label_pic->setPixmap(scaledPixmap);
  qvtkWidgetObj->addPointCloudExample();
  addTextBrowser("Exam", "example show complished.");
}

//===================================================
//  addTextBrowser
//===================================================
void MainWindow::addTextBrowser(QString head_text, QString text)
{
  current_time.restart();
  ui->textBrowser->append("[" + QString::number(current_time.second()) + ":" +
                          QString::number(current_time.msec()) + "] " + "<" +
                          head_text + ">  " + text);
  ui->textBrowser->moveCursor(ui->textBrowser->textCursor().End);
}

void MainWindow::reloadParamWidget()
{
  //
  // change the widget saved values.
  //
  ui->spinBox_filter1->setValue(ObjectRecognition->axisFilter_axis_size);
  ui->spinBox_filter2->setValue(ObjectRecognition->gridFilter_grid_size);
  ui->spinBox_filter3->setValue(ObjectRecognition->planeFilter_threshold_plane);
  ui->spinBox_filter4->setValue(ObjectRecognition->outlierFilter_outlier_meanK);
  ui->spinBox_filter5->setValue(
      ObjectRecognition->outlierFilter_outlier_Thresh);
  ui->spinBox_filter6->setValue(
      ObjectRecognition->backgroundFilter_noise_filter);
  ui->spinBox_filter7->setValue(ObjectRecognition->backgroundFilter_resolution);
  ui->spinBox_filter8->setValue(ObjectRecognition->downSample_sample_radius);

  ui->spinBox_seg_1->setValue(ObjectRecognition->rgbSegmentation_dist_thre);
  ui->spinBox_seg_2->setValue(
      ObjectRecognition->rgbSegmentation_point_color_thre);
  ui->spinBox_seg_3->setValue(
      ObjectRecognition->rgbSegmentation_region_color_thre);
  ui->spinBox_seg_4->setValue(ObjectRecognition->rgbSegmentation_min_cluster);
  ui->spinBox_seg_5->setValue(
      ObjectRecognition->mlsReconstruction_search_radius);
  ui->spinBox_seg_6->setValue(
      ObjectRecognition->shot352Estimation_descr_rad_352);
  ui->spinBox_seg_7->setValue(
      ObjectRecognition->shot1344Estimation_descr_rad_1344);

  ui->spinBox_ran_1->setValue(ObjectRecognition->RANSA_max_iterations);
  ui->spinBox_ran_2->setValue(ObjectRecognition->RANSA_number_samples);
  ui->spinBox_ran_3->setValue(ObjectRecognition->RANSA_randomness);
  ui->spinBox_ran_4->setValue(ObjectRecognition->RANSA_similar_thre);
  ui->spinBox_ran_5->setValue(ObjectRecognition->RANSA_max_corr_distance);
  ui->spinBox_ran_6->setValue(ObjectRecognition->RANSA_min_sample_distance);
  ui->spinBox_ran_7->setValue(ObjectRecognition->NDT_transepsilon);
  ui->spinBox_ran_8->setValue(ObjectRecognition->NDT_stepsize);
  ui->spinBox_ran_9->setValue(ObjectRecognition->NDT_resolution);
  ui->spinBox_ran_10->setValue(ObjectRecognition->NDT_maxiteration);

  ui->spinBox_sac_1->setValue(ObjectRecognition->SACIA_number_samples);
  ui->spinBox_sac_2->setValue(ObjectRecognition->SACIA_randomness);
  ui->spinBox_sac_3->setValue(ObjectRecognition->SACIA_min_sample_distance);
  ui->spinBox_sac_4->setValue(
      ObjectRecognition->SACIA_max_correspondence_distance);
  ui->spinBox_sac_5->setValue(ObjectRecognition->SACIA_max_iterations);
  ui->spinBox_sac_6->setValue(ObjectRecognition->ICP_max_corr_distance);
  ui->spinBox_sac_7->setValue(ObjectRecognition->ICP_max_iter_icp);
  ui->spinBox_sac_8->setValue(ObjectRecognition->ICP_transformation);
  ui->spinBox_sac_9->setValue(ObjectRecognition->ICP_euclidean_Fitness);
}

void MainWindow::on_pushButton_co_clicked()
{
  if (ObjectRecognition->cloud_world->size() == 0)
  {
    addTextBrowser("Cons", "start to read pcd ...");
    QString path = "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/model/" +
                   ui->comboBox_mo->currentText() + ".pcd";
    ObjectRecognition->pcdReadModel(path.toStdString().c_str());
  }
  //  ObjectRecognition->gridFilter(
  //      ObjectRecognition->cloud_world_filter,
  //      ObjectRecognition->cloud_world_filter,
  //      ObjectRecognition->gridFilter_grid_size);
  addTextBrowser("Cons", "start to compute reconstruction ...");
  ObjectRecognition->checkReconstruction();
  addTextBrowser("Cons", "finish compute checkReconstruction ");

  refreshPloudCloudVTK();
}

void MainWindow::on_pushButton_lo_clicked()
{

  QMessageBox::StandardButton btn;
  btn = QMessageBox::question(this, "Tips", "Are you sure to load params?",
                              QMessageBox::Yes | QMessageBox::No);
  if (btn == QMessageBox::Yes)
  {
    if (ObjectRecognition->loadIni())
      addTextBrowser("Para", "Params loaded successivefully.");
    reloadParamWidget();
  }
}

void MainWindow::on_pushButton_sa_clicked()
{
  QMessageBox::StandardButton btn;
  btn =
      QMessageBox::question(this, "Tips", "Are you sure to save/write params?",
                            QMessageBox::Yes | QMessageBox::No);
  if (btn == QMessageBox::Yes)
  {
    if (ObjectRecognition->saveIni())
      addTextBrowser("Para", "Params saved successivefully.");
  }
}

void MainWindow::on_actionReset_Params_triggered()
{
  if (ObjectRecognition->loadIni())
    addTextBrowser("Para", "base Params loaded successivefully.");
  reloadParamWidget();
}

void MainWindow::on_comboBox_wo_currentIndexChanged(const QString& arg1)
{
  QString path =
      "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/world/" + arg1 + ".pcd";
  if (ObjectRecognition->pcdReadWorld(path.toStdString().c_str()))
    addTextBrowser("Pcds", "finished load pcd " + arg1);
  else
    addTextBrowser("Pcds", "failed load pcd " + path);
  qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world, "cloud_world");
}

void MainWindow::on_pushButton_cl_clicked()
{
  qvtkWidgetObj->vtkRemovePointCloud("all", true);
}

void MainWindow::on_spinBox_filter1_valueChanged(double arg1)
{
  ObjectRecognition->axisFilter_axis_size = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->axisFilter_axis_size, 'f', 4));
}

void MainWindow::on_spinBox_filter2_valueChanged(double arg1)
{
  ObjectRecognition->gridFilter_grid_size = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->gridFilter_grid_size, 'f', 4));
}

void MainWindow::on_spinBox_filter3_valueChanged(double arg1)
{
  ObjectRecognition->planeFilter_threshold_plane = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->planeFilter_threshold_plane, 'f', 4));
}

void MainWindow::on_spinBox_filter4_valueChanged(int arg1)
{
  ObjectRecognition->outlierFilter_outlier_meanK = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->outlierFilter_outlier_meanK));
}

void MainWindow::on_spinBox_filter5_valueChanged(double arg1)
{
  ObjectRecognition->outlierFilter_outlier_Thresh = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->outlierFilter_outlier_Thresh, 'f', 4));
}

void MainWindow::on_spinBox_filter6_valueChanged(int arg1)
{
  ObjectRecognition->backgroundFilter_noise_filter = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->backgroundFilter_noise_filter));
}

void MainWindow::on_spinBox_filter7_valueChanged(double arg1)
{
  ObjectRecognition->backgroundFilter_resolution = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->backgroundFilter_resolution, 'f', 4));
}

void MainWindow::on_spinBox_filter8_valueChanged(double arg1)
{
  ObjectRecognition->downSample_sample_radius = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->downSample_sample_radius, 'f', 4));
}

void MainWindow::on_spinBox_seg_1_valueChanged(int arg1)
{
  ObjectRecognition->rgbSegmentation_dist_thre = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->rgbSegmentation_dist_thre));
}

void MainWindow::on_spinBox_seg_2_valueChanged(int arg1)
{
  ObjectRecognition->rgbSegmentation_point_color_thre = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->rgbSegmentation_point_color_thre));
}

void MainWindow::on_spinBox_seg_3_valueChanged(int arg1)
{
  ObjectRecognition->rgbSegmentation_region_color_thre = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->rgbSegmentation_region_color_thre));
}

void MainWindow::on_spinBox_seg_4_valueChanged(int arg1)
{
  ObjectRecognition->rgbSegmentation_min_cluster = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->rgbSegmentation_min_cluster));
}

void MainWindow::on_spinBox_seg_5_valueChanged(double arg1)
{
  ObjectRecognition->mlsReconstruction_search_radius = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->mlsReconstruction_search_radius, 'f',
                      4));
}

void MainWindow::on_spinBox_seg_6_valueChanged(double arg1)
{
  ObjectRecognition->shot352Estimation_descr_rad_352 = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->shot352Estimation_descr_rad_352, 'f',
                      4));
}

void MainWindow::on_spinBox_seg_7_valueChanged(double arg1)
{
  ObjectRecognition->shot1344Estimation_descr_rad_1344 = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->shot1344Estimation_descr_rad_1344, 'f',
                      4));
}

void MainWindow::on_spinBox_ran_1_valueChanged(int arg1)
{
  ObjectRecognition->RANSA_max_iterations = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->RANSA_max_iterations));
}

void MainWindow::on_spinBox_ran_2_valueChanged(int arg1)
{
  ObjectRecognition->RANSA_number_samples = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->RANSA_number_samples));
}

void MainWindow::on_spinBox_ran_3_valueChanged(int arg1)
{
  ObjectRecognition->RANSA_randomness = arg1;
  ui->textBrowser->append("param changed : " +
                          QString::number(ObjectRecognition->RANSA_randomness));
}

void MainWindow::on_spinBox_ran_4_valueChanged(double arg1)
{
  ObjectRecognition->RANSA_similar_thre = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->RANSA_similar_thre, 'f', 4));
}

void MainWindow::on_spinBox_ran_5_valueChanged(double arg1)
{
  ObjectRecognition->RANSA_max_corr_distance = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->RANSA_max_corr_distance, 'f', 4));
}

void MainWindow::on_spinBox_ran_6_valueChanged(double arg1)
{
  ObjectRecognition->RANSA_min_sample_distance = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->RANSA_min_sample_distance, 'f', 4));
}

void MainWindow::on_spinBox_ran_7_valueChanged(double arg1)
{
  ObjectRecognition->NDT_transepsilon = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->NDT_transepsilon, 'f', 4));
}

void MainWindow::on_spinBox_ran_8_valueChanged(double arg1)
{
  ObjectRecognition->NDT_stepsize = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->NDT_stepsize, 'f', 4));
}

void MainWindow::on_spinBox_ran_9_valueChanged(double arg1)
{
  ObjectRecognition->NDT_resolution = float(arg1);
  ui->textBrowser->append("param changed : " +
                          QString::number(ObjectRecognition->NDT_resolution));
}

void MainWindow::on_spinBox_ran_10_valueChanged(int arg1)
{
  ObjectRecognition->NDT_maxiteration = arg1;
  ui->textBrowser->append("param changed : " +
                          QString::number(ObjectRecognition->NDT_maxiteration));
}

void MainWindow::on_spinBox_sac_1_valueChanged(int arg1)
{
  ObjectRecognition->SACIA_number_samples = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->SACIA_number_samples));
}

void MainWindow::on_spinBox_sac_2_valueChanged(int arg1)
{
  ObjectRecognition->SACIA_randomness = arg1;
  ui->textBrowser->append("param changed : " +
                          QString::number(ObjectRecognition->SACIA_randomness));
}

void MainWindow::on_spinBox_sac_3_valueChanged(double arg1)
{
  ObjectRecognition->SACIA_min_sample_distance = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->SACIA_min_sample_distance, 'f', 4));
}

void MainWindow::on_spinBox_sac_4_valueChanged(double arg1)
{
  ObjectRecognition->SACIA_max_correspondence_distance = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->SACIA_max_correspondence_distance, 'f',
                      4));
}

void MainWindow::on_spinBox_sac_5_valueChanged(int arg1)
{
  ObjectRecognition->SACIA_max_iterations = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->SACIA_max_iterations));
}

void MainWindow::on_spinBox_sac_6_valueChanged(double arg1)
{
  ObjectRecognition->ICP_max_corr_distance = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->ICP_max_corr_distance, 'f', 4));
}

void MainWindow::on_spinBox_sac_7_valueChanged(int arg1)
{
  ObjectRecognition->ICP_max_iter_icp = arg1;
  ui->textBrowser->append("param changed : " +
                          QString::number(ObjectRecognition->ICP_max_iter_icp));
}

void MainWindow::on_spinBox_sac_8_valueChanged(double arg1)
{
  ObjectRecognition->ICP_transformation = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->ICP_transformation));
}

void MainWindow::on_spinBox_sac_9_valueChanged(double arg1)
{
  ObjectRecognition->ICP_euclidean_Fitness = arg1;
  ui->textBrowser->append(
      "param changed : " +
      QString::number(ObjectRecognition->ICP_euclidean_Fitness, 'f', 4));
}

void MainWindow::on_pushButton_pr_clicked()
{
  ObjectRecognition->deal_world = ui->checkBox_dealw->isChecked();
  ObjectRecognition->deal_object = ui->checkBox_dealo->isChecked();
  if (ObjectRecognition->cloud_world->size() == 0)
  {
    addTextBrowser("Pcds",
                   "no pointcloud yet, add .pcd into world pointcloud.");
    on_comboBox_wo_currentIndexChanged(ui->comboBox_wo->currentText());
  }
  if (ObjectRecognition->cloud_object->size() == 0)
  {
    addTextBrowser("Pcds",
                   "no pointcloud yet, add .pcd into object pointcloud.");
    on_comboBox_mo_currentIndexChanged(ui->comboBox_mo->currentText());
  }
  ObjectRecognition->reloadPointCloud(ObjectRecognition->deal_world,
                                      ObjectRecognition->deal_object);
  addTextBrowser(
      "Info",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));
  addTextBrowser(
      "Info",
      "world pointcloud size() = " +
          QString::number(ObjectRecognition->cloud_world_filter->size()));
  ObjectRecognition->reAxisFilter(ui->Axisfilter->isChecked());
  if (ui->Axisfilter->isChecked())
    addTextBrowser(
        "Filt",
        "Axisfilter finished.  size() = " +
            QString::number(ObjectRecognition->cloud_world_filter->size()));

  addTextBrowser(
      "Filt",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));

  ObjectRecognition->reGridFilter(ui->Gridfilter->isChecked());
  if (ui->Gridfilter->isChecked())
    addTextBrowser(
        "Filt",
        "Gridfilter finished.  size() = " +
            QString::number(ObjectRecognition->cloud_world_filter->size()));

  addTextBrowser(
      "Filt",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));

  ObjectRecognition->rePlaneFilter(ui->Planefilter->isChecked());
  if (ui->Planefilter->isChecked())
    addTextBrowser(
        "Filt",
        "Planefilter finished. size() = " +
            QString::number(ObjectRecognition->cloud_world_filter->size()));

  addTextBrowser(
      "Filt",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));
  ObjectRecognition->reOutlierFilter(ui->Outlierfilter->isChecked());

  if (ui->Outlierfilter->isChecked())
    addTextBrowser(
        "Filt",
        "Outlierfilter finished. size() = " +
            QString::number(ObjectRecognition->cloud_world_filter->size()));

  addTextBrowser(
      "Filt",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));

  ObjectRecognition->reMlsRecoonstruction(ui->Reconstruction->isChecked());
  if (ui->Reconstruction->isChecked())
    addTextBrowser(
        "Filt",
        "Reconstruction finished.  size() = " +
            QString::number(ObjectRecognition->cloud_world_filter->size()));

  addTextBrowser(
      "Filt",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_filter->width) + "/" +
          QString::number(ObjectRecognition->cloud_world_filter->height));

  ObjectRecognition->reKeypoint(ui->Keypoint->isChecked());
  if (ui->Keypoint->isChecked())
    addTextBrowser(
        "KeyP",
        "Keypoint finished.  size() = " +
            QString::number(ObjectRecognition->cloud_world_keypoint->size()));

  addTextBrowser(
      "KeyP",
      "current pointcloud : " +
          QString::number(ObjectRecognition->cloud_world_keypoint->width) +
          "/" +
          QString::number(ObjectRecognition->cloud_world_keypoint->height));

  addTextBrowser("Filt", "\n ### Filters finished. ### \n");
  addTextBrowser(
      "Filt",
      " world  size = " +
          QString::number(ObjectRecognition->cloud_world_filter->size()));
  addTextBrowser(
      "Filt",
      " object size = " +
          QString::number(ObjectRecognition->cloud_object_filter->size()));
  refreshPloudCloudVTK();
  tableDisplay();
}

void MainWindow::refreshPloudCloudVTK()
{
  qvtkWidgetObj->vtkRemovePointCloud("all", true);
  if (ui->checkBox_wc->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world,
                                  "cloud_world");
  if (ui->checkBox_wk->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world_filter,
                                  "cloud_world_filter");
  if (ui->checkBox_wr->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world_keypoint,
                                  "cloud_world_result");
  if (ui->checkBox_wa->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_world_aligned,
                                  "cloud_world_aligned");
  if (ui->checkBox_oc->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_object,
                                  "cloud_object");
  if (ui->checkBox_ok->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_object_filter,
                                  "cloud_object_filter");
  if (ui->checkBox_or->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_object_keypoint,
                                  "cloud_object_keypoint");
  if (ui->checkBox_oa->isChecked())
    qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_object_aligned,
                                  "cloud_object_alianed");

  addTextBrowser("Qvtk", "refresh the pointcloud display done.");
}

void MainWindow::on_pushButton_di_clicked() { refreshPloudCloudVTK(); }

void MainWindow::on_checkBox_dealw_clicked(bool checked)
{
  ObjectRecognition->deal_world = checked;
  addTextBrowser("Info", "world state changed : " + QString::number(checked));
}

void MainWindow::on_checkBox_dealo_clicked(bool checked)
{
  ObjectRecognition->deal_object = checked;
  addTextBrowser("Info", "object state changed : " + QString::number(checked));
}

void MainWindow::on_comboBox_mo_currentIndexChanged(const QString& arg1)
{
  QString path =
      "/home/wang/catkin_qtws/src/qt_ros_pcl/pcd/model/" + arg1 + ".pcd";
  if (ObjectRecognition->pcdReadModel(path.toStdString().c_str()))
    addTextBrowser("Pcds", "finished load pcd " + arg1);
  else
    addTextBrowser("Pcds", "failed load pcd " + path);
  qvtkWidgetObj->showPointCloud(ObjectRecognition->cloud_object,
                                "cloud_object");
}

void MainWindow::on_pushButton_fe_clicked()
{
  if (!ObjectRecognition->deal_process)
  {
    addTextBrowser("Feat", "not correct keypoints yet!");
    return;
  }
  if (ui->feature->isChecked())
    switch (ui->feature_choose->currentIndex())
    {
    case 0:
      addTextBrowser("Feat", "start shot352...");
      ObjectRecognition->reSHOT352(true);
      addTextBrowser("Feat", "\n## shot352 process done ##\n\n");
      break;
    case 1:
      break;
    case 2:
      addTextBrowser("Feat", "start fpfh...");
      ObjectRecognition->reFPFH(true);
      addTextBrowser("Feat", "\n ## fpfh process done ##\n\n");
      break;

    default:
      addTextBrowser("Feat", "wrong chose in feature choose box.");
    }
  else
    addTextBrowser("Feat", "didnt tick feature checkbox yet.");
  tableDisplay();
}

void MainWindow::tableDisplay()
{
  table->setColumnCount(3);
  table->setRowCount(5);

  table->setItem(0, 1, new QStandardItem("world"));
  table->setItem(0, 2, new QStandardItem("object"));
  table->setItem(1, 0, new QStandardItem("cloud"));
  table->setItem(2, 0, new QStandardItem("filter"));
  table->setItem(3, 0, new QStandardItem("keypoint"));
  table->setItem(4, 0, new QStandardItem("shot352"));
  table->setItem(5, 0, new QStandardItem("fpfh"));
  table->setItem(
      1, 1,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_world->size())).c_str()));
  table->setItem(
      2, 1,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_world_filter->size()))
              .c_str()));
  table->setItem(
      3, 1,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_world_keypoint->size()))
              .c_str()));
  table->setItem(
      1, 2,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_object->size()))
              .c_str()));
  table->setItem(
      2, 2,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_object_filter->size()))
              .c_str()));
  table->setItem(
      3, 2,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_object_keypoint->size()))
              .c_str()));
  table->setItem(
      4, 1,
      new QStandardItem(
          std::to_string(
              int(ObjectRecognition->cloud_descr_shot352_world->size()))
              .c_str()));
  table->setItem(
      4, 2,
      new QStandardItem(
          std::to_string(
              int(ObjectRecognition->cloud_descr_shot352_object->size()))
              .c_str()));
  table->setItem(
      5, 1,
      new QStandardItem(
          std::to_string(int(ObjectRecognition->cloud_descr_fpfh_world->size()))
              .c_str()));
  table->setItem(
      5, 2,
      new QStandardItem(
          std::to_string(
              int(ObjectRecognition->cloud_descr_fpfh_object->size()))
              .c_str()));
}

void MainWindow::on_pushButton_do_clicked()
{
  switch (ui->comboBox_do->currentIndex())
  {
  case 0:
    // filter
    on_pushButton_pr_clicked();
    break;
  case 1:
    // filter feature
    on_pushButton_pr_clicked();
    on_pushButton_fe_clicked();
    break;
  case 2:
    // filter feature align
    on_pushButton_pr_clicked();
    on_pushButton_fe_clicked();
    on_pushButton_al_clicked();
    break;
  case 3:
    on_pushButton_pr_clicked();
    on_pushButton_fe_clicked();
    on_pushButton_al_clicked();
    on_pushButton_regi_clicked();
    break;
  }
  refreshPloudCloudVTK();
}

void MainWindow::on_pushButton_al_clicked()
{
  // align
  if (ui->Registration->isChecked())
  {
    switch (ui->regi_choose->currentIndex())
    {
    case 0:
      addTextBrowser("Algn", "start RANSACFPFH.");
      if (ObjectRecognition->reRANSACFPFH(true))
        addTextBrowser("Algn", "finish RANSACFPFH.");
      else
        addTextBrowser("Algn", "you didn't get fpfh yet.");
      break;
    case 1:
      addTextBrowser("Algn", "start SACIAFPFH.");
      if (ObjectRecognition->reSACIAFPFH(true))
        addTextBrowser("Algn", "finish SACIAFPFH.");
      else
        addTextBrowser("Algn", "you didn't get fpfh yet.");
      break;
    }
  }
  else
    addTextBrowser("Algn", "you didn't tick the align checkbox.");
}

void MainWindow::on_pushButton_clicked() { refreshPloudCloudVTK(); }

void MainWindow::on_pushButton_regi_clicked()
{
  if (ui->NDT->isChecked())
  {
    addTextBrowser("Regi", "NDT start...");
    if(ObjectRecognition->reNDT(true))
    addTextBrowser("Regi", "NDT finished.");
    else addTextBrowser("Regi", "NDT can't get the cloud properly.");
  }

  if (ui->ICP->isChecked())
  {
    addTextBrowser("Regi", "NDT start...");
    if(ObjectRecognition->reICP(true))
    addTextBrowser("Regi", "ICP finished.");
    else addTextBrowser("Regi", "ICP can't get the cloud properly.");
  }
  refreshPloudCloudVTK();
}
