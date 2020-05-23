/*
 * mainwindow.h
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This file for mainwindow.cpp
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "objreco.h"
#include "qros.h"
#include "qvtk.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QTableView>
#include <QStandardItemModel>
#include <QTime>


//#include "pcd_io.h"
//#include "qpcl.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  void refreshPloudCloudVTK();
  void tableDisplay();
  /// ros pointer for publish msg
  qros* qrosObj;
  ObjReco* ObjectRecognition;
  QStandardItemModel* table;
  QTime current_time;
  //  pcd_io* pcd_ioObj;


private slots:

  void on_spinBox_sac_1_valueChanged(int arg1);
  void on_spinBox_sac_2_valueChanged(int arg1);
  void on_spinBox_sac_3_valueChanged(double arg1);
  void on_spinBox_sac_4_valueChanged(double arg1);
  void on_spinBox_sac_5_valueChanged(int arg1);
  void on_spinBox_sac_6_valueChanged(double arg1);
  void on_spinBox_sac_7_valueChanged(int arg1);
  void on_spinBox_sac_8_valueChanged(double arg1);
  void on_spinBox_sac_9_valueChanged(double arg1);

  void on_spinBox_ran_1_valueChanged(int arg1);
  void on_spinBox_ran_2_valueChanged(int arg1);
  void on_spinBox_ran_3_valueChanged(int arg1);

  void on_spinBox_ran_4_valueChanged(double arg1);
  void on_spinBox_ran_5_valueChanged(double arg1);
  void on_spinBox_ran_6_valueChanged(double arg1);
  void on_spinBox_ran_7_valueChanged(double arg1);
  void on_spinBox_ran_8_valueChanged(double arg1);
  void on_spinBox_ran_9_valueChanged(double arg1);
  void on_spinBox_ran_10_valueChanged(int arg1);

  void on_spinBox_seg_1_valueChanged(int arg1);
  void on_spinBox_seg_2_valueChanged(int arg1);
  void on_spinBox_seg_3_valueChanged(int arg1);
  void on_spinBox_seg_4_valueChanged(int arg1);
  void on_spinBox_seg_5_valueChanged(double arg1);
  void on_spinBox_seg_6_valueChanged(double arg1);
  void on_spinBox_seg_7_valueChanged(double arg1);

  void on_spinBox_filter8_valueChanged(double arg1);
  void on_spinBox_filter7_valueChanged(double arg1);
  void on_spinBox_filter6_valueChanged(int arg1);
  void on_spinBox_filter5_valueChanged(double arg1);
  void on_spinBox_filter4_valueChanged(int arg1);
  void on_spinBox_filter3_valueChanged(double arg1);
  void on_spinBox_filter2_valueChanged(double arg1);
  void on_spinBox_filter1_valueChanged(double arg1);

  void on_comboBox_wo_currentIndexChanged(const QString& arg1);
  void on_comboBox_mo_currentIndexChanged(const QString &arg1);
  void on_checkBox_dealw_clicked(bool checked);
  void on_checkBox_dealo_clicked(bool checked);

  void on_pushButton_al_clicked();
  void on_pushButton_do_clicked();
  void on_pushButton_fe_clicked();
  void on_pushButton_di_clicked();
  void on_pushButton_pr_clicked();
  void on_pushButton_cl_clicked();
  void on_pushButton_sa_clicked();
  void on_pushButton_lo_clicked();
  void on_pushButton_co_clicked();
  void on_pushButton_ex_clicked();
  void on_pushButton_quit_clicked();
  void on_pushButton_sh_clicked();
  void on_actionReset_Params_triggered();
  void on_pushButton_clicked();
  void on_pushButton_regi_clicked();

private:
  void addTextBrowser(QString head_text, QString text);
  void reloadParamWidget();
  qvtk* qvtkWidgetObj;
  Ui::MainWindow* ui;
};

#endif // MAINWINDOW_H
