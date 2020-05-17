/*
 * mainwindow.h
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This file for mainwindow.cpp
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qvtk.h"
#include "qros.h"
#include "objreco.h"
//#include "pcd_io.h"
//#include "qpcl.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  /// ros pointer for publish msg
  qros* qrosObj;
  ObjReco* ObjectRecognition;
//  pcd_io* pcd_ioObj;

private slots:
  void on_pushButton_cl_clicked();

private slots:
  void on_comboBox_wo_currentIndexChanged(const QString &arg1);

private slots:
  void on_actionReset_Params_triggered();

private slots:
  void on_pushButton_sa_clicked();

private slots:
  void on_pushButton_lo_clicked();

private slots:
  void on_pushButton_co_clicked();

private slots:
  void on_pushButton_ex_clicked();
  void on_tabWidget_tabBarClicked(int index);
  void on_pushButton_quit_clicked();
  void on_pushButton_pc_clicked();

private:
  void addTextBrowser(QString text);
  void reloadParamWidget();
  qvtk* qvtkWidgetObj;
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
