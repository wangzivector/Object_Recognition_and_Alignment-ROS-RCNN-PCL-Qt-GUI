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
#include "pcd_io.h"

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
  pcd_io* pcd_ioObj;

private slots:
  void on_pushButton_quit_clicked();

private slots:
  void on_pushButton_pc_clicked();

private:
  qvtk* qvtkWidgetObj;
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
