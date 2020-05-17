/*
 * gui_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cppfile gui related widget visualizer
 */
#include "gui_node.h"
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char* argv[])
{
  //
  // create window object
  //
  QApplication a(argc, argv);
  MainWindow w;
  w.setWindowIcon(QIcon(":/image/icon.ico"));
  //
  // create ros node and pass to window object
  //
//  ros::init(argc, argv, "gui_node");
//  qros* qros_node = new qros();
//  w.qrosObj = qros_node;

  w.show();
  return a.exec();
}
