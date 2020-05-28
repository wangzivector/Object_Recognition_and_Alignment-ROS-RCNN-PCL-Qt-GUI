/*
 * qros.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * object-based implement of qros. ros stuff
 * but this is for GUI
 */
#include "qros.h"

qros::qros() { ROS_INFO("Hello here creates qros node!"); }

qros::~qros() {}

bool qros::qrosPublish(QString info)
{

  //
  // publish
  //
  info_pub.infomation = info.toStdString().c_str();
  pub_info.publish(info_pub);
  return true;
}
