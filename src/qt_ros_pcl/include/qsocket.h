#ifndef QSOCKET_H
#define QSOCKET_H
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

// OpenCV
#include <opencv2/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/nonfree/nonfree.hpp>

typedef unsigned char uchar;


class qsocket
{
public:
  qsocket();
  bool socket_connect();
  cv::Mat socket_process(cv::Mat img);

  int client_c;
  int server_s;
};

#endif // QSOCKET_H
