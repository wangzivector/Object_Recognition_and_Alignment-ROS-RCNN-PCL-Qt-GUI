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


typedef unsigned char uchar;


class qsocket
{
public:
  qsocket();
  ~qsocket();
  bool socket_connect();
  bool socket_close();
  cv::Mat socket_process(cv::Mat img);

  int client_c;
  int server_s;
};

#endif // QSOCKET_H
