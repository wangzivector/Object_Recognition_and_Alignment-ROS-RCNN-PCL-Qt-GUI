#include "qsocket.h"

qsocket::qsocket()
{
  //build a socket for the client
  client_c = socket(AF_INET, SOCK_STREAM, 0);
//  //build a socket for the server
  server_s = socket(AF_INET, SOCK_STREAM, 0);

}


qsocket::~qsocket()
{
  close(client_c);
//  close(server_s);
}

bool qsocket::socket_close()
{
  close(client_c);
//  close(server_s);
  client_c = socket(AF_INET, SOCK_STREAM, 0);
  return true;
}

bool qsocket::socket_connect()
{

  //the struct for the address of the server
  struct sockaddr_in adr_s;
  adr_s.sin_family = AF_INET;
  adr_s.sin_addr.s_addr = inet_addr("192.168.1.209");
  adr_s.sin_port =htons(8886);
  std::printf("start to send data ...\n");
//  bind(server_s, (struct sockaddr *)&adr_s, sizeof(adr_s));

  //connect to the socket of the server.
  if(connect(client_c, (struct sockaddr*)&adr_s, sizeof(adr_s))<0)
  {
    std::perror("connect");
    return false;
  }
  return true;
}

cv::Mat qsocket::socket_process(cv::Mat img)
{
  //send the rgb image
//  Mat rgb_sent = imread("./0281.jpg");  //load the 3-channel image
  cv::Mat rgb_sent = img.clone();
  rgb_sent = rgb_sent.reshape(0,1);  //turn Mat into a vector
  int rgbsize_sent = rgb_sent.total()*rgb_sent.elemSize();  //rgbsize = total bytes of the rgb image
  std::printf("sending data: %d\n", rgbsize_sent);

  if(send(client_c, rgb_sent.data, rgbsize_sent, 0)<0)
      std::perror("send rgb");

  //接收彩色图
  std::printf("start to resend data...\n");
  cv::Mat rgb = cv::Mat::zeros(480,640, CV_8UC3);
  int rgbsize = rgb.total()*rgb.elemSize();
  std::printf("rgb.total:%lu, rgb.elemsize():%lu, rgbsize:%d \n", rgb.total(),rgb.elemSize(),  rgbsize);
              //rgb.total:307200, rgb.elemsize():3, rgbsize:921600(个字节),由于每个元素8位,一个字节,因此rgbsize也是图像中总元素的个数
  uchar rgbData[rgbsize];   //每个元素都是一个uchar类型
  int nbytes;
  for(int i = 0; i<rgbsize; i+=nbytes)
  {
      if((nbytes = recv(client_c, rgbData+i, rgbsize-i, 0))==-1)
      {
          std::perror("recv");
          return rgb;
      }
  }

  int ptr = 0;
  //visit the 3-channel rgb
  for(int m = 0; m<rgb.rows; m++)
      for(int n = 0; n<rgb.cols; n++)
      {
          rgb.at<cv::Vec3b>(m,n)[0] = rgbData[ptr+0];
          rgb.at<cv::Vec3b>(m,n)[1] = rgbData[ptr+1];
          rgb.at<cv::Vec3b>(m,n)[2] = rgbData[ptr+2];
          ptr+=3;
      }
  printf("have rece image %zu\n", rgb.total());
  return rgb;

}
