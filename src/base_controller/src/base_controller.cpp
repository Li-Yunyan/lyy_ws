/******************************************************************
基于串口通信的ROS航站楼服务机器人基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制机器人的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为pr/min
（2）格式：
2.读取串口
（1）内容：
（2）格式：
*******************************************************************/
#include "ros/ros.h" //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>     //里程计消息
#include <tf/transform_broadcaster.h>
//以下为串口通讯需要的头文件
#include "serial/serial.h"
#include "yaml-cpp/yaml.h"         //用来解析和生成yaml文件
#include <boost/filesystem.hpp>    //描述了C++程序可用于执行涉及文件系统的操作的组件，包括路径、常规文件和目录
#include <cstdio>                  //将stdio.h的内容用C++头文件的形式表示出来
#include <fstream>                 //输入输出到指定文件，或者从指定文件中读出数据
#include <iostream>                //定义了标准输入/输出流对象
#include <math.h>
#include <string>
#include <unistd.h>                //POSIX标准定义的unix类系统定义符号常量的头文件，包含许多UNIX系统服务的函数原型

#define HAVE_YAMLCPP_GT_0_5_0
/****************************************************************************/
using std::string;
using std::exception;              //C++异常处理
using std::cout;                   //C++标准输出
using std::cerr;                   //C++标准错误输出
using std::endl;                   //C++标准库中的操控器
using std::vector;                 //vector表示可以改变大小的数组的序列容器
using std::ifstream;               //c++文件操作
/*****************************************************************/
int motor_model = 0, serial_model = 0;       //电机模式 串口模式
float ratio = 1000.0f;                       //转速转换比例，执行速度调整比例
float D = 0.6f;                              //两轮间距，单位是m
float S = 0.2f;                              //轮子直径，单位m
float linear_temp = 0, angular_temp = 0;     //暂存的线速度和角速度
float SGAT = 0.1;                            //控制采样时间
/****************************************************/
string left_serial_port;                     //左轮串口号
int left_serial_baud;                        //左轮波特率
string right_serial_port;                    //右轮串口号
int right_serial_baud;                       //右轮波特率

/*****************************************************/
unsigned char left_speed_data_serial[8] = {0x02, 0x00, 0xc4, 0xc6,        //速度模式
                                           0x0a, 0x0a, 0x0a, 0x1e};       //加速度，加/减速时间10×100ms
unsigned char right_speed_data_serial[8] = {0x02, 0x00, 0xc4, 0xc6,
                                            0x0a, 0x0a, 0x0a, 0x1e};
unsigned char left_speedall_16[4] = {0x06, 0x00, 0x00, 0x00};             //速度量，设置转速0
unsigned char right_speedall_16[4] = {0x06, 0x00, 0x00, 0x00};            //数据 =（0RPM/3000×8192）换算为16进制

unsigned char unable_motor_data[4] = {0x00, 0x00, 0x00, 0x00};            //电机停止
unsigned char enable_motor_data[4] = {0x00, 0x00, 0x01, 0x01};            //电机使能
unsigned char check_motor_data[3] = {0x80, 0x00, 0x80};                   //读监控参数
bool have_enabled_motor = 0;

unsigned char rec_buffer_left[32], rec_buffer_right[32];                  //串口数据接收变量
// string rec_buffer;  

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData                                  //union的作用为实现char数组和float之间的转换
{
  float d;
  unsigned char data[4];
} right_speed_data, left_speed_data;
/************************************************************/
struct wheelstate {                              //声明结构体，轮子状态
  float position_x;
  float position_y;
  float oriention;
  float vel_linear;
  float vel_angular;
};

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
// YAML-CPP 0.5中缺少>>运算符，因此此函数为增加了对YAML-CPP 0.3 API下编写代码的支持
template <typename T> void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}
#endif

// 读取机器人模型参数
class wheelModel {
public:
  /** Trivial constructor */
  wheelModel(const string &fname, float &_ratio, float &_D, float &_S,
             int &_serial_model) {
    float ratio, D, S;
    int serial_model;
    ifstream fin(fname.c_str());                 //读取文件fname的内容
    if (fin.fail()) {
      ROS_ERROR("param could not open %s.", fname.c_str());
      exit(-1);
    }

#ifdef HAVE_YAMLCPP_GT_0_5_0
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);            //文件加载
#else
    YAML::Parser parser(fin);                    //文件解析
    YAML::Node doc;
    parser.GetNextDocument(doc);                 //发布解析文件
#endif

    try {                                        //检查相关参数是否存在，不存在则报错
      doc["ratio"] >> ratio;
    } catch (YAML::InvalidScalar &) {
      ROS_ERROR(
          "The wheel model does not contain a ratio tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["D"] >> D;
    } catch (YAML::InvalidScalar &) {
      ROS_ERROR("The wheel model does not contain a D tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["S"] >> S;
    } catch (YAML::InvalidScalar &) {
      ROS_ERROR("The wheel model does not contain a S tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["serial_model"] >> serial_model;
    } catch (YAML::InvalidScalar &) {
      ROS_ERROR("The wheel model does not contain a serial_model tag or it is "
                "invalid.");
      exit(-1);
    }

    _ratio = ratio;
    _D = D;
    _S = S;
    _serial_model = serial_model;
  }
};

/*// 16进制转10进制                                 
unsigned long HextoDec(const unsigned char *hex, int length) {   //hex：0x16  dec：22
  int i;                                                         //i=0，rslt1 = hex[0] << 8
  unsigned long rslt = 0;                                        //i=1，rslt2 = hex[1] << 0
  for (i = 0; i < length; i++) {
    rslt += (unsigned long)(hex[i]) << (8 * (length - 1 - i));
  }
  return rslt;
}*/

// 10进制转16进制
int DectoHex(int dec, unsigned char *hex, int length) {
  int i;
  for (i = length - 1; i >= 0; i--) {
    hex[i] = (dec % 256) & 0xFF;
    dec /= 256;
  }
  return 0;
}

//计算车底盘状态
void getwheelstate(unsigned char rec_buffer_left[32],
                   unsigned char rec_buffer_right[32], wheelstate &ws) {

  if (rec_buffer_left[12] == 0xe4 && rec_buffer_right[12] == 0xe4) {     //输出转速
    ROS_INFO("left:%x,%x,right:%x,%x",rec_buffer_left[13],rec_buffer_left[14],rec_buffer_right[13],rec_buffer_right[14]);
    int left = 0x00;
    int a = (rec_buffer_left[13] >= 0xf0) ? -1 : 1;                      //不等式成立将a赋值为-1
    // ROS_INFO("a:%d",a);
    left += int(rec_buffer_left[13]);
    left = left << 8;
    left += int(rec_buffer_left[14]);                                    //left为16进制数
    if (a == -1) {
      left = 0x00ffff - left;
    }

    int right = 0x00;
    int b = (rec_buffer_right[13] >= 0xf0) ? -1 : 1;
    right += int(rec_buffer_right[13]);
    right = right << 8;
    right += int(rec_buffer_right[14]);
    ROS_INFO("left:%x,right:%x",left,right);
    if (b == -1) {
      right = 0x00ffff - right;
    }

    // ROS_INFO("left:%d,right:%d",left,right);
    float _right = right * 6000 / 16384;                                 //right为数值，_right为转速
    float _left = left * 6000 / 16384;

    _right = _right * S * 3.141592 / 60;                                 //将转速rad/min转换为前进距离m
    _left = _left * S * 3.141592 / 60;
    if (a == -1) {
      _left = -_left;
    }
    if (b == 1) {                                                        //？？？
      _right = -_right;
    }
    // ROS_INFO("left:%f,right:%f",_left,_right);
    ws.vel_linear = (_right + _left) / 2;                                //计算线速度
    ws.vel_angular = (_right - _left) / D;                               //计算角速度
    float th = 0;                                                        //角度
    if (ws.vel_angular < 0)
      th = (_right - _left) * (SGAT + 0.122) / D;
    else
      th = (_right - _left) * (SGAT + 0.152) / D;
    float r = (_left + _right) / 2 * th;
    ws.oriention += th;                                                  //方向
    float delta_x = ws.vel_linear * (SGAT + 0.13) * cos(ws.oriention);   //x方向的线速度
    float delta_y = ws.vel_linear * (SGAT + 0.11) * sin(ws.oriention);   //y方向的线速度
    ws.position_x += delta_x;                                            //位置
    ws.position_y += delta_y;
    // ws.position_x=0;
    // ws.position_y=0;
    // ws.vel_linear=0;
    // ws.vel_angular=0;
    // ws.oriention=0;
    //  ROS_INFO("%f,%f,%f,%f,%f,",ws.position_x,ws.position_y,ws.vel_linear,ws.vel_angular,ws.oriention);
  } else {
    // ROS_WARN("wheelstate is wrong!");
  }
}

void callback(const geometry_msgs::Twist &cmd_input) //订阅/cmd_vel主题回调函数
{

  angular_temp = cmd_input.angular.z;                                   //获取/cmd_vel的角速度,rad/s
  linear_temp = cmd_input.linear.x;                                     //获取/cmd_vel的线速度.m/s

  //将转换好的小车速度分量为左右轮速度
  left_speed_data.d = linear_temp - 0.5f * angular_temp * D;
  right_speed_data.d = linear_temp + 0.5f * angular_temp * D;

  //存入数据到要发布的左右轮速度消息
  left_speed_data.d = left_speed_data.d * 60 / S / 3.141592;             //转换为转速 rp/min
  right_speed_data.d = right_speed_data.d * 60 / S / 3.141592;
  // ROS_INFO("%f",left_speed_data.d);
  /**************************************************/
  serial::Serial left_serial(
      left_serial_port, left_serial_baud,
      serial::Timeout::simpleTimeout(10));                               //左电机配置串口
  serial::Serial right_serial(
      right_serial_port, right_serial_baud,
      serial::Timeout::simpleTimeout(10));                               //右电机配置串口
  ros::param::get("/base_controller/motor_model", motor_model);          //获取并将参数赋值给motor_model

  if (motor_model != 1) {
    ROS_ERROR("motor model is wrong");
    exit(-1);
  }
  if (motor_model == 1) {
    bool enable_motor = 1;
    // bool enable_motor = 0;
    // ros::param::get("/base_controller/enable_motor", enable_motor);

    if (enable_motor) {
      if (!have_enabled_motor) {
        left_serial.write(enable_motor_data, 4);                         //写入电机使能指令
        right_serial.write(enable_motor_data, 4);
        ros::Duration(0.01).sleep();
        if (left_serial.available() && right_serial.available()) {
          unsigned char rec_buffer[2];
          left_serial.read(rec_buffer, 2);                               //获取串口发送来的数据
          right_serial.read(rec_buffer, 2);                              //获取串口发送来的数据
        } else {
          ROS_WARN("serial4 is wrong!");
        }
        have_enabled_motor = 1;
      }

      left_speed_data.d = left_speed_data.d / 3000 * 8192;               //转速转换为数值
      right_speed_data.d = -right_speed_data.d / 3000 * 8192;
      // unsigned char
      // left_speed_data[8]={0x02,0x00,0xc4,0xc6,0x0a,0x14,0x14,0x32};//速度模式
      // 加速度
      // unsigned char
      // right_speed_data[8]={0x02,0x00,0xc4,0xc6,0x0a,0x14,0x14,0x32};
      ROS_INFO("left_speed: %f", left_speed_data.d);                     //手动设置左/右轮速度
      ROS_INFO("right_speed: %f", right_speed_data.d);
      int left_data_first = (left_speed_data.d);
      // int left_data_second=(left_speed_data.d);
      int right_data_first = (right_speed_data.d);
      // int right_data_second=(right_speed_data.d);
      // left_data_first=left_data_first/100;
      // left_data_second=left_data_second%100;
      // right_data_first=right_data_first/100;
      // right_data_second=right_data_second%100;
      unsigned char left_speed_16_first[2] = {0};
      unsigned char right_speed_16_first[2] = {0};
      // unsigned char left_speed_16_second[2]={0};
      // unsigned char right_speed_16_second[2]={0};
      DectoHex(abs(left_data_first), left_speed_16_first, 2);            //十进制转十六进制
      DectoHex(abs(right_data_first), right_speed_16_first, 2);
      // DectoHex(left_data_second,left_speed_16_second,2);
      // DectoHex(right_data_second,right_speed_16_second,2);
      int m = 0x00ffff;
      int n = 0;
      if (left_data_first < 0) {                                         //设置的十进制数小于0
        n += int(left_speed_16_first[0]) << 8;
        n += int(left_speed_16_first[1]);
        m -= n;
        left_speed_16_first[0] = (m >> 8) & 0xff;                        //得到十六进制数
        left_speed_16_first[1] = (m & 0xff);
      }
      m = 0xffff;
      n = 0;
      if (right_data_first < 0) {
        n += int(right_speed_16_first[0]) << 8;
        n += int(right_speed_16_first[1]);
        m -= n;
        right_speed_16_first[0] = (m >> 8) & 0xff;
        right_speed_16_first[1] = (m & 0xff);
      }

      left_speedall_16[1] = left_speed_16_first[0];                      //设置转速
      left_speedall_16[2] = left_speed_16_first[1];
      right_speedall_16[1] = right_speed_16_first[0];
      right_speedall_16[2] = right_speed_16_first[1];
      left_speedall_16[3] =
          left_speedall_16[0] + left_speedall_16[1] + left_speedall_16[2];
      right_speedall_16[3] =
          right_speedall_16[0] + right_speedall_16[1] + right_speedall_16[2];
      // unsigned char enable_motor[4]={0x00,0x00,0x01,0x01};
      left_serial.write(left_speedall_16, 4);                            //将设置的转速指令写到串口
      right_serial.write(right_speedall_16, 4);
      ros::Duration(0.05).sleep();
      if (left_serial.available() && right_serial.available()) {
        unsigned char rec_buffer[16];
        left_serial.read(rec_buffer, 16);                                //获取串口发送来的数据
        right_serial.read(rec_buffer, 16);                               //获取串口发送来的数据
      } else {
        ROS_WARN("serial1 is wrong!");
      }
      left_serial.write(check_motor_data, 3);                            //将监控参数的指令写到串口
      right_serial.write(check_motor_data, 3);
      ros::Duration(0.1).sleep();
      if (left_serial.available() && right_serial.available()) {
        left_serial.read(rec_buffer_left, 32);                           //获取串口发送来的数据
        right_serial.read(rec_buffer_right, 32);                         //获取串口发送来的数据
                                                 /*for(int i=0;i<32;i++){
                                                       printf("%x ",rec_buffer_left[i]);
                                                   }
                                                   printf("\n\n\n");
                                                   for(int i=0;i<32;i++){
                                                       printf("%x ",rec_buffer_right[i]);
                                                   }
                                                   printf("\n\n\n");*/
      } else {
        ROS_WARN("serial2 is wrong!");
      }
    } else {                                                              //enable_motor ！= 1

      ROS_INFO("motor_closed");
      left_serial.write(unable_motor_data, 4);                            //将停止电机指令写到串口
      right_serial.write(unable_motor_data, 4);

      ros::Duration(0.05).sleep();
      if (left_serial.available() && right_serial.available()) {
        unsigned char rec_buffer[2];
        left_serial.read(rec_buffer, 2);                                  //获取串口发送来的数据
        right_serial.read(rec_buffer, 2);                                 //获取串口发送来的数据
      } else {
        ROS_WARN("serial3 is wrong!");
      }

      have_enabled_motor = 0;
    }
  }

  //串口指令

  /**************************************************/
}

int main(int argc, char **argv) {
  ROS_INFO("1");
  ros::init(argc, argv, "base_controller");                               //初始化串口节点
  ROS_INFO("2");
  if (argc != 2) {                                                        //检测yaml
    ROS_WARN(" Using deprecated base_controller interface ");
    ROS_INFO("3");
  }
  string fname(argv[1]);                                                  //读取YAML
  wheelModel wm(fname, ratio, D, S, serial_model);                        //轮子模型写入
  ROS_INFO("4");
  ros::NodeHandle n;                                                      //定义节点进程句柄

  //获取电机串口信息
  ros::param::get("/base_controller/serial_param/left_serial_port",
                  left_serial_port);
  ros::param::get("/base_controller/serial_param/left_serial_baud",
                  left_serial_baud);
  ros::param::get("/base_controller/serial_param/right_serial_port",
                  right_serial_port);
  ros::param::get("/base_controller/serial_param/right_serial_baud",
                  right_serial_baud);
  ros::param::get("/base_controller/motor_model", motor_model);
  ros::param::get("/base_controller/SGAT", SGAT);
  ROS_INFO("5");
  //串口模式选择  1:232模式
  if (serial_model != 1) {
    ROS_ERROR("serial model is wrong");
    exit(-1);
  }
  ROS_INFO("5");
  if (serial_model == 1 && motor_model == 1) {
  //   serial::Serial left_serial(
  //     left_serial_port, left_serial_baud,
  //     serial::Timeout::simpleTimeout(1000));                              //左电机配置串口
  // serial::Serial right_serial(
  //     right_serial_port, right_serial_baud,
  //     serial::Timeout::simpleTimeout(1000));                              //右电机配置串口
    ROS_INFO("6");
  }
  // string left_serial_port("/dev/ttyUSB0");
  // unsigned long left_serial_baud=115200;
  serial::Serial left_serial(
      left_serial_port, left_serial_baud,
      serial::Timeout::simpleTimeout(1000));    
        ROS_INFO("7");                          //左电机配置串口
  serial::Serial right_serial(
      right_serial_port, right_serial_baud,
      serial::Timeout::simpleTimeout(1000));                              //右电机配置串口

  left_serial.write(left_speed_data_serial, 8);                           //设置为速度模式，并设置加/减速度，写入指令
  right_serial.write(right_speed_data_serial, 8);
  ros::Duration(0.01).sleep();
  if (left_serial.available() && right_serial.available()) {
    unsigned char rec_buffer[4];
    left_serial.read(rec_buffer, 4);                                      //获取串口发送来的数据
    right_serial.read(rec_buffer, 4); 
      ROS_INFO("8");                                    //获取串口发送来的数据
  } else {
    ROS_WARN("serial is wrong!");
  }

  int controller_model = 0;
  ros::param::get("controller_model", controller_model);                  //获取控制模式
  ros::Subscriber sub;
  if (controller_model == 1) {                                            //手动模式
    sub = n.subscribe("/cmd_vel", 1, callback); 
      ROS_INFO("9");                           //订阅/cmd_vel主题
  }

  wheelstate ws;                                                          //初始化车状态
  ws.vel_linear = 10;
  ws.vel_angular = 0;
  ws.position_x = 0;
  ws.position_y = 0;
  ws.oriention = 0;

  ros::Publisher odom_pub =
      n.advertise<nav_msgs::Odometry>("odom", 20);                        //定义要发布/odom主题
  // static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
  //  geometry_msgs::TransformStamped
  //  odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
  nav_msgs::Odometry odom;                                                //定义里程计对象
  geometry_msgs::Quaternion odom_quat;                                    //四元数变量
  
  //定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
  float covariance[36] = {
      0.01, 0,    0,     0,     0,     0,                                 // covariance on gps_x
      0,    0.01, 0,     0,     0,     0,                                 // covariance on gps_y
      0,    0,    99999, 0,     0,     0,                                 // covariance on gps_z
      0,    0,    0,     99999, 0,     0,                                 // large covariance on rot x
      0,    0,    0,     0,     99999, 0,                                 // large covariance on rot y
      0,    0,    0,     0,     0,     0.01};                             // large covariance on rot z
  
  //载入covariance矩阵
  for (int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = covariance[i];
  }

  ros::Rate loop_rate(int(1 / SGAT)); 
    ROS_INFO("10");                                    //设置周期休眠时间
  while (ros::ok()) {
    // ros::Time begin = ros::Time::now();

    ros::spinOnce();                                                      //callback函数必须处理所有问题时，才可以用到
    // ros::Time end = ros::Time::now();
    // ROS_INFO("%f ", (begin-end).toSec ());
    getwheelstate(rec_buffer_left, rec_buffer_right, ws);                 //调用getwheelstate函数
    // ROS_INFO("%s",);
      ROS_INFO("11");
    for (int i = 0; i < 32; ++i) {
      rec_buffer_left[i] = 0;
      rec_buffer_right[i] = 0;
    }
    odom_quat =
        tf::createQuaternionMsgFromYaw(ws.oriention); //将偏航角转换成四元数
                                                      /*
                                                              //载入坐标（tf）变换时间戳
                                                              odom_trans.header.stamp = ros::Time::now();
                                                              //发布坐标变换的父子坐标系
                                                              odom_trans.header.frame_id = "odom";
                                                              odom_trans.child_frame_id = "base_footprint";
                                                              //tf位置数据：x,y,z,方向
                                                              odom_trans.transform.translation.x = ws.position_x;
                                                              odom_trans.transform.translation.y = ws.position_y;
                                                              odom_trans.transform.translation.z = 0.0;
                                                              odom_trans.transform.rotation = odom_quat;
                                                              //发布tf坐标变化
                                                              odom_broadcaster.sendTransform(odom_trans);
                                                      */
    //载入里程计时间戳
    odom.header.stamp = ros::Time::now();
    //里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = ws.position_x;
    odom.pose.pose.position.y = ws.position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //载入线速度和角速度
    odom.twist.twist.linear.x = ws.vel_linear;
    // odom.twist.twist.linear.y = odom_vy;
    odom.twist.twist.angular.z = ws.vel_angular;
    //发布里程计
    odom_pub.publish(odom);
    loop_rate.sleep();                                 //周期休眠
  }
  //程序周期性调用

  // ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到

  return 0;
}