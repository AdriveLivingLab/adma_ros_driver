/**
  * @file adma_connect.cpp
  * @brief This file contains required definitions and functions for
  * receviing information from ADMA sensor with data format Version: 3.3.2.0
  * @authors Lakshman Balasubramanian
  * @date 06/08/2020
  * */


#include "ros/ros.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <adma_connect_imu/Adma.h>
#include <adma_connect_imu/adma_parse.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

// Tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

/** \namespace BOOST UDP link*/
using boost::asio::ip::udp;

/** \brief Length of the stream */
size_t len = 0;
/** \brief Check the timings */
bool performance_check = 0;

/** \brief speed of accessing the data */
#define loopSpeed 100
/// \file
/// \brief  Main function
/// \param  argc An integer argument count of the command line arguments
/// \param  argv An argument vector of the command line arguments
/// \param  loop_rate Set the frequency of publising in Hz
/// \return an integer 0 upon exit success

static inline double getZoneMeridian(const std::string& utm_zone)
{
  int zone_number = std::atoi(utm_zone.substr(0,2).c_str());
  return (zone_number == 0) ? 0.0 : (zone_number - 1) * 6.0 - 177.0;
}
static inline double SQUARE(double x) { return x * x; }


int main(int argc, char **argv)
{
  /* Initialize node */
  ros::init(argc, argv, "adma_connect_pkg");
  ros::NodeHandle nh("~");
  /* Initialize IMU node */
  //ros::NodeHandle nh_imu("imu");
  /* Port number to which ADMA broadcasts */
  /** \get port number list from launch file */
  std::string port_num_ADMA;
  std::string ip_adress_ADMA;
  if(!nh.getParam("port_num_ADMA", port_num_ADMA))
  {
     ROS_INFO("Missing Portnumber (see ADMA_pub_Ethernet.launch file!)");
  }
  if(!nh.getParam("ip_adress_ADMA", ip_adress_ADMA))
  {
     ROS_INFO("Missing IP Adress (see ADMA_pub_Ethernet.launch file!)");
  }

  /** \brief Port Number to which ADMA broadcasts */
  const unsigned short port = static_cast<unsigned short>(std::strtoul(port_num_ADMA.c_str(), NULL, 0));
  /** \brief IP address to which ADMA broadcasts */
  const boost::asio::ip::address address = boost::asio::ip::address::from_string(ip_adress_ADMA);

  /* Initiliaze publisher */
  ros::Publisher  publisher_  = nh.advertise<adma_connect_imu::Adma>("adma_data",1);
  ros::Publisher  raw_publisher_  = nh.advertise<std_msgs::String>("raw_adma_data",1);
  

  /* +++++++++++++++ Aigis +++++++++++++++++++++ */
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  /* +++++++++++++++ Aigis +++++++++++++++++++++ */

  /* Initilaize loop rate */
  ros::Rate loop_rate(loopSpeed);
  /* Create an IO Service wit the OS given the IP and the port */
  boost::asio::io_service io_service;
  /* Establish UDP connection*/
  udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(address, port);
  std::cout << "Local bind " << local_endpoint << std::endl;
  unsigned int seq = 0;
  /* Endless loop until ROS is ok*/
  while (ros::ok())
  {
    seq++;
    /* Socket handling */
    udp::socket socket(io_service);
    socket.open(udp::v4());
    socket.bind(local_endpoint);
    /* The length of the stream from ADMA is 856 bytes */
    boost::array<char, 856> recv_buf;
    udp::endpoint sender_endpoint;
    len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);    
    /* Prepare for parsing */
    std::string local_data(recv_buf.begin(), recv_buf.end());
    /* Load the messages on the publisers */
    std_msgs::String msg_raw_adma;
    msg_raw_adma.data = local_data;
    adma_connect_imu::Adma message;
    getParsedData(local_data,message);
    /* publish the ADMA message */
    // fill timestamp and increment seq counter
    message.header.stamp = ros::Time::now();
    message.header.seq = seq;

    /* Get current time */
    double grab_time = ros::Time::now().toSec();

    if (performance_check)
    {
    char INS_Time_msec[] = {local_data[584],local_data[585],local_data[586],local_data[587]};
    memcpy(&message.INSTimemsec , &INS_Time_msec, sizeof(message.INSTimemsec));
    float weektime = message.INSTimeWeek;
    ROS_INFO("%f ", ((grab_time*1000)-(message.INSTimemsec+1592697600000)));
    }
    message.TimeMsec = ros::Time::now().toSec()*1000;
    message.TimeNsec = ros::Time::now().toNSec();
    
    /* publish ADMA message */
    publisher_.publish(message);
    raw_publisher_.publish(msg_raw_adma);    
    
    
    /* +++++++++++++++ Aigis +++++++++++++++++++++ */
    // publish imu message with DIN 70000 settings
          
    // IMU
    static double orientation_stddev[3];
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
    tf::Quaternion q;
    q.setRPY((double)message.fINSRoll, (double)message.fINSPitch, (double)message.fINSYaw);
    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp = ros::Time::now();
    msg_imu.header.frame_id = "base_link";
    msg_imu.header.seq = seq;
    msg_imu.linear_acceleration.x = message.fAccHorX_1 * 9.81;
    msg_imu.linear_acceleration.y = message.fAccHorX_1 * 9.81;
    msg_imu.linear_acceleration.z = message.fAccHorX_1 * 9.81;
    msg_imu.angular_velocity.x = message.fRateHorX * M_PI / 180;
    msg_imu.angular_velocity.y = message.fRateHorY * M_PI / 180;
    msg_imu.angular_velocity.z = message.fRateHorZ * M_PI / 180;
    msg_imu.orientation.w = q.w();
    msg_imu.orientation.x = q.x();
    msg_imu.orientation.y = q.y();
    msg_imu.orientation.z = q.z();
    orientation_stddev[0] = (double)message.fINSStddevRoll;
    orientation_stddev[1] = (double)message.fINSStddevPitch;
    orientation_stddev[2] = (double)message.fINSStddevYaw;
    msg_imu.orientation_covariance[0] = SQUARE(orientation_stddev[0]); // x
    msg_imu.orientation_covariance[4] = SQUARE(orientation_stddev[1]); // y
    msg_imu.orientation_covariance[8] = SQUARE(orientation_stddev[2]); // z
    pub_imu.publish(msg_imu);
    
  }
    /* Loop rate maintain*/
  ros::spinOnce();
  loop_rate.sleep();
  return 0;
}
