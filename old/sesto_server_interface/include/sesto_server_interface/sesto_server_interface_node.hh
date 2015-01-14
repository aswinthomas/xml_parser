
#ifndef _SESTO_SERVER_INTERFACE_NODE_HH
#define _SESTO_SERVER_INTERFACE_NODE_HH

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <stdio.h>
#include <angles/angles.h>
#include <math.h>

using namespace std;


class SestoServerInterfaceNode {
private:
  static const double LOCATION_TELEMETRY_PUBLISH_DELAY = 0.5; //in sec

  double dataTimestamp,lastLocationTelemetryPublishTime;
  geometry_msgs::PoseWithCovariance currPos;

  unsigned short robotId,serverId;

  string data_buffer;
  int state;
  void extractRouteMsg(string buffer);
  unsigned int rcvdTotalLength;
  unsigned short rcvdMsgId;
  int multicastSendCount,TCPSendCount, TCPReceiveCount;

public:
  ///subscribe and advertise
  ros::Publisher pUDPWrite,pTCPWrite;
  ///Construct and Destroy!
  /**
   */
  SestoServerInterfaceNode();
  /**
   * \brief   Destroys this instance.
   */
  ~SestoServerInterfaceNode(){ }

  void fromServerCallback(const std_msgs::String::ConstPtr& msg);
  void setCurrPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void initialize();

  int task();
  void publishMsgs();
};

#endif
