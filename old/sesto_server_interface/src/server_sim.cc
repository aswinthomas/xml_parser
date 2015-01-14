#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sesto_server_interface/ROSUtil.hh>
#include <boost/thread.hpp>

ros::Publisher pTCPWrite,pMulticastWrite;
int serverId = 0;
int robotId = 1;
using namespace ROSUtil;
boost::shared_ptr<boost::thread> taskThread;

unsigned int rcvdTCPTotalLength=0, rcvdMulticastTotalLength=0;
unsigned short rcvdTCPMsgId=0, rcvdMulticastMsgId=0;
string data_bufferTCP,data_bufferMulticast;
int stateTCP=0,stateMulticast=0;
int telemetryIndex=0,ackIndex=0;
unsigned short planId=0,startId=0;

/*
void sendPause() {
  string buffer;
  char pause=0x01;
  buffer.append(1,pause); buffer.append(1,spare);
  pWrite.publish(packMsgToSend(serverId,robotId,buffer,headerMsg.navModeMsgId));
}

void sendStop() {
  string buffer;
  char pause=0x00;
  buffer.append(1,pause); buffer.append(1,spare);
  pWrite.publish(packMsgToSend(serverId,robotId,buffer,headerMsg.navModeMsgId));
}

void sendStart() {
  string buffer;
  char pause=0x02;
  buffer.append(1,pause); buffer.append(1,spare);
  pWrite.publish(packMsgToSend(serverId,robotId,buffer,headerMsg.navModeMsgId));
}
*/
void sendRouteData() {
  while(1) {
    planId++;
    double locX,locY,locYaw;
    unsigned short txtMsgId=headerMsg.routeMsgId;
    string buffer,noOfTargetsStr,locXStr,locYStr,locYawStr;
    unsigned short noOfTargets=3;
    noOfTargetsStr=toBytes<unsigned short>(noOfTargets);
    buffer=noOfTargetsStr;

    locX=1.564; locY=22.7655; locYaw=0.85;
    locXStr=toBytes<double>(locX); locYStr=toBytes<double>(locY); locYawStr=toBytes<double>(locYaw);
    buffer+=locXStr+locYStr+locYawStr; 
    locX=45.345; locY=12.7562; locYaw=1.43272;
    locXStr=toBytes<double>(locX); locYStr=toBytes<double>(locY); locYawStr=toBytes<double>(locYaw);
    buffer+=locXStr+locYStr+locYawStr; 
    locX=103.2465; locY=1.743654; locYaw=3.14562;
    locXStr=toBytes<double>(locX); locYStr=toBytes<double>(locY); locYawStr=toBytes<double>(locYaw);
    buffer+=locXStr+locYStr+locYawStr; 

    pTCPWrite.publish(packMsgToSend(serverId,robotId,buffer,txtMsgId));
    ROS_INFO("Route plan Sent: %d",planId);
    sleep(1);
  }

}

void extractLocalizationData(string buffer) {
  string posXStr,posYStr,posYawStr;
  int index=0;
  posXStr = buffer.substr(headerMsg.headerLength+index,DOUBLE_SIZE); index+=DOUBLE_SIZE;
  posYStr = buffer.substr(headerMsg.headerLength+index,DOUBLE_SIZE); index+=DOUBLE_SIZE;
  posYawStr = buffer.substr(headerMsg.headerLength+index,DOUBLE_SIZE); index+=DOUBLE_SIZE;

  double posX = bytesTo<double>(posXStr);
  double posY = bytesTo<double>(posYStr);
  double posYaw = bytesTo<double>(posYawStr);
  telemetryIndex++;
  ROS_INFO("ind:%d posX:%.2f posY:%.2f posYaw:%.2f",telemetryIndex,posX,posY,posYaw);
}

void extractACKData(string buffer) {
  ackIndex++;
  string idStr = buffer.substr(headerMsg.headerLength,SHORT_SIZE);
  unsigned short id = bytesTo<unsigned short>(idStr);

  ROS_INFO("ackIndex:%d Got ACK for Message:%d",ackIndex,id);
}

void readTCPData(const std_msgs::StringConstPtr msg) {
  int buffer_index=0, bytes_read = msg->data.length();
  string buffer=msg->data;

  while(bytes_read > 0){
    switch(stateTCP){
      case 0:
        if(buffer[buffer_index]==headerMsg.syncByte1){ stateTCP=1; data_bufferTCP+=buffer[buffer_index];}
        else { data_bufferTCP.clear(); stateTCP=0;}
        bytes_read--; buffer_index++;
        break;
      case 1:
        if(buffer[buffer_index]==headerMsg.syncByte2){ stateTCP=2; data_bufferTCP+=buffer[buffer_index];}
        else { data_bufferTCP.clear(); stateTCP=0;}
        bytes_read--; buffer_index++;
        break;
      case 2:
        if(buffer[buffer_index]==headerMsg.syncByte3){ stateTCP=3; data_bufferTCP+=buffer[buffer_index];}
        else { data_bufferTCP.clear(); stateTCP=0;}
        bytes_read--; buffer_index++;
        break;
      case 3:
        if(data_bufferTCP.length()<headerMsg.headerLength){ data_bufferTCP+=buffer[buffer_index];}
        if(data_bufferTCP.length()==headerMsg.headerLength) stateTCP = 4;
        bytes_read--; buffer_index++;
        break;
      case 4:{
        string projIdStr,msgIdStr,totLenStr,sourceIdStr,destinationIdStr;
        unsigned short rcvdProjId=0,destinationId=0,sourceId=0;
        sourceIdStr=data_bufferTCP.substr(8,SHORT_SIZE); destinationIdStr=data_bufferTCP.substr(10,SHORT_SIZE);
        msgIdStr=data_bufferTCP.substr(12,SHORT_SIZE); projIdStr=data_bufferTCP.substr(14,SHORT_SIZE);
        totLenStr=data_bufferTCP.substr(16,INT_SIZE);
        rcvdTCPMsgId=bytesTo<unsigned short>(msgIdStr); rcvdProjId=bytesTo<unsigned short>(projIdStr);
        sourceId = bytesTo<unsigned short>(sourceIdStr);
        rcvdTCPTotalLength=bytesTo<unsigned int>(totLenStr); destinationId=bytesTo<unsigned short>(destinationIdStr);
        //ROS_INFO("MsgId:%d", rcvdMsgId);
        if(verifyHeaderChecksum(data_bufferTCP) && rcvdProjId==headerMsg.projectId) {
          stateTCP=5;
        } else { data_bufferTCP.clear(); stateTCP=0; ROS_INFO("TCP Header Checksum Error");}
      }
      break;
      case 5:
        if(data_bufferTCP.length()<rcvdTCPTotalLength) {
          data_bufferTCP+=buffer[buffer_index]; bytes_read--; buffer_index++;
        }
        if(data_bufferTCP.length()==rcvdTCPTotalLength) {
          if(verifyMessageChecksum(data_bufferTCP)) {
            if(rcvdTCPMsgId==headerMsg.ackMsgId) { extractACKData(data_bufferTCP);}
          } else ROS_INFO("TCP Data Checksum Error");
          data_bufferTCP.clear(); stateTCP=0;
        }
        break;
    }
  }
}


void readMulticastData(const std_msgs::StringConstPtr msg) {
  int buffer_index=0, bytes_read = msg->data.length();
  string buffer=msg->data;

  while(bytes_read > 0){
    switch(stateMulticast){
      case 0:
        if(buffer[buffer_index]==headerMsg.syncByte1){ stateMulticast=1; data_bufferMulticast+=buffer[buffer_index];}
        else { data_bufferMulticast.clear(); stateMulticast=0;}
        bytes_read--; buffer_index++;
        break;
      case 1:
        if(buffer[buffer_index]==headerMsg.syncByte2){ stateMulticast=2; data_bufferMulticast+=buffer[buffer_index];}
        else { data_bufferMulticast.clear(); stateMulticast=0;}
        bytes_read--; buffer_index++;
        break;
      case 2:
        if(buffer[buffer_index]==headerMsg.syncByte3){ stateMulticast=3; data_bufferMulticast+=buffer[buffer_index];}
        else { data_bufferMulticast.clear(); stateMulticast=0;}
        bytes_read--; buffer_index++;
        break;
      case 3:
        if(data_bufferMulticast.length()<headerMsg.headerLength){ data_bufferMulticast+=buffer[buffer_index];}
        if(data_bufferMulticast.length()==headerMsg.headerLength) stateMulticast = 4;
        bytes_read--; buffer_index++;
        break;
      case 4:{
        string projIdStr,msgIdStr,totLenStr,sourceIdStr,destinationIdStr;
        unsigned short rcvdProjId=0,destinationId=0,sourceId=0;
        sourceIdStr=data_bufferMulticast.substr(8,SHORT_SIZE); destinationIdStr=data_bufferMulticast.substr(10,SHORT_SIZE);
        msgIdStr=data_bufferMulticast.substr(12,SHORT_SIZE); projIdStr=data_bufferMulticast.substr(14,SHORT_SIZE);
        totLenStr=data_bufferMulticast.substr(16,INT_SIZE);
        rcvdMulticastMsgId=bytesTo<unsigned short>(msgIdStr); rcvdProjId=bytesTo<unsigned short>(projIdStr);
        sourceId = bytesTo<unsigned short>(sourceIdStr);
        rcvdMulticastTotalLength=bytesTo<unsigned int>(totLenStr); destinationId=bytesTo<unsigned short>(destinationIdStr);
        //ROS_INFO("MsgId:%d", rcvdMsgId);
        if(verifyHeaderChecksum(data_bufferMulticast) && rcvdProjId==headerMsg.projectId && destinationId==0) {
          stateMulticast=5;
        } else { data_bufferMulticast.clear(); stateMulticast=0; ROS_INFO("Multicast Header Checksum Error");}
      }
      break;
      case 5:
        if(data_bufferMulticast.length()<rcvdMulticastTotalLength) {
          data_bufferMulticast+=buffer[buffer_index]; bytes_read--; buffer_index++;
        }
        if(data_bufferMulticast.length()==rcvdMulticastTotalLength) {
          if(verifyMessageChecksum(data_bufferMulticast)) {
            if(rcvdMulticastMsgId==headerMsg.locationTelemetryMsgId) { extractLocalizationData(data_bufferMulticast);}
          } else ROS_INFO("Multicast Data Checksum Error");
          data_bufferMulticast.clear(); stateMulticast=0;
        }
        break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Server");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  pTCPWrite = n.advertise<std_msgs::String>("tcp_socket/write", 1);
  pMulticastWrite = n.advertise<std_msgs::String>("multicast_socket/write", 1);

  ros::Subscriber sub1 = n.subscribe("tcp_socket/read", 1, &readTCPData);
  ros::Subscriber sub2 = n.subscribe("multicast_socket/read", 1, &readMulticastData);

  taskThread = boost::shared_ptr<boost::thread>(new boost::thread(sendRouteData));

  ros::spin();
  return 0;
}
