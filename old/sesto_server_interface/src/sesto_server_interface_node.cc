
#include <sesto_server_interface/sesto_server_interface_node.hh>
#include <sesto_server_interface/ROSUtil.hh>


#define PRINT_FREQ 5

using namespace std;
using namespace ROSUtil;

string nodeName("[SestoServerInterfaceNode]");

SestoServerInterfaceNode::SestoServerInterfaceNode() {
  dataTimestamp=ros::Time::now().toSec();
  rcvdTotalLength=rcvdMsgId=0;
  robotId=1; // Hard coded for temporary Wifi testing at Micron
  serverId=0;
  lastLocationTelemetryPublishTime=ros::Time::now().toSec();
  state=0;
  currPos.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);
  multicastSendCount=TCPSendCount=TCPReceiveCount=0;
}

void SestoServerInterfaceNode::initialize() {

}


int SestoServerInterfaceNode::task() {
 return 1;
}


void SestoServerInterfaceNode::extractRouteMsg(string buffer) {
  string numberOfWaypointsStr,waypointPosXStr,waypointPosYStr,waypointYawStr;
  double waypointPosX,waypointPosY,waypointYaw;
  unsigned short numberOfWaypoints;
  
  const int SIZE_OF_CONSTANT_PART_IN_BYTES=1*SHORT_SIZE;
  const int SIZE_OF_REPEATABLE_PART_IN_BYTES=2*DOUBLE_SIZE;
  
  numberOfWaypointsStr=buffer.substr(headerMsg.headerLength,SHORT_SIZE);
  numberOfWaypoints= bytesTo<unsigned short>(numberOfWaypointsStr);

  ROS_INFO("Received number of waypoints : %d",numberOfWaypoints);

  for(unsigned short k=0; k<numberOfWaypoints; k++) {
    waypointPosXStr = buffer.substr(headerMsg.headerLength+SIZE_OF_CONSTANT_PART_IN_BYTES+(SIZE_OF_REPEATABLE_PART_IN_BYTES*k),DOUBLE_SIZE);
    waypointPosX = bytesTo<double>(waypointPosXStr);
    waypointPosYStr = buffer.substr(headerMsg.headerLength+SIZE_OF_CONSTANT_PART_IN_BYTES+(SIZE_OF_REPEATABLE_PART_IN_BYTES*k)+DOUBLE_SIZE,DOUBLE_SIZE);
    waypointPosY = bytesTo<double>(waypointPosYStr);
    waypointYawStr = buffer.substr(headerMsg.headerLength+SIZE_OF_CONSTANT_PART_IN_BYTES+(SIZE_OF_REPEATABLE_PART_IN_BYTES*k)+DOUBLE_SIZE,DOUBLE_SIZE);
    waypointYaw = bytesTo<double>(waypointYawStr);

    ROS_INFO("Received waypoint (%.2f,%.2f,%.2f)",waypointPosX,waypointPosY,waypointYaw);
  }

}

void SestoServerInterfaceNode::setCurrPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  currPos.pose=msg->pose.pose;
}


void SestoServerInterfaceNode::fromServerCallback(const std_msgs::String::ConstPtr& msg) {
  int buffer_index=0, bytes_read = msg->data.length();
  string buffer=msg->data;
//ROS_INFO("bytes_read : %d",bytes_read);
  while(bytes_read > 0){
    switch(state){
      case 0:
        if(buffer[buffer_index]==headerMsg.syncByte1){ state=1; data_buffer+=buffer[buffer_index];}
        else { data_buffer.clear(); state=0;}
        bytes_read--; buffer_index++;
        break;
      case 1:
        if(buffer[buffer_index]==headerMsg.syncByte2){ state=2; data_buffer+=buffer[buffer_index];}
        else { data_buffer.clear(); state=0;}
        bytes_read--; buffer_index++;
        break;
      case 2:
        if(buffer[buffer_index]==headerMsg.syncByte3){ state=3; data_buffer+=buffer[buffer_index];}
        else { data_buffer.clear(); state=0;}
        bytes_read--; buffer_index++;
        break;
      case 3:
        if(data_buffer.length()<headerMsg.headerLength){ data_buffer+=buffer[buffer_index];}
        if(data_buffer.length()==headerMsg.headerLength) state = 4;
        bytes_read--; buffer_index++;
        break;
      case 4:{
        string projIdStr,msgIdStr,totLenStr,sourceIdStr,destinationIdStr;
        unsigned short rcvdProjId=0,destinationId=0,sourceId=0;
        sourceIdStr=data_buffer.substr(8,SHORT_SIZE); destinationIdStr=data_buffer.substr(10,SHORT_SIZE);
        msgIdStr=data_buffer.substr(12,SHORT_SIZE); projIdStr=data_buffer.substr(14,SHORT_SIZE);
        totLenStr=data_buffer.substr(16,INT_SIZE);
        rcvdMsgId=bytesTo<unsigned short>(msgIdStr); rcvdProjId=bytesTo<unsigned short>(projIdStr);
        sourceId = bytesTo<unsigned short>(sourceIdStr);
        rcvdTotalLength=bytesTo<unsigned int>(totLenStr); destinationId=bytesTo<unsigned short>(destinationIdStr);
        //ROS_INFO("MsgId:%d", rcvdMsgId);
        if(verifyHeaderChecksum(data_buffer) && rcvdProjId==headerMsg.projectId && sourceId==0 && destinationId==robotId) {
          state=5;
        } else { data_buffer.clear(); state=0; ROS_INFO("%20s Header Checksum Error",nodeName.c_str());}
      }
      break;
      case 5:
        if(data_buffer.length()<rcvdTotalLength) {
          data_buffer+=buffer[buffer_index]; bytes_read--; buffer_index++;
        }
        if(data_buffer.length()==rcvdTotalLength){
          if(verifyMessageChecksum(data_buffer)) {
            if(rcvdMsgId==headerMsg.routeMsgId) {
              TCPReceiveCount++;
              ROS_INFO("%d messages received by AGV through TCP",TCPReceiveCount);
              extractRouteMsg(data_buffer);
              pTCPWrite.publish(packACKMsgToSend(robotId,serverId,headerMsg.routeMsgId));
              TCPSendCount++;
              ROS_INFO("%d messages sent by AGV through TCP",TCPSendCount);
            }
          } else ROS_INFO("%20s Data Checksum Error",nodeName.c_str());
          data_buffer.clear(); state=0;
        }
        break;
    }
  }
}


void SestoServerInterfaceNode::publishMsgs() {

  //publish telemetry for server
  if(fabs(lastLocationTelemetryPublishTime-ros::Time::now().toSec())>LOCATION_TELEMETRY_PUBLISH_DELAY) {
    lastLocationTelemetryPublishTime=ros::Time::now().toSec();
    string buffer;
    buffer=toBytes<double>(currPos.pose.position.x);
    buffer+=toBytes<double>(currPos.pose.position.y);
    buffer+=toBytes<double>(tf::getYaw(currPos.pose.orientation));
    pUDPWrite.publish(packMsgToSend(robotId,serverId,buffer,headerMsg.locationTelemetryMsgId));
    multicastSendCount++;
    ROS_INFO("%d messages sent by AGV through multicast",multicastSendCount);
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "sesto_server_interface_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  SestoServerInterfaceNode ssin;
  double freq;
  if(!(np.hasParam("freq"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'freq' parameter not set. Defaulting to 5Hz",nodeName.c_str());
  np.param<double>("freq",freq,5);
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s freq:%.2f",nodeName.c_str(),freq);
  ros::Rate loop_rate(freq);

  //subscribers
  ros::Subscriber sub1 = n.subscribe("tcp_socket/read", 1, &SestoServerInterfaceNode::fromServerCallback, &ssin);
  ros::Subscriber sub2 = n.subscribe("/amcl_pose", 1, &SestoServerInterfaceNode::setCurrPoseCallback, &ssin);

  //publishers
  ssin.pUDPWrite = n.advertise<std_msgs::String>("multicast_socket/write", 1);
  ssin.pTCPWrite = n.advertise<std_msgs::String>("tcp_socket/write", 1);

  ssin.initialize();
  int sleepCount=0;
  while (ros::ok()) {
    ssin.task();
    ssin.publishMsgs();
    ros::spinOnce();   //for subscriber callbacks to be called
    loop_rate.sleep();
  }
  ros::spin();

  
  /*
  
  
  // Set the global C and C++ locale to the user-configured locale,
  // so we can use std::cout with UTF-8, via Glib::ustring, without exceptions.
  std::locale::global(std::locale(""));

  std::string filepath;
  if(argc > 1 )
    filepath = argv[1]; //Allow the user to specify a different XML file to parse.
  else
    filepath = "example.xml";
    
  // Parse the entire document in one go:
  int return_code = EXIT_SUCCESS;
  try
  {
    MySaxParser parser;
    parser.set_substitute_entities(true);
    parser.parse_file(filepath);
  }
  catch(const xmlpp::exception& ex)
  {
    std::cerr << "libxml++ exception: " << ex.what() << std::endl;
    return_code = EXIT_FAILURE;
  }

  // Incremental parsing, sometimes useful for network connections:
  try
  {
    std::cout << std::endl << "Incremental SAX Parser:" << std::endl;
    
    std::ifstream is(filepath.c_str());
    if (!is)
      throw xmlpp::exception("Could not open file " + filepath);

    char buffer[64];
    const size_t buffer_size = sizeof(buffer) / sizeof(char);

    //Parse the file:
    MySaxParser parser;
    parser.set_substitute_entities(true);
    do
    {
      std::memset(buffer, 0, buffer_size);
      is.read(buffer, buffer_size-1);
      if(is.gcount())
      {
        // We use Glib::ustring::ustring(InputIterator begin, InputIterator end)
        // instead of Glib::ustring::ustring( const char*, size_type ) because it
        // expects the length of the string in characters, not in bytes.
        Glib::ustring input(buffer, buffer+is.gcount());
        parser.parse_chunk(input);
      }
    }
    while(is);

    parser.finish_chunk_parsing();
  }
  catch(const xmlpp::exception& ex)
  {
    std::cerr << "Incremental parsing, libxml++ exception: " << ex.what() << std::endl;
    return_code = EXIT_FAILURE;
  }

  return return_code;
}
*/
  return 0;
}
