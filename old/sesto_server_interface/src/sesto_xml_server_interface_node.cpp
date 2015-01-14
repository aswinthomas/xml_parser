
#include <sesto_server_interface/sesto_xml_server_interface_node.hh>

#define PRINT_FREQ 0.5

using namespace std;

string nodeName("[SestoXMLServerInterfaceNode]");


SestoXMLServerInterfaceNode::SestoXMLServerInterfaceNode() {
  publish_vector.clear();
}

void SestoXMLServerInterfaceNode::initialize() {

}


void SestoXMLServerInterfaceNode::publishTestMsgs(){
//  string buffer="<RobotTelemetry><AGV idRobot=\"1234\" xPos =\"1.22\" yPos =\"2.33\" thetaR=\"12.2\" Batt=\"50.2\" status=\"1\"/></RobotTelemetry>";
  string buffer="<RobotTasklist><Node idNode = \"1\" xPos = \"3.456\" yPos = \"5.678\" thetaR=\"0.0\" isStation=0/><Test idNode = \"2\" xPos = \"3.456\" yPos = \"5.678\" thetaR=\"150.2\" isStation=1/></RobotTasklist>";
  pTestMsgs.publish(buffer);
}

void SestoXMLServerInterfaceNode::publishMsgs() {
  for(int i=0;i<publish_vector.size();i++){
    publish_vector.back()->publish();
    publish_vector.pop_back();
  }
}

void SestoXMLServerInterfaceNode::amclPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg) {
  robotTelemetryPopulator.set_container(msg);
}

void SestoXMLServerInterfaceNode::fromServerCallback(const std_msgs::String::ConstPtr& msg) {
  vector<XMLNode> nodes = XMLParsePack::parse(msg->data); // parse the XML syntax in the incoming string
  ROS_INFO("nodes.size() : %lu", nodes.size());
  for(int i=0;i<nodes.size();i++){
    if(nodes[i].name.compare("RobotTasklist")==0){
      robotTasklistPopulator.set_ros_msg(nodes[i].elements);
      publish_vector.push_back(&robotTasklistPopulator);
    }else{
      ROS_WARN("Unknown message type : %s",nodes[i].name.c_str());
    }
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "sesto_xml_server_interface_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  global_np=&n;

//  XMLParsePack::parseAndPackTest();

  SestoXMLServerInterfaceNode node;
  double freq;
  if(!(np.hasParam("freq"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'freq' parameter not set. Defaulting to 5Hz",nodeName.c_str());
  np.param<double>("freq",freq,5);
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s freq:%.2f",nodeName.c_str(),freq);
  ros::Rate loop_rate(freq);

  //server socket subscriber
  ros::Subscriber sub1 = n.subscribe("server_socket/read", 1, &SestoXMLServerInterfaceNode::fromServerCallback, &node);
  //AGV topics subscribers
  ros::Subscriber sub2 = n.subscribe("/amcl_pose", 1, &SestoXMLServerInterfaceNode::amclPoseCallBack, &node);

  //server socket publisher
//  node.pServerSocket = n.advertise<std_msgs::String>("server_socket/write", 1);
  //Test topics publishers
  node.pTestMsgs = n.advertise<std_msgs::String>("server_socket/read", 1);


  node.initialize();
  int sleepCount=0;
  while (ros::ok()) {
    node.publishTestMsgs();
    node.publishMsgs();
    ros::spinOnce();   //for subscriber callbacks to be called
    loop_rate.sleep();
  }
  ros::spin();


  return 0;
}
