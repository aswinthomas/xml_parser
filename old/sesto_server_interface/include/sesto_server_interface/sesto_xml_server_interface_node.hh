
#ifndef _SESTO_XML_SERVER_INTERFACE_NODE_HH
#define _SESTO_XML_SERVER_INTERFACE_NODE_HH

#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <swarm_msgs/RobotTelemetry.h>
#include <swarm_msgs/RobotHealth.h>
#include <swarm_msgs/RobotTasklist.h>
#include <swarm_msgs/RobotTasklistWaypoint.h>
#include <sesto_server_interface/XMLParsePack.hh>
#include <std_msgs/String.h>
#include <tf/tf.h>

using namespace std;

ros::NodeHandle* global_np;

class XMLROSMsg {
protected:
  ros::Publisher pub_;
public:
  XMLROSMsg(){}
  virtual ~XMLROSMsg(){}
  virtual void publish()=0;
};



class XMLROSMsgToRobot:public XMLROSMsg {
private:
public:
  XMLROSMsgToRobot(){}
  ~XMLROSMsgToRobot(){}
  virtual void set_ros_msg(vector<XMLElement> elements)=0;
};



class XMLROSMsgToServer:public XMLROSMsg {
protected:
  XMLNode node;
  string stringmsg;
public:
  XMLROSMsgToServer(){
    pub_ = global_np->advertise<std_msgs::String>("server_socket/write", 1);
  }
  ~XMLROSMsgToServer(){}
  virtual void set_container(const geometry_msgs::Pose::ConstPtr& msg)=0;
  virtual void set_container(const swarm_msgs::RobotHealth::ConstPtr& msg)=0;
  virtual void set_xml_node()=0;
};


class RobotTelemetryPopulator:public XMLROSMsgToServer {
private:
  swarm_msgs::RobotTelemetry container;
public:
  RobotTelemetryPopulator(){
    node.name="AGV";
  }
  ~RobotTelemetryPopulator(){ }

  void publish(){
    set_xml_node();
    stringmsg=XMLParsePack::pack(node);
    pub_.publish(stringmsg);
  }

  void set_container(const geometry_msgs::Pose::ConstPtr& msg){
    container.pose=*msg;
  }

  void set_container(const swarm_msgs::RobotHealth::ConstPtr& msg){
    container.battery_level=msg->battery_level;
  }

  void set_xml_node(){
    node.elements.clear();

    XMLElement element;
    element.name="AGV";

    XMLAttribute att;
    att.name="xPos";
    att.value=boost::lexical_cast<std::string>(container.pose.position.x);
    element.attributes.push_back(att);

    att.name="yPos";
    att.value=boost::lexical_cast<std::string>(container.pose.position.y);
    element.attributes.push_back(att);

    att.name="thetaR";
    att.value=boost::lexical_cast<std::string>(tf::getYaw(container.pose.orientation));
    element.attributes.push_back(att);

    node.elements.push_back(element);
  }

};



class RobotTasklistPopulator : public XMLROSMsgToRobot {
private:
  swarm_msgs::RobotTasklist robotTasklist;

public:
  RobotTasklistPopulator(){
    pub_ = global_np->advertise<swarm_msgs::RobotTasklist>("/robot_tasklist", 1);
  }
  ~RobotTasklistPopulator(){ }

  void publish(){
    pub_.publish(robotTasklist);
  }

  void set_ros_msg(vector<XMLElement> elements){
    robotTasklist.waypoints.clear();
    for(int i=0;i<elements.size();i++){
//      ROS_INFO("elements[%d].name : %s",i,elements[i].name.c_str());
      if(elements[i].name.compare("Node")==0){
        swarm_msgs::RobotTasklistWaypoint waypoint;
        for(int j=0;j<elements[i].attributes.size();j++){
          if(elements[i].attributes[j].name.compare("idNode")==0){
            waypoint.idNode=::atoi(elements[i].attributes[j].value.c_str());
          }else if(elements[i].attributes[j].name.compare("xPos")==0){
            waypoint.pose.position.x=::atof(elements[i].attributes[j].value.c_str());
          }else if(elements[i].attributes[j].name.compare("yPos")==0){
            waypoint.pose.position.y=::atof(elements[i].attributes[j].value.c_str());
          }else if(elements[i].attributes[j].name.compare("thetaR")==0){
            waypoint.pose.orientation=tf::createQuaternionMsgFromYaw(::atof(elements[i].attributes[j].value.c_str()));
          }else if(elements[i].attributes[j].name.compare("isStation")==0){
            waypoint.isStation=::atoi(elements[i].attributes[j].value.c_str());
          }
        }
        robotTasklist.waypoints.push_back(waypoint);
      }else{
      }
    }
  }

};



class SestoXMLServerInterfaceNode {
private:
  std::vector<XMLROSMsg*> publish_vector;
  std::vector<ros::Publisher> publisherList;

  RobotTelemetryPopulator robotTelemetryPopulator;
  RobotTasklistPopulator robotTasklistPopulator;
public:
  ///subscribe and advertise
  ros::Publisher pServerSocket;
  ros::Publisher pRobotTelemetry;
  ros::Publisher pTestMsgs;
  ///Construct and Destroy!
  /**
   */
  SestoXMLServerInterfaceNode();
  /**
   * \brief   Destroys this instance.
   */
  ~SestoXMLServerInterfaceNode(){ }

  void amclPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg);
  void fromServerCallback(const std_msgs::String::ConstPtr& msg);
  void initialize();
  void publishMsgs();
  void publishTestMsgs();
};



#endif
