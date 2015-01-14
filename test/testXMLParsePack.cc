
#include <xml_parser/XMLParsePack.hh>
#include <gtest/gtest.h>

using namespace std;

class ParseandPackTest : public testing::Test {
public:
  struct XML {
    string str;
    vector<XMLNode> nodes;
  };
  XML XML1;
  XML XML2;
  XMLParsePack xmlpp;


  void setup1() {
    XML xml;
    xml.str="<RobotTelemetry><AGV idRobot=\"1234\" xPos =\"1.22\" yPos =\"2.33\" thetaR=\"12.2\" Batt=\"50.2\" status=\"1\"/></RobotTelemetry>";

    XMLAttribute attr;
    XMLElement element;
    XMLNode node;
    attr.name="idRobot"; attr.value="1234"; element.attributes.push_back(attr);
    attr.name="xPos"; attr.value="1.22"; element.attributes.push_back(attr);
    attr.name="yPos"; attr.value="2.33"; element.attributes.push_back(attr);
    attr.name="thetaR"; attr.value="12.2"; element.attributes.push_back(attr);
    attr.name="Batt"; attr.value="50.2"; element.attributes.push_back(attr); 
    attr.name="status"; attr.value="1"; element.attributes.push_back(attr);
    element.name="AGV";
    node.elements.push_back(element);
    node.name="RobotTelemetry";
    xml.nodes.push_back(node);
    XML1=xml;
  }
  
  void setup2() {
    XML xml;
    xml.str="<RobotTelemetry><AGV idRobot=\"1234\" xPos =\"1.22\" yPos =\"2.33\" thetaR=\"12.2\" Batt=\"50.2\" status=\"1\"/>"
        "<Health idRobot=\"5548\" motorStatus=\"Disabled\" LaserStatus=\"NoResponse\"/>"
        "</RobotTelemetry>";

    XMLAttribute attr;
    XMLElement element;
    XMLNode node;
    attr.name="idRobot"; attr.value="1234"; element.attributes.push_back(attr);
    attr.name="xPos"; attr.value="1.22"; element.attributes.push_back(attr);
    attr.name="yPos"; attr.value="2.33"; element.attributes.push_back(attr);
    attr.name="thetaR"; attr.value="12.2"; element.attributes.push_back(attr);
    attr.name="Batt"; attr.value="50.2"; element.attributes.push_back(attr); 
    attr.name="status"; attr.value="1"; element.attributes.push_back(attr);
    element.name="AGV";
    node.elements.push_back(element);
    element.attributes.clear();
    attr.name="idRobot"; attr.value="5548"; element.attributes.push_back(attr);
    attr.name="motorStatus"; attr.value="Disabled"; element.attributes.push_back(attr);
    attr.name="LaserStatus"; attr.value="NoResponse"; element.attributes.push_back(attr);
    element.name="Health";
    node.elements.push_back(element);
    node.name="RobotTelemetry";
    xml.nodes.push_back(node);
    XML2=xml;
  }
  
  virtual void SetUp(){
    setup1();
    setup2();
  }
  virtual void TearDown(){}
};

TEST_F(ParseandPackTest, testParseSingleElement) {
  XML xml=XML1;
  vector<XMLNode> nodes=xmlpp.parse(xml.str);

  for(unsigned int i=0; i<nodes.size(); i++) {
    EXPECT_EQ(nodes[i].name,xml.nodes[i].name);
    int j=0;
    EXPECT_EQ(nodes[i].elements[j].name,xml.nodes[i].elements[j].name);
    for(unsigned int k=0; k<nodes[i].elements[j].attributes.size(); k++) {
      EXPECT_EQ(nodes[i].elements[j].attributes[k].name,xml.nodes[i].elements[j].attributes[k].name);
      EXPECT_EQ(nodes[i].elements[j].attributes[k].value,xml.nodes[i].elements[j].attributes[k].value);
    }
  }
}

TEST_F(ParseandPackTest, testParseMultipleElements) {
  XML xml=XML2;
  vector<XMLNode> nodes=xmlpp.parse(xml.str);

  for(unsigned int i=0; i<nodes.size(); i++) {
    EXPECT_EQ(nodes[i].name,xml.nodes[i].name);
    for(unsigned int j=0; j<nodes[i].elements.size(); j++) {
      EXPECT_EQ(nodes[i].elements[j].name,xml.nodes[i].elements[j].name);
      for(unsigned int k=0; k<nodes[i].elements[j].attributes.size(); k++) {
        EXPECT_EQ(nodes[i].elements[j].attributes[k].name,xml.nodes[i].elements[j].attributes[k].name);
        EXPECT_EQ(nodes[i].elements[j].attributes[k].value,xml.nodes[i].elements[j].attributes[k].value);
      }
    }
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}






