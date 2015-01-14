
#include <xml_parser/XMLParsePack.hh>

using namespace std;

#define DEBUG 1

XMLParsePack::XMLParsePack() {
}

vector<XMLNode> XMLParsePack::parse(string buffer) {
  vector<XMLNode> nodes;

  try {
    TiXmlDocument *root = new TiXmlDocument();
    //    printf("%s\n",buffer.c_str());
    const char *cstr = buffer.c_str();
    root->Parse((const char*)cstr, 0, TIXML_ENCODING_UTF8);

    for(TiXmlElement *child=root->FirstChildElement(); child; child = child->NextSiblingElement()) {
      XMLNode node;  
      node.name = child->ValueStr();
      if(DEBUG) printf("Node:%s\n", node.name.c_str());

      //TiXmlElement* myChild = child->FirstChildElement();
      //while(myChild) {
      for(TiXmlElement *myElement=child->FirstChildElement(); myElement; myElement = myElement->NextSiblingElement()) {
        XMLElement element; 
        element.name = myElement->ValueStr();
        if(DEBUG) printf("Element:%s\n",element.name.c_str());
        for(TiXmlAttribute *myAttribute=myElement->FirstAttribute(); myAttribute; myAttribute = myAttribute->Next()) {
          XMLAttribute attribute; 
          attribute.name = myAttribute->Name();
          attribute.value = myAttribute->ValueStr();
          if(DEBUG) printf("Attribute:%s value:%s\n",attribute.name.c_str(),attribute.value.c_str());
          element.attributes.push_back(attribute);
        }
        node.elements.push_back(element);
        //myChild = myChild->FirstChildElement();
      }
      nodes.push_back(node);
    }
    delete root;
  } catch(const std::exception& e) {
    std::cerr << "Exception caught while xml parsing: " << e.what() << std::endl;
    //nodes.clear();
  }
  return nodes;
}

string XMLParsePack::pack(XMLNode node) {
  TiXmlDocument xmlRoot;
  TiXmlElement *xmlNode = new TiXmlElement(node.name.c_str());
  for(int i=0; i<node.elements.size(); i++) {
    TiXmlElement *xmlElement = new TiXmlElement(node.elements[i].name.c_str()); 
    for(int j=0; j<node.elements[i].attributes.size(); j++) {
      xmlElement->SetAttribute(node.elements[i].attributes[j].name,node.elements[i].attributes[j].value);
    }
    xmlNode->LinkEndChild(xmlElement);
  }

  xmlRoot.LinkEndChild(xmlNode);
  string out;
  out<<xmlRoot;
  printf("%s\n",out.c_str());
  xmlNode->Clear();
  return out;
}

void XMLParsePack::parseAndPackTest() {
  string buffer="<RobotTelemetry><AGV idRobot=\"1234\" xPos =\"1.22\" yPos =\"2.33\" thetaR=\"12.2\" Batt=\"50.2\" status=\"1\"/></RobotTelemetry>";
  vector<XMLNode> nodes = parse(buffer);
  if(nodes.size()) pack(nodes[0]);
}

