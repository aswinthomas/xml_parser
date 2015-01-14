
#ifndef _XML_PARSE_PACK_HH
#define _XML_PARSE_PACK_HH

#include <stdio.h>
#include <tinyxml.h>
#include <vector>

using namespace std;

struct XMLAttribute {
  std::string name;
  std::string value;
};

struct XMLElement {
  std::string name;
  vector<XMLAttribute> attributes;
};

struct XMLNode {
  std::string name;
  vector<XMLElement> elements;
};

/*
 * <node>
 *   <element attribute1name="attribute1value" attribute2name="attribute2value"/>
 * </node>
 */

class XMLParsePack {
private:


public:

  static vector<XMLNode> parse(std::string buffer);
  static std::string pack(XMLNode node);

 static void parseAndPackTest();
  ///Construct and Destroy!
  /**
   */
  XMLParsePack();
  /**
   * \brief   Destroys this instance.
   */
  ~XMLParsePack(){ }
};

#endif
