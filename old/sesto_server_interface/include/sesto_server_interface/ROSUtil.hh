#ifndef _ROSUTIL_H_
#define _ROSUTIL_H_

#include <math.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <deque>
#include <vector>
#include <utility>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace ROSUtil
{
static const short CHAR_SIZE = sizeof(char); // Int8 or UInt8 (also for bool)
static const short SHORT_SIZE = sizeof(short);  // Int16 or UInt16
static const short INT_SIZE = sizeof(int); // Int32 or UInt32
static const short FLOAT_SIZE = sizeof(float); // Float32
static const short LONG_INT_SIZE = sizeof(long long int); // Int64 or UInt64
static const short DOUBLE_SIZE = sizeof(double); // Double64

char spare = 0xFF;

struct HeaderMsg
{
  unsigned short projectId;
  unsigned int checkSumLength, headerLength;
  char syncByte1, syncByte2, syncByte3, spare;
  unsigned char majorVersion, minorVersion, bugFixVersion;
  unsigned short routeMsgId, locationTelemetryMsgId, ackMsgId;
};

HeaderMsg headerMsg = {3, 
                       4, 28, 
                       0xA1, 0xB2, 0xC3, 0xFF, 
                       0x01, 0x00, 0x00,
                       3300, 3400, 255
                       
};

template<class S>
  string toBytes(S a)
  {
    char* b = reinterpret_cast<char*>(&a);
    string str;
    for (unsigned int i = 0; i < sizeof(S); i++)
      str += (b[i] & 0xFF);
    return str;
  }

template<class D>
  D bytesTo(string a)
  {
    char* b = new char(sizeof(D));
    for (unsigned int i = 0; i < sizeof(D); i++)
      b[i] = (a[i] & 0xFF);
    D d = *(reinterpret_cast<D*>(b));
    delete b;
    return d;
  }

template<class D>
  D bytesToReversed(string a)
  {
    std::reverse(a.begin(), a.end());
    char* b = new char(sizeof(D));
    for (unsigned int i = 0; i < sizeof(D); i++)
      b[i] = (a[i] & 0xFF);
    D d = *(reinterpret_cast<D*>(b));
    delete b;
    return d;
  }

template<class S, class D>
  D toType(S a)
  {
    return boost::lexical_cast<D>(a);
  }

unsigned int computeChecksum(unsigned char* buffer, int byteCount)
{
  unsigned int sum1 = 0xFFFF;
  unsigned int sum2 = 0xFFFF;
  unsigned int tlen = 0;
  unsigned int shortCount = byteCount / sizeof(short);
  unsigned int oddLength = byteCount % 2;
  //unsigned int index=0;
  while (shortCount)
  {
    tlen = shortCount > 360 ? 360 : shortCount;
    shortCount -= tlen;
    do
    {
      sum1 += *buffer++;
      sum1 += (*buffer++ << 8);
      sum2 += sum1;
    } while (--tlen);
    if ((oddLength == 1) && (shortCount < 1))
    {
      sum1 += *buffer++;
      sum2 += sum1;
    }
    sum1 = (sum1 & 0xFFFF) + (sum1 >> 16);
    sum2 = (sum2 & 0xFFFF) + (sum2 >> 16);
  }
  sum1 = (sum1 & 0xFFFF) + (sum1 >> 16);
  sum2 = (sum2 & 0xFFFF) + (sum2 >> 16);
  return (sum2 << 16 | sum1);
}

int verifyHeaderChecksum(string buffer)
{
  int checksumMatch = 0;
  unsigned char headerString[headerMsg.headerLength + 1];
  for (unsigned int i = 0; i < headerMsg.headerLength; i++)
    headerString[i] = buffer[i];
  unsigned int calcChecksum = computeChecksum(headerString, headerMsg.headerLength - 8);
  unsigned int rcvdChecksum = bytesTo<unsigned int>(
      buffer.substr(headerMsg.headerLength - (headerMsg.checkSumLength * 2), INT_SIZE));
  if (calcChecksum != rcvdChecksum)
  {
    checksumMatch = 0;
    string headerChkSm = toBytes<unsigned int>(calcChecksum);
    /*printf("Header Checksum Received: ");
    for (unsigned int i = headerMsg.headerLength - 8; i < headerMsg.headerLength - 4; i++)
      printf("%x ", buffer[i]);
    printf("---- Checksum Calculated: ");
    for (unsigned int i = 0; i < headerChkSm.size(); i++)
      printf("%x ", headerChkSm[i]);
    printf("\n");*/
    //printf("Header Checksum No match: %d %d\n",calcChecksum,rcvdChecksum);
  }
  else
    checksumMatch = 1;
  return checksumMatch;
}

int verifyMessageChecksum(string buffer)
{
  int checksumMatch = 0;
  unsigned int rcvdTotalLength = buffer.length();
  unsigned char messageString[rcvdTotalLength - headerMsg.headerLength + 1];
  for (unsigned int i = headerMsg.headerLength, j = 0; i < rcvdTotalLength; i++, j++)
    messageString[j] = buffer[i];
  unsigned int calcChecksum = computeChecksum(messageString, abs(rcvdTotalLength - headerMsg.headerLength));
  unsigned int rcvdChecksum = bytesTo<unsigned int>(buffer.substr(headerMsg.headerLength - 4, INT_SIZE));
  if (calcChecksum != rcvdChecksum)
  {
    checksumMatch = 0;
    string msgChkSm = toBytes<unsigned int>(calcChecksum);
    /*printf("Data Checksum Received: ");
    for (unsigned int i = headerMsg.headerLength - 4; i < headerMsg.headerLength; i++)
      printf("%x ", buffer[i]);
    printf("---- Checksum Calculated: ");
    for (unsigned int i = 0; i < msgChkSm.size(); i++)
      printf("%x ", msgChkSm[i]);
    printf("\n");*/
    //printf("Message Checksum No match: %d %d totLen:%d Msglen:%d\n",calcChecksum,rcvdChecksum,rcvdTotalLength,rcvdTotalLength-headerMsg.headerLength);
  }
  else
    checksumMatch = 1;
  return checksumMatch;
}

//destId==0 if there is no specific destination
std_msgs::String packMsgToSend(unsigned short sourceId, unsigned short destId, string data, unsigned short txtMsgId)
{
  unsigned char empty = 0x00;
  unsigned int msgLen = data.length();
  unsigned int txtTotalLength = msgLen + headerMsg.headerLength;
  std::stringstream ss;
  ss << headerMsg.syncByte1 << headerMsg.syncByte2 << headerMsg.syncByte3 << headerMsg.spare;
  ss << headerMsg.majorVersion << headerMsg.minorVersion << headerMsg.bugFixVersion;
  ss << headerMsg.spare;
  ss << toBytes<unsigned short>(sourceId) << toBytes<unsigned short>(destId);
  ss << toBytes<unsigned short>(txtMsgId) << toBytes<unsigned short>(headerMsg.projectId);
  ss << toBytes<unsigned int>(txtTotalLength);
  for (unsigned int i = 0; i < msgLen; i++)
    ss << data[i];

  std_msgs::String txtString;
  txtString.data = ss.str();
  //update checksum values
  unsigned char header[headerMsg.headerLength - (headerMsg.checkSumLength * 2)];
  unsigned char message[msgLen];
  string headerChkSm, messageChkSm;
  for (unsigned int i = 0; i < headerMsg.headerLength - (headerMsg.checkSumLength * 2); i++)
    header[i] = txtString.data[i];
  for (unsigned int i = headerMsg.headerLength - (headerMsg.checkSumLength * 2), j = 0;
      i < headerMsg.headerLength - (headerMsg.checkSumLength * 2) + msgLen; i++, j++)
    message[j] = txtString.data[i];
  unsigned int headerChecksum = computeChecksum(header, headerMsg.headerLength - (headerMsg.checkSumLength * 2));
  unsigned int messageChecksum = computeChecksum(message, msgLen);
  headerChkSm = toBytes<unsigned int>(headerChecksum);
  messageChkSm = toBytes<unsigned int>(messageChecksum);
  txtString.data.insert(20, 1, headerChkSm[0]);
  txtString.data.insert(21, 1, headerChkSm[1]);
  txtString.data.insert(22, 1, headerChkSm[2]);
  txtString.data.insert(23, 1, headerChkSm[3]);
  txtString.data.insert(24, 1, messageChkSm[0]);
  txtString.data.insert(25, 1, messageChkSm[1]);
  txtString.data.insert(26, 1, messageChkSm[2]);
  txtString.data.insert(27, 1, messageChkSm[3]);
  return txtString;
}

std_msgs::String packACKMsgToSend(unsigned short sourceId,unsigned short destId,unsigned short ackForMsgId) {
  string data;
  data=toBytes<unsigned short>(ackForMsgId);
  return packMsgToSend(sourceId,destId,data,headerMsg.ackMsgId);
}
}

#endif
