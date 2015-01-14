
#ifndef _AGV_DATA_HH
#define _AGV_DATA_HH

#include <stdio.h>
#include <tinyxml.h>

using namespace std;

class AGV {
private:
  int id;
  double xPos,yPos,yaw,batt;
  int status;


public:
  ///Construct and Destroy!
  /**
   */
  AGV();
  /**
   * \brief   Destroys this instance.
   */
  ~AGV(){ }
};

#endif
