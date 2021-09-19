#ifndef _ROS_dgz_msgs_StampedHardwareState_h
#define _ROS_dgz_msgs_StampedHardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dgz_msgs/HardwareState.h"

namespace dgz_msgs
{

  class StampedHardwareState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef dgz_msgs::HardwareState _data_type;
      _data_type data;

    StampedHardwareState():
      header(),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() { return "dgz_msgs/StampedHardwareState"; };
    virtual const char * getMD5() { return "316b2c383d1a84a278942533cf6afacb"; };

  };

}
#endif
