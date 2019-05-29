#ifndef _ROS_quori_driver_BasePos_h
#define _ROS_quori_driver_BasePos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace quori_driver
{

  class BasePos : public ros::Msg
  {
    public:
      typedef int16_t _M1_type;
      _M1_type M1;
      typedef int16_t _M2_type;
      _M2_type M2;
      typedef int16_t _MT_type;
      _MT_type MT;

    BasePos():
      M1(0),
      M2(0),
      MT(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_M1;
      u_M1.real = this->M1;
      *(outbuffer + offset + 0) = (u_M1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_M1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->M1);
      union {
        int16_t real;
        uint16_t base;
      } u_M2;
      u_M2.real = this->M2;
      *(outbuffer + offset + 0) = (u_M2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_M2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->M2);
      union {
        int16_t real;
        uint16_t base;
      } u_MT;
      u_MT.real = this->MT;
      *(outbuffer + offset + 0) = (u_MT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MT.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->MT);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_M1;
      u_M1.base = 0;
      u_M1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_M1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->M1 = u_M1.real;
      offset += sizeof(this->M1);
      union {
        int16_t real;
        uint16_t base;
      } u_M2;
      u_M2.base = 0;
      u_M2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_M2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->M2 = u_M2.real;
      offset += sizeof(this->M2);
      union {
        int16_t real;
        uint16_t base;
      } u_MT;
      u_MT.base = 0;
      u_MT.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MT.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->MT = u_MT.real;
      offset += sizeof(this->MT);
     return offset;
    }

    const char * getType(){ return "quori_driver/BasePos"; };
    const char * getMD5(){ return "ee621ad1564f8b5f6676fb27cd113662"; };

  };

}
#endif