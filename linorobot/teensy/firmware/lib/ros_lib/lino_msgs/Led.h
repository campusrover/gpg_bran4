#ifndef _ROS_lino_msgs_Led_h
#define _ROS_lino_msgs_Led_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lino_msgs
{

  class Led : public ros::Msg
  {
    public:
      typedef int8_t _wire_type;
      _wire_type wire;
      typedef bool _on_type;
      _on_type on;
      typedef bool _blink_type;
      _blink_type blink;

    Led():
      wire(0),
      on(0),
      blink(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_wire;
      u_wire.real = this->wire;
      *(outbuffer + offset + 0) = (u_wire.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wire);
      union {
        bool real;
        uint8_t base;
      } u_on;
      u_on.real = this->on;
      *(outbuffer + offset + 0) = (u_on.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on);
      union {
        bool real;
        uint8_t base;
      } u_blink;
      u_blink.real = this->blink;
      *(outbuffer + offset + 0) = (u_blink.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blink);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_wire;
      u_wire.base = 0;
      u_wire.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wire = u_wire.real;
      offset += sizeof(this->wire);
      union {
        bool real;
        uint8_t base;
      } u_on;
      u_on.base = 0;
      u_on.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->on = u_on.real;
      offset += sizeof(this->on);
      union {
        bool real;
        uint8_t base;
      } u_blink;
      u_blink.base = 0;
      u_blink.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blink = u_blink.real;
      offset += sizeof(this->blink);
     return offset;
    }

    virtual const char * getType() override { return "lino_msgs/Led"; };
    virtual const char * getMD5() override { return "37838ae7aa70723939792ea809ae469c"; };

  };

}
#endif
