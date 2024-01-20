#ifndef _ROS_lino_msgs_Inst_h
#define _ROS_lino_msgs_Inst_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lino_msgs
{

  class Inst : public ros::Msg
  {
    public:
      typedef float _l_encoder_type;
      _l_encoder_type l_encoder;
      typedef float _r_encoder_type;
      _r_encoder_type r_encoder;
      typedef float _l_piderror_type;
      _l_piderror_type l_piderror;
      typedef float _r_piderror_type;
      _r_piderror_type r_piderror;
      typedef float _l_rpm_type;
      _l_rpm_type l_rpm;
      typedef float _r_rpm_type;
      _r_rpm_type r_rpm;

    Inst():
      l_encoder(0),
      r_encoder(0),
      l_piderror(0),
      r_piderror(0),
      l_rpm(0),
      r_rpm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_l_encoder;
      u_l_encoder.real = this->l_encoder;
      *(outbuffer + offset + 0) = (u_l_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_encoder);
      union {
        float real;
        uint32_t base;
      } u_r_encoder;
      u_r_encoder.real = this->r_encoder;
      *(outbuffer + offset + 0) = (u_r_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_encoder);
      union {
        float real;
        uint32_t base;
      } u_l_piderror;
      u_l_piderror.real = this->l_piderror;
      *(outbuffer + offset + 0) = (u_l_piderror.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_piderror.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_piderror.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_piderror.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_piderror);
      union {
        float real;
        uint32_t base;
      } u_r_piderror;
      u_r_piderror.real = this->r_piderror;
      *(outbuffer + offset + 0) = (u_r_piderror.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_piderror.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_piderror.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_piderror.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_piderror);
      union {
        float real;
        uint32_t base;
      } u_l_rpm;
      u_l_rpm.real = this->l_rpm;
      *(outbuffer + offset + 0) = (u_l_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_rpm);
      union {
        float real;
        uint32_t base;
      } u_r_rpm;
      u_r_rpm.real = this->r_rpm;
      *(outbuffer + offset + 0) = (u_r_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_rpm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_l_encoder;
      u_l_encoder.base = 0;
      u_l_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_encoder = u_l_encoder.real;
      offset += sizeof(this->l_encoder);
      union {
        float real;
        uint32_t base;
      } u_r_encoder;
      u_r_encoder.base = 0;
      u_r_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_encoder = u_r_encoder.real;
      offset += sizeof(this->r_encoder);
      union {
        float real;
        uint32_t base;
      } u_l_piderror;
      u_l_piderror.base = 0;
      u_l_piderror.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_piderror.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_piderror.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_piderror.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_piderror = u_l_piderror.real;
      offset += sizeof(this->l_piderror);
      union {
        float real;
        uint32_t base;
      } u_r_piderror;
      u_r_piderror.base = 0;
      u_r_piderror.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_piderror.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_piderror.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_piderror.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_piderror = u_r_piderror.real;
      offset += sizeof(this->r_piderror);
      union {
        float real;
        uint32_t base;
      } u_l_rpm;
      u_l_rpm.base = 0;
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_rpm = u_l_rpm.real;
      offset += sizeof(this->l_rpm);
      union {
        float real;
        uint32_t base;
      } u_r_rpm;
      u_r_rpm.base = 0;
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_rpm = u_r_rpm.real;
      offset += sizeof(this->r_rpm);
     return offset;
    }

    virtual const char * getType() override { return "lino_msgs/Inst"; };
    virtual const char * getMD5() override { return "f1994f0f04917e939ed782420b6a1041"; };

  };

}
#endif
