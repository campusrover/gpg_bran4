#ifndef _ROS_prrexamples_Complex_h
#define _ROS_prrexamples_Complex_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace prrexamples
{

  class Complex : public ros::Msg
  {
    public:
      typedef float _real_type;
      _real_type real;
      typedef float _imaginary_type;
      _imaginary_type imaginary;

    Complex():
      real(0),
      imaginary(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_real;
      u_real.real = this->real;
      *(outbuffer + offset + 0) = (u_real.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_real.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_real.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_real.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->real);
      union {
        float real;
        uint32_t base;
      } u_imaginary;
      u_imaginary.real = this->imaginary;
      *(outbuffer + offset + 0) = (u_imaginary.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imaginary.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imaginary.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imaginary.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imaginary);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_real;
      u_real.base = 0;
      u_real.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_real.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_real.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_real.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->real = u_real.real;
      offset += sizeof(this->real);
      union {
        float real;
        uint32_t base;
      } u_imaginary;
      u_imaginary.base = 0;
      u_imaginary.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_imaginary.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_imaginary.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_imaginary.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->imaginary = u_imaginary.real;
      offset += sizeof(this->imaginary);
     return offset;
    }

    virtual const char * getType() override { return "prrexamples/Complex"; };
    virtual const char * getMD5() override { return "54da470dccf15d60bd273ab751e1c0a1"; };

  };

}
#endif
