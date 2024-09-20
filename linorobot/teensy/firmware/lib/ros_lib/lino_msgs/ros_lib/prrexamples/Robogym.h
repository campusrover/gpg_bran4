#ifndef _ROS_prrexamples_Robogym_h
#define _ROS_prrexamples_Robogym_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace prrexamples
{

  class Robogym : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef float _lin_type;
      _lin_type lin;
      typedef float _ang_type;
      _ang_type ang;
      typedef float _rate_type;
      _rate_type rate;
      typedef float _lim_type;
      _lim_type lim;

    Robogym():
      command(""),
      lin(0),
      ang(0),
      rate(0),
      lim(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      union {
        float real;
        uint32_t base;
      } u_lin;
      u_lin.real = this->lin;
      *(outbuffer + offset + 0) = (u_lin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lin);
      union {
        float real;
        uint32_t base;
      } u_ang;
      u_ang.real = this->ang;
      *(outbuffer + offset + 0) = (u_ang.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ang.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ang.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ang.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ang);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rate);
      union {
        float real;
        uint32_t base;
      } u_lim;
      u_lim.real = this->lim;
      *(outbuffer + offset + 0) = (u_lim.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lim.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lim.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lim.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lim);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      union {
        float real;
        uint32_t base;
      } u_lin;
      u_lin.base = 0;
      u_lin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lin = u_lin.real;
      offset += sizeof(this->lin);
      union {
        float real;
        uint32_t base;
      } u_ang;
      u_ang.base = 0;
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ang = u_ang.real;
      offset += sizeof(this->ang);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
      union {
        float real;
        uint32_t base;
      } u_lim;
      u_lim.base = 0;
      u_lim.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lim.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lim.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lim.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lim = u_lim.real;
      offset += sizeof(this->lim);
     return offset;
    }

    virtual const char * getType() override { return "prrexamples/Robogym"; };
    virtual const char * getMD5() override { return "3d4c9b79fbcfdc523f83bc3d7e9c3136"; };

  };

}
#endif
