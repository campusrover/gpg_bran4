#ifndef _ROS_lino_msgs_Diag_h
#define _ROS_lino_msgs_Diag_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lino_msgs
{

  class Diag : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _subcommand_type;
      _subcommand_type subcommand;
      typedef float _arg1_type;
      _arg1_type arg1;
      typedef float _arg2_type;
      _arg2_type arg2;
      typedef float _arg3_type;
      _arg3_type arg3;

    Diag():
      command(""),
      subcommand(""),
      arg1(0),
      arg2(0),
      arg3(0)
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
      uint32_t length_subcommand = strlen(this->subcommand);
      varToArr(outbuffer + offset, length_subcommand);
      offset += 4;
      memcpy(outbuffer + offset, this->subcommand, length_subcommand);
      offset += length_subcommand;
      union {
        float real;
        uint32_t base;
      } u_arg1;
      u_arg1.real = this->arg1;
      *(outbuffer + offset + 0) = (u_arg1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg1);
      union {
        float real;
        uint32_t base;
      } u_arg2;
      u_arg2.real = this->arg2;
      *(outbuffer + offset + 0) = (u_arg2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg2);
      union {
        float real;
        uint32_t base;
      } u_arg3;
      u_arg3.real = this->arg3;
      *(outbuffer + offset + 0) = (u_arg3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg3);
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
      uint32_t length_subcommand;
      arrToVar(length_subcommand, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_subcommand; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_subcommand-1]=0;
      this->subcommand = (char *)(inbuffer + offset-1);
      offset += length_subcommand;
      union {
        float real;
        uint32_t base;
      } u_arg1;
      u_arg1.base = 0;
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg1 = u_arg1.real;
      offset += sizeof(this->arg1);
      union {
        float real;
        uint32_t base;
      } u_arg2;
      u_arg2.base = 0;
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg2 = u_arg2.real;
      offset += sizeof(this->arg2);
      union {
        float real;
        uint32_t base;
      } u_arg3;
      u_arg3.base = 0;
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg3 = u_arg3.real;
      offset += sizeof(this->arg3);
     return offset;
    }

    virtual const char * getType() override { return "lino_msgs/Diag"; };
    virtual const char * getMD5() override { return "ca43ed677dd3e7e8e6f2d15da8ed4dd7"; };

  };

}
#endif
