#ifndef _ROS_rpsexamples_Mon_h
#define _ROS_rpsexamples_Mon_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rpsexamples
{

  class Mon : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _argument_type;
      _argument_type argument;

    Mon():
      command(""),
      argument("")
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
      uint32_t length_argument = strlen(this->argument);
      varToArr(outbuffer + offset, length_argument);
      offset += 4;
      memcpy(outbuffer + offset, this->argument, length_argument);
      offset += length_argument;
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
      uint32_t length_argument;
      arrToVar(length_argument, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_argument; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_argument-1]=0;
      this->argument = (char *)(inbuffer + offset-1);
      offset += length_argument;
     return offset;
    }

    virtual const char * getType() override { return "rpsexamples/Mon"; };
    virtual const char * getMD5() override { return "89ea12cd1f3864199e2c936705472021"; };

  };

}
#endif
