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
      typedef const char* _header_type;
      _header_type header;
      typedef int32_t _array_len_type;
      _array_len_type array_len;
      uint32_t data_length;
      typedef float _data_type;
      _data_type st_data;
      _data_type * data;

    Inst():
      header(""),
      array_len(0),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_header = strlen(this->header);
      varToArr(outbuffer + offset, length_header);
      offset += 4;
      memcpy(outbuffer + offset, this->header, length_header);
      offset += length_header;
      union {
        int32_t real;
        uint32_t base;
      } u_array_len;
      u_array_len.real = this->array_len;
      *(outbuffer + offset + 0) = (u_array_len.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_array_len.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_array_len.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_array_len.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->array_len);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_header;
      arrToVar(length_header, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_header; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_header-1]=0;
      this->header = (char *)(inbuffer + offset-1);
      offset += length_header;
      union {
        int32_t real;
        uint32_t base;
      } u_array_len;
      u_array_len.base = 0;
      u_array_len.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_array_len.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_array_len.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_array_len.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->array_len = u_array_len.real;
      offset += sizeof(this->array_len);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (float*)realloc(this->data, data_lengthT * sizeof(float));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "lino_msgs/Inst"; };
    virtual const char * getMD5() override { return "894546f5acfd9dd6353c34503ca087fc"; };

  };

}
#endif
