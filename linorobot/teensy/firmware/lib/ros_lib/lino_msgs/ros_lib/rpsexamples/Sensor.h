#ifndef _ROS_rpsexamples_Sensor_h
#define _ROS_rpsexamples_Sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rpsexamples
{

  class Sensor : public ros::Msg
  {
    public:
      typedef float _forward_type;
      _forward_type forward;
      typedef float _left_type;
      _left_type left;
      typedef float _right_type;
      _right_type right;
      typedef float _rear_type;
      _rear_type rear;
      typedef float _shortest_type;
      _shortest_type shortest;
      typedef float _shortest_bearing_type;
      _shortest_bearing_type shortest_bearing;

    Sensor():
      forward(0),
      left(0),
      right(0),
      rear(0),
      shortest(0),
      shortest_bearing(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_forward;
      u_forward.real = this->forward;
      *(outbuffer + offset + 0) = (u_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->forward);
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right);
      union {
        float real;
        uint32_t base;
      } u_rear;
      u_rear.real = this->rear;
      *(outbuffer + offset + 0) = (u_rear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear);
      union {
        float real;
        uint32_t base;
      } u_shortest;
      u_shortest.real = this->shortest;
      *(outbuffer + offset + 0) = (u_shortest.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shortest.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shortest.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shortest.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shortest);
      union {
        float real;
        uint32_t base;
      } u_shortest_bearing;
      u_shortest_bearing.real = this->shortest_bearing;
      *(outbuffer + offset + 0) = (u_shortest_bearing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shortest_bearing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shortest_bearing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shortest_bearing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shortest_bearing);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_forward;
      u_forward.base = 0;
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->forward = u_forward.real;
      offset += sizeof(this->forward);
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right = u_right.real;
      offset += sizeof(this->right);
      union {
        float real;
        uint32_t base;
      } u_rear;
      u_rear.base = 0;
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear = u_rear.real;
      offset += sizeof(this->rear);
      union {
        float real;
        uint32_t base;
      } u_shortest;
      u_shortest.base = 0;
      u_shortest.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shortest.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shortest.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shortest.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shortest = u_shortest.real;
      offset += sizeof(this->shortest);
      union {
        float real;
        uint32_t base;
      } u_shortest_bearing;
      u_shortest_bearing.base = 0;
      u_shortest_bearing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shortest_bearing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shortest_bearing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shortest_bearing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shortest_bearing = u_shortest_bearing.real;
      offset += sizeof(this->shortest_bearing);
     return offset;
    }

    virtual const char * getType() override { return "rpsexamples/Sensor"; };
    virtual const char * getMD5() override { return "601b50e3311aa757a95341fba4a02e9b"; };

  };

}
#endif
