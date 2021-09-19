#ifndef _ROS_dagozilla_msgs_HardwareState_h
#define _ROS_dagozilla_msgs_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msgs
{

  class HardwareState : public ros::Msg
  {
    public:
      typedef int32_t _base_motor_1_pulses_type;
      _base_motor_1_pulses_type base_motor_1_pulses;
      typedef int32_t _base_motor_2_pulses_type;
      _base_motor_2_pulses_type base_motor_2_pulses;
      typedef int32_t _base_motor_3_pulses_type;
      _base_motor_3_pulses_type base_motor_3_pulses;
      typedef int32_t _base_motor_4_pulses_type;
      _base_motor_4_pulses_type base_motor_4_pulses;
      typedef int32_t _base_encoder_1_pulses_type;
      _base_encoder_1_pulses_type base_encoder_1_pulses;
      typedef int32_t _base_encoder_2_pulses_type;
      _base_encoder_2_pulses_type base_encoder_2_pulses;
      typedef int32_t _base_encoder_3_pulses_type;
      _base_encoder_3_pulses_type base_encoder_3_pulses;
      typedef int32_t _dribbler_motor_l_pulses_type;
      _dribbler_motor_l_pulses_type dribbler_motor_l_pulses;
      typedef int32_t _dribbler_motor_r_pulses_type;
      _dribbler_motor_r_pulses_type dribbler_motor_r_pulses;
      typedef float _dribbler_potentio_l_reading_type;
      _dribbler_potentio_l_reading_type dribbler_potentio_l_reading;
      typedef float _dribbler_potentio_r_reading_type;
      _dribbler_potentio_r_reading_type dribbler_potentio_r_reading;
      typedef float _compass_reading_type;
      _compass_reading_type compass_reading;
      typedef float _ir_reading_type;
      _ir_reading_type ir_reading;

    HardwareState():
      base_motor_1_pulses(0),
      base_motor_2_pulses(0),
      base_motor_3_pulses(0),
      base_motor_4_pulses(0),
      base_encoder_1_pulses(0),
      base_encoder_2_pulses(0),
      base_encoder_3_pulses(0),
      dribbler_motor_l_pulses(0),
      dribbler_motor_r_pulses(0),
      dribbler_potentio_l_reading(0),
      dribbler_potentio_r_reading(0),
      compass_reading(0),
      ir_reading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_1_pulses;
      u_base_motor_1_pulses.real = this->base_motor_1_pulses;
      *(outbuffer + offset + 0) = (u_base_motor_1_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_1_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_1_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_1_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_1_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_2_pulses;
      u_base_motor_2_pulses.real = this->base_motor_2_pulses;
      *(outbuffer + offset + 0) = (u_base_motor_2_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_2_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_2_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_2_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_2_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_3_pulses;
      u_base_motor_3_pulses.real = this->base_motor_3_pulses;
      *(outbuffer + offset + 0) = (u_base_motor_3_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_3_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_3_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_3_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_3_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_4_pulses;
      u_base_motor_4_pulses.real = this->base_motor_4_pulses;
      *(outbuffer + offset + 0) = (u_base_motor_4_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_4_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_4_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_4_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_4_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_1_pulses;
      u_base_encoder_1_pulses.real = this->base_encoder_1_pulses;
      *(outbuffer + offset + 0) = (u_base_encoder_1_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_1_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_1_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_1_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_1_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_2_pulses;
      u_base_encoder_2_pulses.real = this->base_encoder_2_pulses;
      *(outbuffer + offset + 0) = (u_base_encoder_2_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_2_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_2_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_2_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_2_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_3_pulses;
      u_base_encoder_3_pulses.real = this->base_encoder_3_pulses;
      *(outbuffer + offset + 0) = (u_base_encoder_3_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_3_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_3_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_3_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_3_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_l_pulses;
      u_dribbler_motor_l_pulses.real = this->dribbler_motor_l_pulses;
      *(outbuffer + offset + 0) = (u_dribbler_motor_l_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_l_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_l_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_l_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_l_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_r_pulses;
      u_dribbler_motor_r_pulses.real = this->dribbler_motor_r_pulses;
      *(outbuffer + offset + 0) = (u_dribbler_motor_r_pulses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_r_pulses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_r_pulses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_r_pulses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_r_pulses);
      union {
        float real;
        uint32_t base;
      } u_dribbler_potentio_l_reading;
      u_dribbler_potentio_l_reading.real = this->dribbler_potentio_l_reading;
      *(outbuffer + offset + 0) = (u_dribbler_potentio_l_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_potentio_l_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_potentio_l_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_potentio_l_reading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_potentio_l_reading);
      union {
        float real;
        uint32_t base;
      } u_dribbler_potentio_r_reading;
      u_dribbler_potentio_r_reading.real = this->dribbler_potentio_r_reading;
      *(outbuffer + offset + 0) = (u_dribbler_potentio_r_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_potentio_r_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_potentio_r_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_potentio_r_reading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_potentio_r_reading);
      union {
        float real;
        uint32_t base;
      } u_compass_reading;
      u_compass_reading.real = this->compass_reading;
      *(outbuffer + offset + 0) = (u_compass_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_compass_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_compass_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_compass_reading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->compass_reading);
      union {
        float real;
        uint32_t base;
      } u_ir_reading;
      u_ir_reading.real = this->ir_reading;
      *(outbuffer + offset + 0) = (u_ir_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir_reading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ir_reading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_1_pulses;
      u_base_motor_1_pulses.base = 0;
      u_base_motor_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_1_pulses = u_base_motor_1_pulses.real;
      offset += sizeof(this->base_motor_1_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_2_pulses;
      u_base_motor_2_pulses.base = 0;
      u_base_motor_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_2_pulses = u_base_motor_2_pulses.real;
      offset += sizeof(this->base_motor_2_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_3_pulses;
      u_base_motor_3_pulses.base = 0;
      u_base_motor_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_3_pulses = u_base_motor_3_pulses.real;
      offset += sizeof(this->base_motor_3_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_4_pulses;
      u_base_motor_4_pulses.base = 0;
      u_base_motor_4_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_4_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_4_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_4_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_4_pulses = u_base_motor_4_pulses.real;
      offset += sizeof(this->base_motor_4_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_1_pulses;
      u_base_encoder_1_pulses.base = 0;
      u_base_encoder_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_1_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_1_pulses = u_base_encoder_1_pulses.real;
      offset += sizeof(this->base_encoder_1_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_2_pulses;
      u_base_encoder_2_pulses.base = 0;
      u_base_encoder_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_2_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_2_pulses = u_base_encoder_2_pulses.real;
      offset += sizeof(this->base_encoder_2_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_3_pulses;
      u_base_encoder_3_pulses.base = 0;
      u_base_encoder_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_3_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_3_pulses = u_base_encoder_3_pulses.real;
      offset += sizeof(this->base_encoder_3_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_l_pulses;
      u_dribbler_motor_l_pulses.base = 0;
      u_dribbler_motor_l_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_l_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_l_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_l_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_l_pulses = u_dribbler_motor_l_pulses.real;
      offset += sizeof(this->dribbler_motor_l_pulses);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_r_pulses;
      u_dribbler_motor_r_pulses.base = 0;
      u_dribbler_motor_r_pulses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_r_pulses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_r_pulses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_r_pulses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_r_pulses = u_dribbler_motor_r_pulses.real;
      offset += sizeof(this->dribbler_motor_r_pulses);
      union {
        float real;
        uint32_t base;
      } u_dribbler_potentio_l_reading;
      u_dribbler_potentio_l_reading.base = 0;
      u_dribbler_potentio_l_reading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_potentio_l_reading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_potentio_l_reading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_potentio_l_reading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_potentio_l_reading = u_dribbler_potentio_l_reading.real;
      offset += sizeof(this->dribbler_potentio_l_reading);
      union {
        float real;
        uint32_t base;
      } u_dribbler_potentio_r_reading;
      u_dribbler_potentio_r_reading.base = 0;
      u_dribbler_potentio_r_reading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_potentio_r_reading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_potentio_r_reading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_potentio_r_reading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_potentio_r_reading = u_dribbler_potentio_r_reading.real;
      offset += sizeof(this->dribbler_potentio_r_reading);
      union {
        float real;
        uint32_t base;
      } u_compass_reading;
      u_compass_reading.base = 0;
      u_compass_reading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_compass_reading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_compass_reading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_compass_reading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->compass_reading = u_compass_reading.real;
      offset += sizeof(this->compass_reading);
      union {
        float real;
        uint32_t base;
      } u_ir_reading;
      u_ir_reading.base = 0;
      u_ir_reading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_reading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir_reading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir_reading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ir_reading = u_ir_reading.real;
      offset += sizeof(this->ir_reading);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/HardwareState"; };
    const char * getMD5(){ return "2fc62010ae26b6b9dc2fbf0c1df77e7a"; };

  };

}
#endif