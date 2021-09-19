#ifndef _ROS_dgz_msgs_HardwareState_h
#define _ROS_dgz_msgs_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dgz_msgs
{

  class HardwareState : public ros::Msg
  {
    public:
      typedef int32_t _base_motor_1_pulse_delta_type;
      _base_motor_1_pulse_delta_type base_motor_1_pulse_delta;
      typedef int32_t _base_motor_2_pulse_delta_type;
      _base_motor_2_pulse_delta_type base_motor_2_pulse_delta;
      typedef int32_t _base_motor_3_pulse_delta_type;
      _base_motor_3_pulse_delta_type base_motor_3_pulse_delta;
      typedef int32_t _base_motor_4_pulse_delta_type;
      _base_motor_4_pulse_delta_type base_motor_4_pulse_delta;
      typedef int32_t _base_encoder_1_pulse_delta_type;
      _base_encoder_1_pulse_delta_type base_encoder_1_pulse_delta;
      typedef int32_t _base_encoder_2_pulse_delta_type;
      _base_encoder_2_pulse_delta_type base_encoder_2_pulse_delta;
      typedef int32_t _base_encoder_3_pulse_delta_type;
      _base_encoder_3_pulse_delta_type base_encoder_3_pulse_delta;
      typedef int32_t _dribbler_motor_l_pulse_delta_type;
      _dribbler_motor_l_pulse_delta_type dribbler_motor_l_pulse_delta;
      typedef int32_t _dribbler_motor_r_pulse_delta_type;
      _dribbler_motor_r_pulse_delta_type dribbler_motor_r_pulse_delta;
      typedef double _dribbler_potentio_l_reading_type;
      _dribbler_potentio_l_reading_type dribbler_potentio_l_reading;
      typedef double _dribbler_potentio_r_reading_type;
      _dribbler_potentio_r_reading_type dribbler_potentio_r_reading;
      typedef double _compass_reading_type;
      _compass_reading_type compass_reading;
      typedef double _ir_reading_type;
      _ir_reading_type ir_reading;

    HardwareState():
      base_motor_1_pulse_delta(0),
      base_motor_2_pulse_delta(0),
      base_motor_3_pulse_delta(0),
      base_motor_4_pulse_delta(0),
      base_encoder_1_pulse_delta(0),
      base_encoder_2_pulse_delta(0),
      base_encoder_3_pulse_delta(0),
      dribbler_motor_l_pulse_delta(0),
      dribbler_motor_r_pulse_delta(0),
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
      } u_base_motor_1_pulse_delta;
      u_base_motor_1_pulse_delta.real = this->base_motor_1_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_motor_1_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_1_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_1_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_1_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_1_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_2_pulse_delta;
      u_base_motor_2_pulse_delta.real = this->base_motor_2_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_motor_2_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_2_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_2_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_2_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_2_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_3_pulse_delta;
      u_base_motor_3_pulse_delta.real = this->base_motor_3_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_motor_3_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_3_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_3_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_3_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_3_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_4_pulse_delta;
      u_base_motor_4_pulse_delta.real = this->base_motor_4_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_motor_4_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_4_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_4_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_4_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_4_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_1_pulse_delta;
      u_base_encoder_1_pulse_delta.real = this->base_encoder_1_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_encoder_1_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_1_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_1_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_1_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_1_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_2_pulse_delta;
      u_base_encoder_2_pulse_delta.real = this->base_encoder_2_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_encoder_2_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_2_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_2_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_2_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_2_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_3_pulse_delta;
      u_base_encoder_3_pulse_delta.real = this->base_encoder_3_pulse_delta;
      *(outbuffer + offset + 0) = (u_base_encoder_3_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoder_3_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoder_3_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoder_3_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoder_3_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_l_pulse_delta;
      u_dribbler_motor_l_pulse_delta.real = this->dribbler_motor_l_pulse_delta;
      *(outbuffer + offset + 0) = (u_dribbler_motor_l_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_l_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_l_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_l_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_l_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_r_pulse_delta;
      u_dribbler_motor_r_pulse_delta.real = this->dribbler_motor_r_pulse_delta;
      *(outbuffer + offset + 0) = (u_dribbler_motor_r_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_r_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_r_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_r_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_r_pulse_delta);
      union {
        double real;
        uint64_t base;
      } u_dribbler_potentio_l_reading;
      u_dribbler_potentio_l_reading.real = this->dribbler_potentio_l_reading;
      *(outbuffer + offset + 0) = (u_dribbler_potentio_l_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_potentio_l_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_potentio_l_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_potentio_l_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_potentio_l_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_potentio_l_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_potentio_l_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_potentio_l_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_potentio_l_reading);
      union {
        double real;
        uint64_t base;
      } u_dribbler_potentio_r_reading;
      u_dribbler_potentio_r_reading.real = this->dribbler_potentio_r_reading;
      *(outbuffer + offset + 0) = (u_dribbler_potentio_r_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_potentio_r_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_potentio_r_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_potentio_r_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_potentio_r_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_potentio_r_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_potentio_r_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_potentio_r_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_potentio_r_reading);
      union {
        double real;
        uint64_t base;
      } u_compass_reading;
      u_compass_reading.real = this->compass_reading;
      *(outbuffer + offset + 0) = (u_compass_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_compass_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_compass_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_compass_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_compass_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_compass_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_compass_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_compass_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->compass_reading);
      union {
        double real;
        uint64_t base;
      } u_ir_reading;
      u_ir_reading.real = this->ir_reading;
      *(outbuffer + offset + 0) = (u_ir_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ir_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ir_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ir_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ir_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ir_reading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_1_pulse_delta;
      u_base_motor_1_pulse_delta.base = 0;
      u_base_motor_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_1_pulse_delta = u_base_motor_1_pulse_delta.real;
      offset += sizeof(this->base_motor_1_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_2_pulse_delta;
      u_base_motor_2_pulse_delta.base = 0;
      u_base_motor_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_2_pulse_delta = u_base_motor_2_pulse_delta.real;
      offset += sizeof(this->base_motor_2_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_3_pulse_delta;
      u_base_motor_3_pulse_delta.base = 0;
      u_base_motor_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_3_pulse_delta = u_base_motor_3_pulse_delta.real;
      offset += sizeof(this->base_motor_3_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_motor_4_pulse_delta;
      u_base_motor_4_pulse_delta.base = 0;
      u_base_motor_4_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_4_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_4_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_4_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_4_pulse_delta = u_base_motor_4_pulse_delta.real;
      offset += sizeof(this->base_motor_4_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_1_pulse_delta;
      u_base_encoder_1_pulse_delta.base = 0;
      u_base_encoder_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_1_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_1_pulse_delta = u_base_encoder_1_pulse_delta.real;
      offset += sizeof(this->base_encoder_1_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_2_pulse_delta;
      u_base_encoder_2_pulse_delta.base = 0;
      u_base_encoder_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_2_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_2_pulse_delta = u_base_encoder_2_pulse_delta.real;
      offset += sizeof(this->base_encoder_2_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoder_3_pulse_delta;
      u_base_encoder_3_pulse_delta.base = 0;
      u_base_encoder_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_encoder_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_encoder_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_encoder_3_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_encoder_3_pulse_delta = u_base_encoder_3_pulse_delta.real;
      offset += sizeof(this->base_encoder_3_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_l_pulse_delta;
      u_dribbler_motor_l_pulse_delta.base = 0;
      u_dribbler_motor_l_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_l_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_l_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_l_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_l_pulse_delta = u_dribbler_motor_l_pulse_delta.real;
      offset += sizeof(this->dribbler_motor_l_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motor_r_pulse_delta;
      u_dribbler_motor_r_pulse_delta.base = 0;
      u_dribbler_motor_r_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_r_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_r_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_r_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_r_pulse_delta = u_dribbler_motor_r_pulse_delta.real;
      offset += sizeof(this->dribbler_motor_r_pulse_delta);
      union {
        double real;
        uint64_t base;
      } u_dribbler_potentio_l_reading;
      u_dribbler_potentio_l_reading.base = 0;
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dribbler_potentio_l_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dribbler_potentio_l_reading = u_dribbler_potentio_l_reading.real;
      offset += sizeof(this->dribbler_potentio_l_reading);
      union {
        double real;
        uint64_t base;
      } u_dribbler_potentio_r_reading;
      u_dribbler_potentio_r_reading.base = 0;
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dribbler_potentio_r_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dribbler_potentio_r_reading = u_dribbler_potentio_r_reading.real;
      offset += sizeof(this->dribbler_potentio_r_reading);
      union {
        double real;
        uint64_t base;
      } u_compass_reading;
      u_compass_reading.base = 0;
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->compass_reading = u_compass_reading.real;
      offset += sizeof(this->compass_reading);
      union {
        double real;
        uint64_t base;
      } u_ir_reading;
      u_ir_reading.base = 0;
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ir_reading = u_ir_reading.real;
      offset += sizeof(this->ir_reading);
     return offset;
    }

    virtual const char * getType() { return "dgz_msgs/HardwareState"; };
    virtual const char * getMD5() { return "98eb1d85bcb259a05db1004f284168b1"; };

  };

}
#endif
