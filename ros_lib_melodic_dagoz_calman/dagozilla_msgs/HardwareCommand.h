#ifndef _ROS_dagozilla_msgs_HardwareCommand_h
#define _ROS_dagozilla_msgs_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msgs
{

  class HardwareCommand : public ros::Msg
  {
    public:
      typedef float _base_motor_1_pwm_type;
      _base_motor_1_pwm_type base_motor_1_pwm;
      typedef float _base_motor_2_pwm_type;
      _base_motor_2_pwm_type base_motor_2_pwm;
      typedef float _base_motor_3_pwm_type;
      _base_motor_3_pwm_type base_motor_3_pwm;
      typedef float _base_motor_4_pwm_type;
      _base_motor_4_pwm_type base_motor_4_pwm;
      typedef float _dribbler_motor_l_pwm_type;
      _dribbler_motor_l_pwm_type dribbler_motor_l_pwm;
      typedef float _dribbler_motor_r_pwm_type;
      _dribbler_motor_r_pwm_type dribbler_motor_r_pwm;
      typedef float _kicker_pwm_type;
      _kicker_pwm_type kicker_pwm;
      typedef bool _kicker_mode_type;
      _kicker_mode_type kicker_mode;

    HardwareCommand():
      base_motor_1_pwm(0),
      base_motor_2_pwm(0),
      base_motor_3_pwm(0),
      base_motor_4_pwm(0),
      dribbler_motor_l_pwm(0),
      dribbler_motor_r_pwm(0),
      kicker_pwm(0),
      kicker_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_motor_1_pwm;
      u_base_motor_1_pwm.real = this->base_motor_1_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_1_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_1_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_1_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_1_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_1_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_2_pwm;
      u_base_motor_2_pwm.real = this->base_motor_2_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_2_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_2_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_2_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_2_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_2_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_3_pwm;
      u_base_motor_3_pwm.real = this->base_motor_3_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_3_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_3_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_3_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_3_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_3_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_4_pwm;
      u_base_motor_4_pwm.real = this->base_motor_4_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_4_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_4_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_4_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_4_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motor_4_pwm);
      union {
        float real;
        uint32_t base;
      } u_dribbler_motor_l_pwm;
      u_dribbler_motor_l_pwm.real = this->dribbler_motor_l_pwm;
      *(outbuffer + offset + 0) = (u_dribbler_motor_l_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_l_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_l_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_l_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_l_pwm);
      union {
        float real;
        uint32_t base;
      } u_dribbler_motor_r_pwm;
      u_dribbler_motor_r_pwm.real = this->dribbler_motor_r_pwm;
      *(outbuffer + offset + 0) = (u_dribbler_motor_r_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_r_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_r_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_r_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motor_r_pwm);
      union {
        float real;
        uint32_t base;
      } u_kicker_pwm;
      u_kicker_pwm.real = this->kicker_pwm;
      *(outbuffer + offset + 0) = (u_kicker_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kicker_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kicker_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kicker_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_kicker_mode;
      u_kicker_mode.real = this->kicker_mode;
      *(outbuffer + offset + 0) = (u_kicker_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kicker_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_motor_1_pwm;
      u_base_motor_1_pwm.base = 0;
      u_base_motor_1_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_1_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_1_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_1_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_1_pwm = u_base_motor_1_pwm.real;
      offset += sizeof(this->base_motor_1_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_2_pwm;
      u_base_motor_2_pwm.base = 0;
      u_base_motor_2_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_2_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_2_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_2_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_2_pwm = u_base_motor_2_pwm.real;
      offset += sizeof(this->base_motor_2_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_3_pwm;
      u_base_motor_3_pwm.base = 0;
      u_base_motor_3_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_3_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_3_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_3_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_3_pwm = u_base_motor_3_pwm.real;
      offset += sizeof(this->base_motor_3_pwm);
      union {
        float real;
        uint32_t base;
      } u_base_motor_4_pwm;
      u_base_motor_4_pwm.base = 0;
      u_base_motor_4_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_4_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_4_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_4_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_motor_4_pwm = u_base_motor_4_pwm.real;
      offset += sizeof(this->base_motor_4_pwm);
      union {
        float real;
        uint32_t base;
      } u_dribbler_motor_l_pwm;
      u_dribbler_motor_l_pwm.base = 0;
      u_dribbler_motor_l_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_l_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_l_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_l_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_l_pwm = u_dribbler_motor_l_pwm.real;
      offset += sizeof(this->dribbler_motor_l_pwm);
      union {
        float real;
        uint32_t base;
      } u_dribbler_motor_r_pwm;
      u_dribbler_motor_r_pwm.base = 0;
      u_dribbler_motor_r_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_r_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_r_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_r_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_motor_r_pwm = u_dribbler_motor_r_pwm.real;
      offset += sizeof(this->dribbler_motor_r_pwm);
      union {
        float real;
        uint32_t base;
      } u_kicker_pwm;
      u_kicker_pwm.base = 0;
      u_kicker_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kicker_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kicker_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kicker_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kicker_pwm = u_kicker_pwm.real;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_kicker_mode;
      u_kicker_mode.base = 0;
      u_kicker_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kicker_mode = u_kicker_mode.real;
      offset += sizeof(this->kicker_mode);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/HardwareCommand"; };
    const char * getMD5(){ return "57c2351b2edb2a9ec76480dbab0eed5e"; };

  };

}
#endif