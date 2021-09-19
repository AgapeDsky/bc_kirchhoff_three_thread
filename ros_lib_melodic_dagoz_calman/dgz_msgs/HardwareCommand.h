#ifndef _ROS_dgz_msgs_HardwareCommand_h
#define _ROS_dgz_msgs_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dgz_msgs
{

  class HardwareCommand : public ros::Msg
  {
    public:
      typedef double _base_motor_1_pwm_type;
      _base_motor_1_pwm_type base_motor_1_pwm;
      typedef double _base_motor_2_pwm_type;
      _base_motor_2_pwm_type base_motor_2_pwm;
      typedef double _base_motor_3_pwm_type;
      _base_motor_3_pwm_type base_motor_3_pwm;
      typedef double _base_motor_4_pwm_type;
      _base_motor_4_pwm_type base_motor_4_pwm;
      typedef double _dribbler_motor_l_pwm_type;
      _dribbler_motor_l_pwm_type dribbler_motor_l_pwm;
      typedef double _dribbler_motor_r_pwm_type;
      _dribbler_motor_r_pwm_type dribbler_motor_r_pwm;
      typedef double _kicker_pwm_type;
      _kicker_pwm_type kicker_pwm;
      typedef bool _is_shoot_type;
      _is_shoot_type is_shoot;

    HardwareCommand():
      base_motor_1_pwm(0),
      base_motor_2_pwm(0),
      base_motor_3_pwm(0),
      base_motor_4_pwm(0),
      dribbler_motor_l_pwm(0),
      dribbler_motor_r_pwm(0),
      kicker_pwm(0),
      is_shoot(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_base_motor_1_pwm;
      u_base_motor_1_pwm.real = this->base_motor_1_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_1_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_1_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_1_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_1_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_base_motor_1_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_base_motor_1_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_base_motor_1_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_base_motor_1_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->base_motor_1_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_2_pwm;
      u_base_motor_2_pwm.real = this->base_motor_2_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_2_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_2_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_2_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_2_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_base_motor_2_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_base_motor_2_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_base_motor_2_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_base_motor_2_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->base_motor_2_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_3_pwm;
      u_base_motor_3_pwm.real = this->base_motor_3_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_3_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_3_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_3_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_3_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_base_motor_3_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_base_motor_3_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_base_motor_3_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_base_motor_3_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->base_motor_3_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_4_pwm;
      u_base_motor_4_pwm.real = this->base_motor_4_pwm;
      *(outbuffer + offset + 0) = (u_base_motor_4_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motor_4_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motor_4_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motor_4_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_base_motor_4_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_base_motor_4_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_base_motor_4_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_base_motor_4_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->base_motor_4_pwm);
      union {
        double real;
        uint64_t base;
      } u_dribbler_motor_l_pwm;
      u_dribbler_motor_l_pwm.real = this->dribbler_motor_l_pwm;
      *(outbuffer + offset + 0) = (u_dribbler_motor_l_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_l_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_l_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_l_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_motor_l_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_motor_l_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_motor_l_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_motor_l_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_motor_l_pwm);
      union {
        double real;
        uint64_t base;
      } u_dribbler_motor_r_pwm;
      u_dribbler_motor_r_pwm.real = this->dribbler_motor_r_pwm;
      *(outbuffer + offset + 0) = (u_dribbler_motor_r_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motor_r_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motor_r_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motor_r_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_motor_r_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_motor_r_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_motor_r_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_motor_r_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_motor_r_pwm);
      union {
        double real;
        uint64_t base;
      } u_kicker_pwm;
      u_kicker_pwm.real = this->kicker_pwm;
      *(outbuffer + offset + 0) = (u_kicker_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kicker_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kicker_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kicker_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kicker_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kicker_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kicker_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kicker_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_shoot;
      u_is_shoot.real = this->is_shoot;
      *(outbuffer + offset + 0) = (u_is_shoot.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_shoot);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_base_motor_1_pwm;
      u_base_motor_1_pwm.base = 0;
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_base_motor_1_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->base_motor_1_pwm = u_base_motor_1_pwm.real;
      offset += sizeof(this->base_motor_1_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_2_pwm;
      u_base_motor_2_pwm.base = 0;
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_base_motor_2_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->base_motor_2_pwm = u_base_motor_2_pwm.real;
      offset += sizeof(this->base_motor_2_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_3_pwm;
      u_base_motor_3_pwm.base = 0;
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_base_motor_3_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->base_motor_3_pwm = u_base_motor_3_pwm.real;
      offset += sizeof(this->base_motor_3_pwm);
      union {
        double real;
        uint64_t base;
      } u_base_motor_4_pwm;
      u_base_motor_4_pwm.base = 0;
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_base_motor_4_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->base_motor_4_pwm = u_base_motor_4_pwm.real;
      offset += sizeof(this->base_motor_4_pwm);
      union {
        double real;
        uint64_t base;
      } u_dribbler_motor_l_pwm;
      u_dribbler_motor_l_pwm.base = 0;
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dribbler_motor_l_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dribbler_motor_l_pwm = u_dribbler_motor_l_pwm.real;
      offset += sizeof(this->dribbler_motor_l_pwm);
      union {
        double real;
        uint64_t base;
      } u_dribbler_motor_r_pwm;
      u_dribbler_motor_r_pwm.base = 0;
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dribbler_motor_r_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dribbler_motor_r_pwm = u_dribbler_motor_r_pwm.real;
      offset += sizeof(this->dribbler_motor_r_pwm);
      union {
        double real;
        uint64_t base;
      } u_kicker_pwm;
      u_kicker_pwm.base = 0;
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kicker_pwm = u_kicker_pwm.real;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_shoot;
      u_is_shoot.base = 0;
      u_is_shoot.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_shoot = u_is_shoot.real;
      offset += sizeof(this->is_shoot);
     return offset;
    }

    virtual const char * getType() { return "dgz_msgs/HardwareCommand"; };
    virtual const char * getMD5() { return "69b3697824744dbfc70ba0a6085dc33e"; };

  };

}
#endif
