#ifndef ENCODER_H
#define ENCODER_H
#include "mbed.h"

void EncoderInit(TIM_Encoder_InitTypeDef * encoder, TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);

namespace mbed{
    
    class EncoderDAGOZ {
        
        public :
            EncoderDAGOZ(TIM_TypeDef * _TIM);
            EncoderDAGOZ(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            EncoderDAGOZ(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            int32_t GetCounter(bool reset);
            TIM_HandleTypeDef* GetTimer();
        
        private :
        
        
        protected :
            TIM_Encoder_InitTypeDef encoder;
            TIM_HandleTypeDef timer;
            TIM_TypeDef * TIM;
        
    };

}// namespace mbed

#endif