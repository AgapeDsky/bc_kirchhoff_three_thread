#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.

/**
 * Quadrature Encoder Interface.
 */
class EncoderMotor {

public:

    typedef enum Encoding 
    {
        X2_ENCODING,
        X4_ENCODING
    } 
    Encoding;
    
    /** Membuat interface dari encoder    
     *
     * @param inA DigitalIn, out A dari encoder
     * @param inB DigitalIn, out B dari encoder
     */
    EncoderMotor(PinName channelA, PinName channelB, float pulsesPerRev, Encoding encoding = X2_ENCODING);
    
    /**
     * Reset encoder.
     *
     * Menset pulse dan revolusi/putaran menjadi 0
     */
    void reset(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    int getPulses(bool reset);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    float getRevolutions(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    //int readDeltaPulses(void);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    //int readDeltaRevolutions(void);

    /**
     * Disable encoder interrupt pins
     *
     * @return Disable interrupt pins
     */  
    void disableInterrupts(void);

    /**
     * Enable encoder interrupt pins
     *
     * @return Enable interrupt pins
     */  
    void enableInterrupts(void);


private:

    /**
     * Menghitung pulse
     *
     * Digunakan setiap rising/falling edge baik channel A atau B
     * Membaca putaran CW atau CCW => mengakibatkan pertambahan/pengurangan pulse
     */
    void encode(void);

    /**
     * Indeks setiap rising edge untuk menghitung putaran
     * Nilai bertambah 1
     */
    //void index(void);

    Encoding encoding_;

    InterruptIn channelA_;
    InterruptIn channelB_;
    //InterruptIn index_;

    float pulsesPerRev_;
    int prevState_;
    int currState_;

    volatile int pulses_;
    volatile float revolutions_;

};

#endif /* ENCODERMOTOR_H */
