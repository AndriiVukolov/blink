/*
 * lux.h
 *
 *  Created on: May 13, 2025
 *      Author: CS60
 */

#ifndef INC_LUX_H_
#define INC_LUX_H_
//Register addresses
#define REG_RESULT  0x00
#define REG_CONFIG  0x01
#define REG_LOWL    0x02
#define REG_HIGHL   0x03
#define REG_MID     0x7E
#define REG_DID     0x7F
//Default values
#define LUX_DELAY       100
#define ADDRESS         0b01000110
#define RANGE           0b1100 //1100b (0Ch), the device operates in automatic full-scale setting mode
#define CONVERSION_TIME 1 //The conversion time 1 = 800 ms
#define CONVERSION_MODE 0b11 //11 = Continuous conversions
#define LATCH           1 // Latch field 1 = The device functions in latched window-style comparison operation, latching the interrupt reporting mechanisms until a user-controlled clearing event.
#define POLARITY        0 // Polarity field: 0 = The INT pin reports active low, pulling the pin low upon an interrupt event
#define MASK_EXPONENT   0 //Mask exponent field
#define FAULT_COUNT     0 //Fault count field - 00 = One fault count (default)

typedef struct {
    uint16_t ADD; //address
    double VAL;//current value
    uint16_t RN; //Range - e full-scale lux range of the device
    uint16_t CT; //Conversion time field
    uint16_t M ; //Mode of conversion operation field
    uint16_t OVF; //Overflow flag field (read-only)
    uint16_t CRF; //Conversion ready field (read-only)
    uint16_t FH; //Flag high field (read-only)
    uint16_t FL; //Flag low field (read-only)
    uint16_t L; //Latch field
    uint16_t POL; //Polarity field
    uint16_t ME; //Mask exponent field
    uint16_t FC; //Fault count field
}lux_t;

void luxInit(lux_t * lux);
void readConfig(lux_t * lux);
void readLux(lux_t * lux);

#endif /* INC_LUX_H_ */
