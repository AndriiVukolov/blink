/*
 * lux.h
 *
 *  Created on: May 13, 2025
 *      Author: CS60
 */

#ifndef INC_LUX_H_
#define INC_LUX_H_

#define LUX_DELAY 100
#define LUX_ADDRESS 0b01000110
#define REG_RESULT  0x00
#define REG_CONFIG  0x01
#define REG_LOWL    0x02
#define REG_HIGHL   0x03
#define REG_MID     0x7E
#define REG_DID     0x7F

typedef struct {
    uint8_t ADD; //address
    double VAL;//current value
    uint8_t RN; //Range - e full-scale lux range of the device
    uint8_t CT; //Conversion time field
    uint8_t M ; //Mode of conversion operation field
    uint8_t OVF; //Overflow flag field (read-only)
    uint8_t CRF; //Conversion ready field (read-only)
    uint8_t FH; //Flag high field (read-only)
    uint8_t FL; //Flag low field (read-only)
    uint8_t L; //Latch field
    uint8_t POL; //Polarity field
    uint8_t ME; //Mask exponent field
    uint8_t FC; //Fault count field
}lux_t;

extern lux_t lux1;

void luxInit(lux_t * lux);
uint16_t readConfig(lux_t  * lux);
void writeReg(lux_t * loclux, uint8_t reg, uint16_t val);
uint16_t readReg(lux_t * loclux, uint8_t reg);
void readLux(lux_t * loclux);

#endif /* INC_LUX_H_ */
