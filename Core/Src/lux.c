/*
 * lux.c
 *
 *  Created on: May 13, 2025
 *      Author: CS60
 */

#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include "lux.h"
#include <math.h>
lux_t lux1 = {0};

void luxInit(lux_t * lux)
{
    uint16_t val;
    lux->ADD = (0b01000110 << 1),
    lux->VAL = 0,
    lux->RN = 0b1100, //1100b (0Ch), the device operates in automatic full-scale setting mode
    lux->CT = 1, //The conversion time 1 = 800 ms
    lux->M  = 0b11, //11 = Continuous conversions
    lux->OVF = 0, //Overflow flag field (read-only)
    lux->CRF = 0, //Conversion ready field (read-only)
    lux->FH = 0, //Flag high field (read-only)
    lux->FL = 0, //Flag low field (read-only)
    lux->L = 1,  //Latch field 1 = = The device functions in latched window-style comparison operation, latching the interrupt reporting mechanisms until a user-controlled clearing event.
    lux->POL = 0, //0 = The INT pin reports active low, pulling the pin low upon an interrupt event
    lux->ME = 0, //Mask exponent field
    lux->FC = 0; //00 = One fault count (default)

    val = 0b1100111000010100;
    writeReg (lux, REG_CONFIG, val);
}

uint16_t readConfig(lux_t  * lux)
{
    uint16_t res;
    res = (uint16_t)readReg(lux, REG_CONFIG);
    lux->RN  = (res & 0xF000)>>12;
    lux->CT  = (res & 0x0800)>>11;
    lux->M   = (res & 0x0600)>>9;
    lux->OVF = (res & 0x0100)>>8;
    lux->CRF = (res & 0x0080)>>7;
    lux->FH  = (res & 0x0040)>>6;
    lux->FL  = (res & 0x0020)>>5;
    lux->L   = (res & 0x0010)>>4;
    lux->POL = (res & 0x0008)>>3;
    lux->ME  = (res & 0x0006)>>2;
    lux->FC  = (res & 0x0003);

    return res;
}
void writeReg(lux_t * loclux, uint8_t reg, uint16_t val)
{
    static uint8_t buf[3] = {0};

    buf[0] = reg;
    buf[1] = ((val & 0xFF00) >> 8);
    buf[2] = (val & 0x00FF);

    if (HAL_I2C_Master_Transmit(&hi2c1, loclux->ADD, buf, 3, LUX_DELAY) != HAL_OK) print("Transmission error");
}

uint16_t readReg(lux_t * loclux, uint8_t reg)
{
    static uint8_t buf[2] = {0};
    static uint16_t result;

    if (HAL_I2C_Master_Transmit(&hi2c1, loclux->ADD, &reg, 1, LUX_DELAY) != HAL_OK) print("Transmission error");
    if (HAL_I2C_Master_Receive(&hi2c1, loclux->ADD, buf, 2, LUX_DELAY) != HAL_OK) print("Reception error");
    result = ((buf[0] << 8) | buf[1]);
    return result;
}

void readLux(lux_t * loclux)
{
    uint16_t R = 0;
    uint16_t E = 0;
    uint16_t MLSB = 0;
    double exponenta = 0;

    MLSB = readReg(loclux, REG_RESULT);
    E = (MLSB & 0xF000) >> 12;
    R = MLSB & 0x0FFF;
    exponenta = 0.01 * pow(2, E);
    loclux->VAL = exponenta * R;
}
