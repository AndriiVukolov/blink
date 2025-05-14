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

static void writeReg(lux_t * lux, uint8_t reg, uint16_t val)
{
    static uint8_t buf[3] = {0};

    buf[0] = reg;
    buf[1] = ((val & 0xFF00) >> 8);
    buf[2] = (val & 0x00FF);

    if (HAL_I2C_Master_Transmit(&hi2c1, (lux->ADD << 1), buf, 3, LUX_DELAY) != HAL_OK) print("Transmission error (writeReg) ");
}
static uint16_t readReg(lux_t * lux, uint8_t reg)
{
    static uint8_t buf[2] = {0};
    static uint16_t result;

    if (HAL_I2C_Master_Transmit(&hi2c1, (lux->ADD << 1), &reg, 1, LUX_DELAY) != HAL_OK) print("Transmission error (readReg) ");
    if (HAL_I2C_Master_Receive(&hi2c1, (lux->ADD << 1), buf, 2, LUX_DELAY) != HAL_OK) print("Reception error (readReg) ");
    result = ((buf[0] << 8) | buf[1]);
    return result;
}
void luxInit(lux_t * lux)
{
    uint16_t val = 0;
    val = (val | (lux->RN << 12) | (lux->CT << 11) | (lux->M << 9) | (lux->L << 4) | (lux->POL << 3) | (lux->ME << 2) | lux->FC);
    writeReg (lux, REG_CONFIG, val);
}
void readConfig(lux_t  * lux)
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
}
void readLux(lux_t * lux)
{
    uint16_t R = 0;
    uint16_t E = 0;
    uint16_t MLSB = 0;
    double exponenta = 0;

    MLSB = readReg(lux, REG_RESULT);
    E = (MLSB & 0xF000) >> 12;
    R = MLSB & 0x0FFF;
    exponenta = 0.01 * pow(2, E);
    lux->VAL = exponenta * R;
}
