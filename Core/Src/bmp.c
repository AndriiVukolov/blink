/*
 * bmp.c
 *
 *  Created on: May 14, 2025
 *      Author: CS60
 */

#include "bmp.h"
#include "spi.h"
#include "microrl.h"
#include <stdio.h>

static uint64_t buffPressureData[BUFF_SIZE] = {0};//pressure data buffer
static uint16_t bIndex = 0;//pointer to last value in pressure data buffer
static BMP390_calib_data_t cdata;
struct calcoef {
        uint16_t t1:16;
        uint16_t t2:16;
        uint8_t  t3:8;
        uint16_t p1:16;
        uint16_t p2:16;
        uint8_t  p3:8;
        uint8_t  p4:8;
        uint16_t p5:16;
        uint16_t p6:16;
        uint8_t  p7:8;
        uint8_t  p8:8;
        uint16_t p9:16;
        uint8_t  p10:8;
        uint8_t  p11:8;
    }coefs;
uint16_t nvm_par[CALIB_QTY];


static void ssEnable(void)
{
    HAL_GPIO_WritePin(CHIP_SELECT_GPIO_Port, CHIP_SELECT_Pin, GPIO_PIN_RESET);
}
static void ssDisable(void)
{
    HAL_GPIO_WritePin(CHIP_SELECT_GPIO_Port, CHIP_SELECT_Pin, GPIO_PIN_SET);
}
SPI_HandleTypeDef SpiInit(uint8_t num)
{
    SPI_HandleTypeDef * spiHandle = NULL;

    if ((num < 1) || (num > 4)) return;
    else
    {
        if (num == 1)
        {
            spiHandle = &hspi1;
            MX_SPI1_Init();
        }
    }
    return (*spiHandle);
}

void bmpWriteByte(bmp_t *bmp, uint8_t regAddress, uint8_t sByte)
{
    uint8_t packet[2] = {0};
    uint8_t dummy[2] = {0};
    uint8_t errCode;
    char stringBuf[60];


    packet[0] = (regAddress & (~0x80));//write command
    packet[1] = sByte;
    ssEnable();
    errCode = HAL_SPI_TransmitReceive(bmp->hspi, packet, dummy, 2, BMP_TIMEOUT);
    ssDisable();
    if (errCode != HAL_OK)
    {
        sprintf(stringBuf, "SPI Transmission error (bmpWrite) = %X \r\n", (unsigned int)bmp->hspi->ErrorCode);
        print(stringBuf);
    }
}

uint8_t bmpReadByte(bmp_t *bmp, uint8_t regAddress)
{
    uint8_t packet[2] = {0};
    uint8_t buffer[3] = {0};

    packet[0] = (regAddress | 0x80);//read command
    ssEnable();

    if (HAL_SPI_TransmitReceive(bmp->hspi, packet, buffer, 3, BMP_TIMEOUT) != HAL_OK) {print("SPI Transmission error (bmpRead) \n\r");}
    ssDisable();
    return (buffer[2]);
}

void setConfig(bmp_t *bmp)
{
    uint8_t bufByte = 0;

    //FIFO watermark
    bmpWriteByte(bmp, ADD_FIFO_WTM_0, (DEF_WTM & 0xFF));
    bmpWriteByte(bmp, ADD_FIFO_WTM_1, (DEF_WTM & 0x100)>>8);
    //FIFO config 1
    bufByte = (DEF_FIFO_TEMP_EN << 4) | (DEF_FIFO_PRESS_EN << 3) | (DEF_FIFO_TIME_EN << 2) | (DEF_FIFO_STOP_OF << 1) | DEF_FIFO_MODE;
    bmpWriteByte(bmp, ADD_FIFO_CONFIG_1, bufByte);
    //FIFO config 2
    bufByte = 0;
    //bufByte = (bmp->DATA_SEL << 3) | (bmp->FIFO_SBSMP);
    bufByte = (DEF_DATA_SELECT << 3) | (DEF_FIFO_SUBSAMPLING);
    bmpWriteByte(bmp, ADD_FIFO_CONFIG_2, bufByte);
    //INT_CTRL - interrupt configuration
    bufByte = 0;
    //bufByte = ((bmp->DRDY_EN << 6)|(bmp->INT_DS << 5)|(bmp->FULL_EN << 4)|(bmp->FWTM_EN << 3)|(bmp->INT_LATCH << 2)|(bmp->INT_LVL << 1)|bmp->INT_OD);
    bufByte = ((DEF_DRDY_EN << 6)|(DEF_INT_DS << 5)|(DEF_FULL_EN << 4)|(DEF_FWTM_EN << 3)|(DEF_INT_LATCH << 2)|(DEF_INT_LVL << 1)|DEF_INT_OD);
    bmpWriteByte(bmp, ADD_INT_CTRL, bufByte);
    //IF_CONF - The serial interface settings
    bufByte = 0;
    //bufByte = (bmp->I2C_WDT_S << 2)|(bmp->I2C_WDT_EN << 1)|bmp->SPI3_M;
    bufByte = (DEF_I2C_WDT_S << 2)|(DEF_I2C_WDT_EN << 1)|DEF_SPI3_M;
    bmpWriteByte(bmp, (ADD_IF_CONF), bufByte);
    //PWR_CTRL register enables or disables pressure and temperature measurement.
    bufByte = 0;
    bufByte = (DEF_PWR_MODE << 4)|(DEF_PW_TEMP << 1)|DEF_PW_PRESS;
    bmpWriteByte(bmp, (ADD_PWR_CTRL), bufByte);
    //OSR - oversampling settings
    bufByte = 0;
    bufByte = (DEF_OSRT << 3)|DEF_OSRP;
    bmpWriteByte(bmp, (ADD_OSR), bufByte);
    //ODR register set the configuration of the output data rates by means of setting the subdivision/subsampling
    bmpWriteByte(bmp, (ADD_ODR), DEF_ODR);
    //CONFIG register controls the IIR filter coefficients
    bmpWriteByte(bmp, (ADD_CONFIG), (DEF_CONFIG << 1));
}

void readBmpConfig(bmp_t *bmp)
{
    bmp->CONFIG = bmpReadByte(bmp, ADD_CONFIG);
}
uint8_t readIntCtrl(bmp_t *bmp)
{
    uint8_t buf = 0;
    buf = bmpReadByte(bmp, ADD_INT_CTRL);
    bmp->DRDY_EN = (buf & 0x40)>>6;
    bmp->INT_DS = (buf & 0x20)>>5;
    bmp->FULL_EN = (buf & 0x10)>>4;
    bmp->FWTM_EN = (buf & 0x08)>>3;
    bmp->INT_LATCH = (buf & 0x04)>>2;
    bmp->INT_LVL = (buf & 0x02)>>1;
    bmp->INT_OD = (buf & 0x01)>>0;
    return(buf);
}

void setInt(bmp_t *bmp)
{
    uint8_t bufByte = 0;

    bufByte = ((DEF_DRDY_EN << 6)|(DEF_INT_DS << 5)|(DEF_FULL_EN << 4)|(DEF_FWTM_EN << 3)|(DEF_INT_LATCH << 2)|(DEF_INT_LVL << 1)|DEF_INT_OD);
    bmp->DRDY_EN = DEF_DRDY_EN;
    bmp->INT_DS = DEF_INT_DS;
    bmp->FULL_EN = DEF_FULL_EN;
    bmp->FWTM_EN = DEF_FWTM_EN;
    bmp->INT_LATCH = DEF_INT_LATCH;
    bmp->INT_LVL = DEF_INT_LVL;
    bmp->INT_OD = DEF_INT_OD;
    bmpWriteByte(bmp, ADD_INT_CTRL, bufByte);
}
void readInt(bmp_t *bmp)
{
    uint8_t buf = 0;
    uint8_t str[30] = {0};


    buf = bmpReadByte(bmp, ADD_INT_CTRL);
    sprintf(str, "Interrupt config byte: %X \r\n", buf);
    print(str);
}

static void toBuff(uint32_t val)
{
    if (bIndex < BUFF_SIZE)
    {
        buffPressureData[bIndex++] = val;
    }
    else bIndex = 0;
}


void readPressure(bmp_t *bmp)
{
    uint8_t buf[3] = {0};

    buf[0] = bmpReadByte(bmp, ADD_DATA_0);
    buf[1] = bmpReadByte(bmp, ADD_DATA_1);
    buf[2] = bmpReadByte(bmp, ADD_DATA_2);

    //bmpReadStream(bmp, buf, ADD_PRESS_START, ADD_PRESS_BYTE_QTY);
    bmp->PDATA = (0 | (buf[2]<<16) | (buf[1]<<8) | buf[0]);

    toBuff(bmp->PDATA);
}
uint32_t readTemperature(bmp_t *bmp)
{
    uint8_t buf[3] = {0};
    buf[0] = bmpReadByte(bmp, ADD_DATA_3);
    buf[1] = bmpReadByte(bmp, ADD_DATA_4);
    buf[2] = bmpReadByte(bmp, ADD_DATA_5);
    bmp->TDATA = (buf[2]<<16) | (buf[1]<<8) | buf[0];
    return bmp->TDATA;
}
uint8_t readStatus(bmp_t *bmp)
{
    uint8_t buf = 0;
    buf = bmpReadByte(bmp, ADD_STATUS);
    bmp->DRDY_TEMP = (buf & 0x40);
    bmp->DRDY_PRESS = (buf & 0x20);
    bmp->CRDY = (buf & 0x10);
    return buf;
}
uint8_t readID(bmp_t *bmp)
{
    uint8_t buf = 0;
    buf = bmpReadByte(bmp, ADD_CHIP_ID);
    bmp->CHIP_ID = buf;
    return buf;
}
void readAllRegs(bmp_t *bmp)
{
    uint8_t buf[BUFF_WHOLE_SIZE];
    char str[30];

    for (uint16_t i = 0; i < BUFF_WHOLE_SIZE; i++)
    {
        buf[i] = bmpReadByte(bmp, i);
        sprintf(str, "Reg %X : %X \n\r", i, buf[i]);
        print(str);
    }
}
void readCalibration(bmp_t *bmp)
{
    uint8_t ind = 0;
    double temp_var;

    for (uint8_t i = CALIB_START; i <= CALIB_END; i++)
    {
        nvm_par[ind++] = bmpReadByte(bmp, i);
    }

    coefs.t1 = nvm_par[0] | (nvm_par[1]<<8);
    coefs.t2 = nvm_par[2] | (nvm_par[3]<<8);
    coefs.t3 = nvm_par[4];
    coefs.p1 = nvm_par[5] | (nvm_par[6]<<8);
    coefs.p2 = nvm_par[7] | (nvm_par[8]<<8);
    coefs.p3 = nvm_par[9];
    coefs.p4 = nvm_par[10];
    coefs.p5 = nvm_par[11] | (nvm_par[12]<<8);
    coefs.p6 = nvm_par[13] | (nvm_par[14]<<8);
    coefs.p7 = nvm_par[15];
    coefs.p8 = nvm_par[16];
    coefs.p9 = nvm_par[17] | (nvm_par[18]<<8);
    coefs.p10 = nvm_par[19];
    coefs.p11 = nvm_par[20];


    //cdata.par_t1 = ((double)coefs.t1)/pow(2,(-8));
    temp_var = 0.00390625f;
    cdata.par_t1 = ((double)coefs.t1) / temp_var;
    temp_var = 1073741824.0f;
    cdata.par_t2 = ((double)coefs.t2)/temp_var;
    temp_var = 281474976710656.0f;
    cdata.par_t3 = ((double)coefs.t3)/temp_var;
    temp_var = 1048576.0f;
    cdata.par_p1 = ((double)coefs.p1)/temp_var;
    temp_var = 536870912.0f;
    cdata.par_p2 = ((double)coefs.p2)/temp_var;
    temp_var = 4294967296.0f;
    cdata.par_p3 = ((double)coefs.p3)/temp_var;
    temp_var = 137438953472.0f;
    cdata.par_p4 = ((double)coefs.p4)/temp_var;
    temp_var = 0.125f;
    cdata.par_p5 = ((double)coefs.p5)/temp_var;
    temp_var = 64.0f;
    cdata.par_p6 = ((double)coefs.p6)/temp_var;
    temp_var = 256.0f;
    cdata.par_p7 = ((double)coefs.p7)/temp_var;
    temp_var = 32768.0f;
    cdata.par_p8 = ((double)coefs.p8)/temp_var;
    temp_var = 281474976710656.0f;
    cdata.par_p9 = ((double)coefs.p9)/temp_var;
    temp_var = 281474976710656.0f;
    cdata.par_p10 = ((double)coefs.p10)/temp_var;
    temp_var = 36893488147419103232.0f;
    cdata.par_p11 = ((double)coefs.p11)/temp_var;
}

int8_t compensate_pressure(bmp_t *bmp, uint64_t *pressure)
{
    int8_t rslt = BMP3_OK;
    BMP390_calib_data_t *reg_calib_data;
    reg_calib_data = &cdata;
    uint16_t resolution = 1000;

    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;
    uint64_t uncomp_press;


    switch (bmp->OSRP)
    {
        case OSRx1:
        {
            resolution = 2640;
            break;
        }
        case OSRx2:
        {
            resolution = 1320;
            break;
        }
        case OSRx4:
        {
            resolution = 660;
            break;
        }
        case OSRx8:
        {
            resolution = 330;
            break;
        }
        case OSRx16:
        {
            resolution = 170;
            break;
        }
        case OSRx32:
        {
            resolution = 85;
            break;
        }
    }

    uncomp_press = (bmp->PDATA * resolution) / 1000;

    partial_data1 = (int64_t)(reg_calib_data->tlin * reg_calib_data->tlin);
    partial_data2 = (int64_t)(partial_data1 / 64);
    partial_data3 = (int64_t)((partial_data2 * reg_calib_data->tlin) / 256);
    partial_data4 = (int64_t)((reg_calib_data->par_p8 * partial_data3) / 32);
    partial_data5 = (int64_t)((reg_calib_data->par_p7 * partial_data1) * 16);
    partial_data6 = (int64_t)((reg_calib_data->par_p6 * reg_calib_data->tlin) * 4194304);
    offset = (int64_t)((reg_calib_data->par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6);
    partial_data2 = (int64_t)((reg_calib_data->par_p4 * partial_data3) / 32);
    partial_data4 = (int64_t)((reg_calib_data->par_p3 * partial_data1) * 4);
    partial_data5 = (int64_t)((reg_calib_data->par_p2 - (int32_t)16384) * reg_calib_data->tlin * 2097152);
    sensitivity   = (int64_t)(((reg_calib_data->par_p1 - (int32_t)16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5);
    partial_data1 = (int64_t)((sensitivity / 16777216) * uncomp_press);
    partial_data2 = (int64_t)(reg_calib_data->par_p10 * reg_calib_data->tlin);
    partial_data3 = (int64_t)(partial_data2 + ((int32_t)65536 * reg_calib_data->par_p9));
    partial_data4 = (int64_t)((partial_data3 * uncomp_press) / (int32_t)8192);

    /* dividing by 10 followed by multiplying by 10
     * To avoid overflow caused by (uncomp_data->pressure * partial_data4)
     */
    partial_data5 = (int64_t)((uncomp_press * (partial_data4 / 10)) / (int32_t)512);
    partial_data5 = (int64_t)(partial_data5 * 10);
    partial_data6 = (int64_t)(uncomp_press * uncomp_press);
    partial_data2 = (int64_t)((reg_calib_data->par_p11 * partial_data6) / (int32_t)65536);
    partial_data3 = (int64_t)((int64_t)(partial_data2 * uncomp_press) / 128);
    partial_data4 = (int64_t)((offset / 4) + partial_data1 + partial_data5 + partial_data3);
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

    if (comp_press < BMP3_MIN_PRES_INT)
    {
        comp_press = BMP3_MIN_PRES_INT;
        rslt = BMP3_W_MIN_PRES;
    }

    if (comp_press > BMP3_MAX_PRES_INT)
    {
        comp_press = BMP3_MAX_PRES_INT;
        rslt = BMP3_W_MAX_PRES;
    }

    (*pressure) = comp_press;

    return rslt;
}

void softReset(bmp_t *bmp)
{
    bmpWriteByte(bmp, ADD_CMD, CMD_SOFTRESET);
}

void bmpInit(bmp_t *bmp)
{
    ssDisable();
    //softReset(bmp);
    setConfig(bmp);
    print("\r\nBMP is set \r\n");
    readCalibration(bmp);
    print("Calibration coefficients are read \r\n");
//    readAllRegs(bmp);
//    setInt(bmp);
//    print("====================================\r\n");
//    readInt(bmp);
}



