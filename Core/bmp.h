/*
 * bmp.h
 *
 *  Created on: May 14, 2025
 *      Author: CS60
 */

#ifndef BMP_H_
#define BMP_H_

#include <stdint.h>
#include "spi.h"
#include "main.h"

typedef enum{
    OSRx1 = 0b000,
    OSRx2 = 0b001,
    OSRx4 = 0b010,
    OSRx8 = 0b011,
    OSRx16 = 0b100,
    OSRx32 = 0b101
}osr_t;

typedef enum {
    ODR_200 = 0x00, //5ms
    ODR_100 = 0x01, //10ms
    ODR_50 = 0x02, //20ms
    ODR_25 = 0x03, //40ms
    ODR_12p5 = 0x04, //80ms
    ODR_6p25 = 0x05, //160ms
    ODR_3p1 = 0x06,//320ms
    ODR_1p5 = 0x07,//640ms
    ODR_0p78 = 0x08,//1.280s
    ODR_0p39 = 0x09, //2.560ms
    ODR_0p2 = 0x0A, //5.120ms
    ODR_0p1 = 0x0B, //10.24ms
    ODR_0p05 = 0x0C, //20.48
    ODR_0p02 = 0x0D, //40.96
    ODR_0p01 = 0x0E, //81.92
    ODR_0p006 = 0x0F, //163.84s
    ODR_0p003 = 0x10, //327.68s
    ODR_0p0015 = 0x11//655.36s
}odr_t;

#define ODR_SEL ODR_1p5//subdivision factor for pressure and temperature measurements is 2^value. Allowed values are 0..17. Other values are saturated at 17.
//IIR filter coefficient
typedef enum {
    COEF_0 = 0b000,
    COEF_1 = 0b001,
    COEF_3 = 0b010,
    COEF_7 = 0b011,
    COEF_15 = 0b100,
    COEF_31 = 0b101,
    COEF_63 = 0b110,
    COEF_127 = 0b111
}config_t;
typedef enum {
    CmdNop = 0x00,
    FifoFlush = 0x0B,//Clears all data in the ifo, does not change FIFO_CONFIG register
    SoftReset = 0xB6//Triggers a reset, all user configuration settings are overwritten with their default state
}command_t;

typedef struct{
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double comp_press;
    double tlin;
}BMP390_calib_data_t;

#define BMP_TIMEOUT 5
#define BUFF_SIZE 30
#define BUFF_WHOLE_SIZE 0x20
#define SS_PIN BMP_CS_Pin
#define SS_PORT BMP_CS_GPIO_Port
#define CALIB_END 69
#define CALIB_START 49
#define CALIB_QTY 21
#define SETTINGS_START_ADDRESS 0x15
#define SETTINGS_STOP_ADDRESS 0x1F
#define SETTINGS_SIZE (SETTINGS_STOP_ADDRESS - SETTINGS_START_ADDRESS)

#define BMP3_OK                                 INT8_C(0)
#define BMP3_W_MIN_PRES                         INT8_C(5)
#define BMP3_W_MAX_PRES                         INT8_C(6)
#define BMP3_MIN_PRES_INT                       UINT64_C(3000000)
#define BMP3_MAX_PRES_INT                       UINT64_C(12500000)

//registers addresses
#define ADD_CHIP_ID 0x00
#define ADD_REV_ID 0x01
#define ADD_ERR_REG 0x02
#define ADD_STATUS 0x03

#define ADD_DATA_0 0x04
#define ADD_DATA_1 0x05
#define ADD_DATA_2 0x06
#define ADD_DATA_3 0x07
#define ADD_DATA_4 0x08
#define ADD_DATA_5 0x09
#define ADD_SENSORTIME_0 0x0C
#define ADD_SENSORTIME_1 0x0D
#define ADD_SENSORTIME_2 0x0E

#define ADD_EVENT 0x10
#define ADD_INT_STATUS 0x11
#define ADD_FIFO_LENGTH_0 0x12
#define ADD_FIFO_LENGTH_1 0x13
#define ADD_FIFO_DATA 0x14
#define ADD_FIFO_WTM_0 0x15
#define ADD_FIFO_WTM_1 0x16
#define ADD_FIFO_CONFIG_1 0x17
#define ADD_FIFO_CONFIG_2 0x18
#define ADD_INT_CTRL 0x19
#define ADD_IF_CONF 0x1A
#define ADD_PWR_CTRL 0x1B
#define ADD_OSR 0x1C
#define ADD_ODR 0x1D
#define ADD_CONFIG 0x1F
#define ADD_CMD 0x7E

#define ADD_PRESS_START ADD_DATA_0
#define ADD_TEMP_START ADD_DATA_3
#define ADD_PRESS_BYTE_QTY 3
#define ADD_TEMP_BYTE_QTY 3
//defaults
#define DEF_FIFO_MODE 0 //Enable FIFO mode
#define DEF_FIFO_STOP_OF 0 // Enable Stop writing fifo if full
#define DEF_FIFO_TIME_EN 0 //Return sensortime frame after the last valid data frame
#define DEF_FIFO_PRESS_EN 0 //Store pressure data in FIFO
#define DEF_FIFO_TEMP_EN 0 //Store temperature data in FIFO
#define DEF_FIFO_SUBSAMPLING 2 //FIFO downsampling selection for pressure and temperature data, factor is 2^fifo_subsampling
#define DEF_DATA_SELECT 0 //for pressure and temperature, select data source 0 - unfiltered data
#define DEF_INT_OD 0 // Configure output: 1-open-drain or 0-push-pull
#define DEF_INT_LVL 1 //Level of INT pin 0-active low, 1 - active high
#define DEF_INT_LATCH 0//Latching of interrupts for INT pin and INT_STATUS register
#define DEF_FWTM_EN 0 //enable FIFO watermark reached interrupt for INT pin and INT_STATUS
#define DEF_FULL_EN 0 //enable Fifo full interrupt for INT pin and INT_STATUS
#define DEF_INT_DS 0
#define DEF_DRDY_EN 1 //enable temperature / pressure data ready interrupt for INT pin and INT_STATUS
#define DEF_SPI3_M 0 //Configure SPI Interface Mode for primary interface 0 - SPI4/1 - SPI3
#define DEF_I2C_WDT_EN 0 // Enable for the I2C Watchdog timer, backed by NVM
#define DEF_I2C_WDT_S 1// 0 - wdt short after 1.25ms
#define DEF_PW_PRESS 1 //Pressure sensor enable
#define DEF_PW_TEMP 0 //Temperature sensor enable
#define DEF_PWR_MODE 0b11 //Normal mode
#define DEF_OSRP OSRx4 //Oversampling setting pressure measurement
#define DEF_OSRT OSRx1//Oversampling setting temperature measurement
#define DEF_ODR ODR_12p5 // Set the configuration of the output data rates ODR_50 = 20ms(50Hz)
#define DEF_CONFIG COEF_15 //IIR filter config
#define DEF_WTM 0
#define CMD_FIFO_FLUSH 0xB0
#define CMD_SOFTRESET 0xB6

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t ChipName[10];
    uint16_t CHIP_ID;
    uint16_t REV_ID;
    uint16_t ERR_REG;
    uint16_t DRDY_TEMP;
    uint16_t DRDY_PRESS;
    uint16_t CRDY;
    uint32_t PDATA;
    uint32_t TDATA;
    uint32_t SENSORTIME;
    uint16_t EVENT;
    uint16_t INT_STATUS;
    uint16_t FIFO_LENGTH;
    uint16_t FIFO_DATA;
    uint16_t FIFO_WTM;
    uint16_t FIFO_TEMP_EN;
    uint16_t FIFO_PRESS_EN;
    uint16_t FIFO_TIME_EN;
    uint16_t FIFO_STOP_OF;
    uint16_t FIFO_MODE;
    uint16_t DATA_SEL;
    uint16_t FIFO_SBSMP;
    uint16_t DRDY_EN;
    uint16_t INT_DS;
    uint16_t FULL_EN;
    uint16_t FWTM_EN;
    uint16_t INT_LATCH;
    uint16_t INT_LVL;
    uint16_t INT_OD;
    uint16_t I2C_WDT_S;
    uint16_t I2C_WDT_EN;
    uint16_t SPI3_M;
    uint16_t PW_MODE;
    uint16_t PW_TEMP;
    uint16_t PW_PRESS;
    osr_t OSRT;
    osr_t OSRP;
    odr_t ODR;
    config_t CONFIG;
    command_t CMD;
}bmp_t;

void bmpInit(bmp_t *bmp);
void readPressure(bmp_t *bmp);
uint32_t readTemperature(bmp_t *bmp);
uint8_t readStatus(bmp_t *bmp);
uint8_t readID(bmp_t *bmp);
void readAllRegs(bmp_t *bmp);
double CalculatePressure(uint32_t uncomp_press);
void softReset(bmp_t *bmp);
uint8_t readIntCtrl(bmp_t *bmp);
void readBmpConfig(bmp_t *bmp);
void setConfig(bmp_t *bmp);
void setConfigStream(bmp_t *bmp);
void bmpWriteByte(bmp_t *bmp, uint8_t regAddress, uint8_t sByte);
uint32_t bmpReadVal(bmp_t *bmp, uint8_t address);
uint8_t bmpReadByte(bmp_t *bmp, uint8_t regAddress);
int8_t compensate_pressure(bmp_t *bmp, uint64_t *pressure);
void readCalibration(bmp_t *bmp);
SPI_HandleTypeDef SpiInit(uint8_t num);



#endif /* BMP_H_ */
