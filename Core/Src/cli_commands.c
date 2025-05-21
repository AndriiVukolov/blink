/*
 * cli_commands.c
 *
 *  Created on: May 7, 2025
 *      Author: CS60
 */
#include <stdio.h>
#include "tim.h"
#include "microrl.h"
#include <stdlib.h>
#include <string.h>
#include "usart.h"
#include "adc.h"
#include "cli_commands.h"
#include "lux.h"
#include "bmp.h"


void print_help (void)
{
    print ("Use TAB key for completion\n\rCommand:\n\r");
    print ("clear               - clears screen\n\r");
    print ("ledon               - turns LED on\n\r");
    print ("ledoff              - turns LED off\n\r");
    print ("led_set <brightness> - sets LED brightness 0..100\n\r");
    print ("adc_get             - prints current ADC value\n\r");
    print ("adc_status          - shows current state of ADC\n\r");
    print ("opt_read            - reads current value of optic sensor (Lux)\n\r");
    print ("b_mp_read            - reads current value of pressure sensor (Pa)\n\r");
    print ("b_mp_read <-c>       - reads current value of pressure sensor <-c - cyclic> (Pa)\n\r");
    print ("bmp_id_read          - reads current value of sensor ID\n\r");
    print ("bmp_read_press_int   - reads current value of pressure sensor (int value)\n\r");
    print ("bmp_reset            - makes soft reset of pressure sensor (int value)\n\r");
    print ("bmp_read_all         - reads all registers of pressure sensor (int value)\n\r");
    print ("bmp_read_config      - reads configuration registers of pressure sensor (int value)\n\r");
    print ("bmp_read_int_ctrl    - reads interrupt configuration registers of pressure sensor (int value)\n\r");
    print ("bmp_set_config       - writes current configuration into pressure sensor (int value)\n\r");
    print ("bmp_write [add] [val] - writes value val into register in address add\n\r");
    print ("bmp_read [add]       - writes value of register in address add\n\r");
    print ("bmp_read_calibration - reads value of calibration registers\n\r");


#define _CMD_HELP   "help"
#define _CMD_CLEAR  "clear"
#define _CMD_LED_ON "ledon"
#define _CMD_LED_OFF "ledoff"
#define _CMD_SET_BRIGHTNESS "led_set"
#define _CMD_ADC_GET "adc_get"
#define _CMD_ADC_STATUS "adc_status"
#define _CMD_OPT_READ "opt_read"
#define _CMD_BMP_READ "b_mp_read"
#define _CMD_CHIP_ID "bmp_id_read"
#define _CMD_BMP_READ_PRESS_INT "bmp_read_press_int"
#define _CMD_BMP_RESET "bmp_reset"
#define _CMD_BMP_READ_ALL "bmp_read_all"
#define _CMD_BMP_READ_CONFIG "bmp_read_config"
#define _CMD_BMP_READ_INT_CTRL "bmp_read_int_ctrl"
#define _CMD_BMP_SET_CONFIG "bmp_set_config"
#define _CMD_BMP_WRITE_BYTE "bmp_write"
#define _CMD_BMP_READ_BYTE "bmp_read"
#define _CMD_BMP_READ_CALIBRATION "bmp_read_calibration"
}
void cmdHelp (void)
{
    print ("microrl v");
    print (MICRORL_LIB_VER);
    print("\n\r");
    print_help ();        // print help
}

void cmdClear(void)
{
    print ("\033[2J");    // ESC seq for clear entire screen
    print ("\033[H");     // ESC seq for move cursor at left-top corner
}
void cmdLedon(void)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    print("led turning on \n\r");
}
void cmdLedoff(void)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    print("led turning off \n\r");
}
void cmdSetBrightness(const char * val)
{
    char buffer[STRING_SIZE] = {0};
    uint32_t pulseWidth = 0;
    uint32_t brightness = 0;
    char * cptr = NULL;
    uint8_t len;
    len = strlen(val);

    if ((len <= 4) && (len > 1))
    {
        brightness = strtoul(val, &cptr, 10);
        pulseWidth = (htim3.Init.Period * brightness / 100);
        TIM3_PeriodSet(pulseWidth);
    }
    else if (len > 4)
    {
        print ("value too long!\n\r");
    }
    else
    {
        print ("Enter the value \n\r");
    }
    sprintf(buffer, "brightness is %lu", brightness);
    print(buffer);
    print("% \n\r");
}

void cmdADCGet(const char * val)
{
    uint32_t adcVal = 0;
    char buffer[STRING_SIZE] = {0};
    char gChar = 0;

    if (strcmp(val, _ARG_CYCLIC) == 0)
    {
        while(gChar != KEY_ETX)
        {
            adcVal = pollADC();
            sprintf(buffer, "ADC value: %lu \r", adcVal);
            print(buffer);
            gChar = get_char();
            if (gChar == KEY_ETX) print("\n\r");

        }
    }
    else if (strcmp(val, "-1") == 0)
    {
        adcVal = pollADC();
        sprintf(buffer, "ADC value: %lu \r\n", adcVal);
        print(buffer);
    }
    else
    {
        print ("value is incorrect\n\r");
    }
}
void cmdADCGetStatus(void)
{
    uint32_t adcVal = 0;
    char buffer[STRING_SIZE] = {0};

    adcVal = HAL_ADC_GetState(&hadc1);
    sprintf(buffer, "Status %lu \r\n", adcVal);
    print(buffer);
}
void cmdOptRead(lux_t * lux, const char * val)
{
    char buffer[STRING_SIZE] = {0};
    char gChar = 0;

    if (strcmp(val, _ARG_CYCLIC) == 0)
    {
        while(gChar != KEY_ETX)
        {
            readLux(lux);
            readConfig(lux);
            sprintf(buffer, "Illumination level: %f \r", lux->VAL);
            print(buffer);
            memset(buffer, 0, 40);
            gChar = get_char();
            if (gChar == KEY_ETX) print("\n\r");
        }
    }
    else if (strcmp(val, "0") == 0)
    {
        readLux(lux);
        sprintf(buffer, "Illumination level: %f \r\n", lux->VAL);
        print(buffer);
    }
    else
    {
        print ("value is incorrect\n\r");
    }
}
void cmdBmpReadPress(bmp_t * bmp, const char * val)
{
    char buffer1[STRING_SIZE] = {0};
    char gChar = 0;
    static uint32_t prevVal = 0;
    uint64_t outValInt = 0;
    double outVal = 0;

    if (strcmp(val, _ARG_CYCLIC) == 0)
    {
        while(gChar != KEY_ETX)
        {
            if (prevVal != bmp->PDATA)
            {
                prevVal = bmp->PDATA;
                compensate_pressure(bmp, &outValInt);
                outVal = (double)outValInt / 100;
                sprintf(buffer1, "Pressure (float): %f \r", outVal);
                print(buffer1);
            }
            memset(buffer1, 0, STRING_SIZE);
            gChar = get_char();
            if (gChar == KEY_ETX) print("\n\r");
        }
    }
    else if (strcmp(val, "0") == 0)
    {
        compensate_pressure(bmp, &outValInt);
        outVal = outValInt / 100;
        sprintf(buffer1, "Pressure level: %f \n\r", outVal);
        print(buffer1);
        memset(buffer1, 0, STRING_SIZE);
    }
    else
    {
        print ("value is incorrect\n\r");
    }

}
void cmdBmpReadPressInt(bmp_t *bmp)
{
    char buffer[STRING_SIZE] = {0};

    sprintf(buffer, "Pressure level (int): %lu \r\n", bmp->PDATA);
    print(buffer);
    memset(buffer, 0, STRING_SIZE);
}
void cmdChipIdRead(bmp_t* bmp)
{
    uint8_t idVal = 0;
    char buffer[STRING_SIZE] = {0};

    idVal = readID(bmp);
    sprintf(buffer, "ChipID:  %X \r\n", (uint16_t)idVal);
    print(buffer);
}
void cmdReset(bmp_t* bmp)
{
    char buffer[STRING_SIZE] = {0};

    softReset(bmp);
    sprintf(buffer, "Chip:  %s reseted \r\n", bmp->ChipName);
    print(buffer);
}
void cmdReadAll(bmp_t* bmp)
{
    readAllRegs(bmp);
}
void cmdReadConfig(bmp_t* bmp)
{
    char buffer[STRING_SIZE] = {0};

    readBmpConfig(bmp);
    sprintf(buffer, "Chip %s config byte: %x \r\n", bmp->ChipName, bmp->CONFIG);
    print(buffer);
}
void cmdReadIntCtrl(bmp_t* bmp)
{
    char buffer[STRING_SIZE] = {0};
    uint8_t ictrl = 0;

    ictrl = readIntCtrl(bmp);
    sprintf(buffer, "Chip %s interrupt config byte: %x \r\n", bmp->ChipName, ictrl);
    print(buffer);
}
void cmdSetConfig(bmp_t * bmp)
{
    char buffer[STRING_SIZE] = {0};

    setConfig(bmp);
    sprintf(buffer, "Chip %s - settings written \r\n", bmp->ChipName);
    print(buffer);
}
void cmdWriteByte(bmp_t * bmp, const char * add, const char * bt)
{
    char buffer[STRING_SIZE] = {0};
    uint8_t address;
    uint8_t dbyte;
    address = (uint8_t)strtol(add,NULL,16);
    dbyte = (uint8_t)strtol(bt,NULL,16);
    bmpWriteByte(bmp, address, dbyte);

    sprintf (buffer, "Register %X written with value %X \r\n", address, dbyte);
    print(buffer);

}
void cmdReadByte(bmp_t * bmp, const char * addS)
{
    uint8_t bt = 0;
    char buffer[STRING_SIZE] = {0};
    uint8_t add;

    add = strtol(addS,NULL,16);
    bt = bmpReadByte(bmp,add);
    sprintf (buffer, "Register: %X  -  Value: %X \r\n", add, bt);
    print(buffer);
}
void cmdReadCalibration(bmp_t * bmp)
{
    readCalibration(bmp);
}
