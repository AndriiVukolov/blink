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


void print_help (void)
{
    print ("Use TAB key for completion\n\rCommand:\n\r");
    print ("clear               - clear screen\n\r");
    print ("ledon               - turns LED on\n\r");
    print ("ledoff              - turns LED off\n\r");
    print ("led_set <brightness> - sets LED brightness 0..100\n\r");
    print ("adc_get             - print current ADC value\n\r");
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
void cmdSetBrightness(char * val)
{
    static char buffer[20] = {0};
    static uint32_t pulseWidth = 0;
    static uint32_t brightness = 0;
    static char * cptr = NULL;
    static uint8_t len;
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

void cmdADCGet(char * val)
{
    static uint32_t adcVal = 0;
    static char buffer[30] = {0};
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
    static uint32_t adcVal = 0;
    static char buffer[30] = {0};

    adcVal = HAL_ADC_GetState(&hadc1);
    sprintf(buffer, "Status %lu \r\n", adcVal);
    print(buffer);
}
void cmdOptRead(char * val)
{
    char buffer[40] = {0};
    char gChar = 0;

    if (strcmp(val, _ARG_CYCLIC) == 0)
    {
        while(gChar != KEY_ETX)
        {
            readLux(&lux1);
            sprintf(buffer, "Illumination level: %f\r", lux1.VAL);
            print(buffer);
            memset(buffer, 0, 40);
            gChar = get_char();
            if (gChar == KEY_ETX) print("\n\r");

        }
    }
    else if (strcmp(val, "0") == 0)
    {
        readLux(&lux1);
        sprintf(buffer, "Illumination level: %f \r\n", lux1.VAL);
        print(buffer);
    }
    else
    {
        print ("value is incorrect\n\r");
    }
}
