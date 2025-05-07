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
#include "usart.h"



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
    static char buffer[5] = {0};
    static uint32_t pulseWidth = 0;
    static uint32_t brightness = 0;
    static char * cptr = NULL;
    static uint8_t len;
    len = strlen(val);

    if ((len <= 4) && (len >0))
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
    print("brightness is ");
    sprintf(buffer, "%lu", brightness);
    print(buffer);
    print("% \n\r");
}
