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
#include "cli_commands.h"


void cmdHelp (void)
{
    print ("\rmicrorl v");
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
    print("\rled turning on \n\r");
}
void cmdLedoff(void)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    print("\rled turning off \n\r");
}
void cmdSetBrightness(const char * val)
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
        print ("\rvalue too long!\n\r");
    }
    else if (len == 0)
    {
        print ("\rEnter the value \n\r");
    }
    print("\rbrightness is ");
    sprintf(buffer, "%lu", brightness);
    print(buffer);
    print("% \n\r");
}
void print_help (void)
{
    print ("Use TAB key for completion.\n\rCommands:\n\r");
    print ("clear               - clear screen\n\r");
    print ("ledon               - turns LED on\n\r");
    print ("ledoff              - turns LED off\n\r");
    print ("led_set <brightness> - sets LED brightness 0..100\n\r");
}
