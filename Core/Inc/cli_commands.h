/*
 * cli_commands.h
 *
 *  Created on: May 7, 2025
 *      Author: CS60
 */

#ifndef INC_CLI_COMMANDS_H_
#define INC_CLI_COMMANDS_H_

#include "lux.h"
// definition commands word
#define _CMD_HELP   "help"
#define _CMD_CLEAR  "clear"
#define _CMD_LED_ON "ledon"
#define _CMD_LED_OFF "ledoff"
#define _CMD_SET_BRIGHTNESS "led_set"
#define _CMD_ADCGET "adc_get"
#define _CMD_ADCSTATUS "adc_status"
#define _CMD_OPTREAD "opt_read"

//keys
#define _ARG_CYCLIC "-c"

//available  commands
//char * keyword [] = {_CMD_HELP, _CMD_CLEAR, _CMD_LED_ON, _CMD_LED_OFF, _CMD_SET_BRIGHTNESS};
void cmdHelp (void);
void cmdClear(void);
void cmdLedon(void);
void cmdLedoff(void);
void cmdSetBrightness(char * val);
void cmdADCGet();
void print_help (void);
void cmdADCGet(char * val);
void cmdADCGetStatus(void);
void cmdOptRead(lux_t * lux, char * val);


#endif /* INC_CLI_COMMANDS_H_ */
