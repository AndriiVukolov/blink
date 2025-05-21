/*
 * cli_commands.h
 *
 *  Created on: May 7, 2025
 *      Author: CS60
 */

#ifndef INC_CLI_COMMANDS_H_
#define INC_CLI_COMMANDS_H_

#include "lux.h"
#include "bmp.h"
// definition commands word
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

//keys
#define _ARG_CYCLIC "-c"

#define STRING_SIZE 50

//available  commands
//char * keyword [] = {_CMD_HELP, _CMD_CLEAR, _CMD_LED_ON, _CMD_LED_OFF, _CMD_SET_BRIGHTNESS};
void cmdHelp (void);
void cmdClear(void);
void cmdLedon(void);
void cmdLedoff(void);
void cmdSetBrightness(const char * val);
void cmdADCGet();
void print_help (void);
void cmdADCGet(const char * val);
void cmdADCGetStatus(void);
void cmdOptRead(lux_t * lux, const char * val);
void cmdBmpReadPress(bmp_t * bmp, const char * val);
void cmdBmpReadPressInt(bmp_t *bmp);
void cmdChipIdRead(bmp_t* bmp);
void cmdReset(bmp_t* bmp);
void cmdReadAll(bmp_t* bmp);
void cmdReadConfig(bmp_t* bmp);
void cmdReadIntCtrl(bmp_t* bmp);
void cmdWriteByte(bmp_t * bmp, const char * add, const char * bt);
void cmdReadByte(bmp_t * bmp, const char * addS);
void cmdSetConfig(bmp_t * bmp);
void cmdReadCalibration(bmp_t * bmp);


#endif /* INC_CLI_COMMANDS_H_ */
