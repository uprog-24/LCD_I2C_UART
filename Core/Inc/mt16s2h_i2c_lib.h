/**
 * \file mt16s2h_i2c_lib.h
 */
#ifndef INC_MT16S2H_I2C_LIB_H_
#define INC_MT16S2H_I2C_LIB_H_

#include "main.h"
#include <stdbool.h>

#define DEVICE_ADDR 0x38 << 1

typedef enum DISPLAY_CONTROL_MODE {
	DISPLAY_ON, DISPLAY_OFF, BLINK_ON, BLINK_OFF, CURSOR_ON, CURSOR_OFF
} display_control_mode_t;

void MT16S2H_Init();
void MT16S2H_Clear();
void MT16S2H_Home();
void MT16S2H_SetDisplayControlMode(display_control_mode_t mode);

void MT16S2H_SetCursor(uint8_t col, uint8_t row);
void MT16S2H_PrintStr(char *str);

#endif /* INC_MT16S2H_I2C_LIB_H_ */
