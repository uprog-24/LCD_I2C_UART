/**
 * @file mt16s2h_i2c_lib.c
 */
#include "mt16s2h_i2c_lib.h"

/* Command */
#define LCD_CLEAR_DISPLAY   0x01  ///< Command to clear display.
#define LCD_RETURN_HOME     0x02  ///< Command to return cursor to (0,0).

/* Entry Mode Set */
#define LCD_ENTRY_MODE_SET 0x04  ///< Mask of command to set entry mode.
#define LCD_ENTRY_RIGHT 0x02  ///< Command to shift cursor to right.
#define LCD_ENTRY_LEFT 0x00  ///< Command to shift cursor to left.

/* Display On/Off Control */
#define LCD_DISPLAY_CONTROL 0x08  ///< Mask of command to display control.
#define LCD_DISPLAY_ON      0x04  ///< Command to display on.
#define LCD_DISPLAY_OFF     0x00  ///< Command to display off.
#define LCD_CURSOR_ON       0x02  ///< Command to cursor on.
#define LCD_CURSOR_OFF      0x00  ///< Command to cursor off.
#define LCD_BLINK_ON        0x01  ///< Command to blink on.
#define LCD_BLINK_OFF       0x00  ///< Command to blink off.

/* Cursor Shift */
#define LCD_CURSOR_SHIFT 0x10  ///< Mask of command to set cursor shift.
#define LCD_DISPLAY_MOVE 0x08  ///< Command to enable shift of display.
#define LCD_CURSOR_MOVE  0x00  ///< Command to enable shift of cursor.
#define LCD_MOVE_RIGHT   0x04  ///< Command to shift Display/Cursor right.
#define LCD_MOVE_LEFT    0x00  ///< Command to shift Display/Cursor left.

/* Function Set */
#define LCD_FUNCTION_SET 0x28  ///< Mask of command to function set.
#define LCD_8BIT_MODE    0x10  ///< Command to set 8bit mode.
#define LCD_4BIT_MODE    0x00  ///< Command to set 4bit mode.

#define LCD_SET_CGRAM_ADDR  0x40  ///< Mask of command to set CGRAM address.
#define LCD_SET_DDRAM_ADDR  0x80  ///< Mask of command to set DDRAM address.

/* Enable Bit, Data strob */
#define ENABLE        0x04               ///< Enable bit, data strob for low nibble (3-0 configure bits): LED_E_RW_RS (0100).
#define E_SET(byte)   (byte |= ENABLE)   ///< E=1, byte = xxxx_x1xx.
#define E_RESET(byte) (byte &= ~ENABLE)  ///< E=0, byte = xxxx_x0xx.

/* Read Write Bit */
#define RW 0x0  ///< Write data mode.

/* Register Select Bit */
#define RS1_DATA    0x01  ///< Register Select bit for send Data (symbol) (RS=1).
#define RS0_COMMAND 0x00  ///< Register Select bit for send Command from datasheet (RS=0).

/* Mask */
#define MASK_HIGH_4BIT 0xF0  ///< Mask to get 7-4 bits.

/* Device Address */
#define MIN_DEVICE_ADDR 1    ///< Min device address in 7 bit mode
#define MAX_DEVICE_ADDR 128  ///< Max device address in 7 bit mode

#define TRANSMIT_I2C_TIMEOUT_MS 200U  ///< 200 ms limit to complete I2C transmit data

typedef enum {
	STATUS_OK, STATUS_FAIL
} statuses_t;

/// Structure I2C
extern I2C_HandleTypeDef hi2c2;

/// Device I2C Address
static uint8_t device_addr = 0;

/// Time Counter for I2C transmit data
static uint16_t timeout_counter = 0;

/// Display Mode byte to Entry Mode Set commands
static uint8_t dpMode;
/// Display Control byte to Display ON/OFF Control commands
static uint8_t dpControl;
/// Cursor Display Shift byte to Cursor Display Shift commands
static uint8_t cdShift;
/// Display Function byte to Function Set commands
static uint8_t dpFunction;

/// Byte to set numbers of rows
static uint8_t dpRows;
/// Byte to set Backlight to display
static uint8_t dpBacklight;

/// Flag to control I2C_MasterTxCplt
volatile bool is_i2c_master_tx_completed = false;
/**
 * Handle Interrupt by I2C_MasterTxCplt.
 * @param[in] hi2c pointer to structure I2C2.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C2) {
		is_i2c_master_tx_completed = true;
	}
}

/**
 * Set Device Address on line I2C.
 * @param[in] device_addr.
 */
static void SetDeviceAddr(uint8_t *device_addr) {
	HAL_StatusTypeDef status;
	for (int addr = MIN_DEVICE_ADDR; addr < MAX_DEVICE_ADDR; addr++) {
		status = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (addr << 1), 4, 100);
		if (status == HAL_OK) {
			*device_addr = (uint16_t) (addr << 1);
		}
	}
}

/**
 * Wait Transmit Completed status.
 * @param[out] statuses_t STATUS_OK: data successfully transmitted, STATUS_FAIL: transmit data failed.
 */
static statuses_t I2C_WaitTxCompletedStatus() {
	timeout_counter = 0;
	while (!is_i2c_master_tx_completed) {
		HAL_Delay(1);
		timeout_counter++;
		if (timeout_counter == TRANSMIT_I2C_TIMEOUT_MS) {
			return STATUS_FAIL;
		}
	}

	return STATUS_OK;
}

/**
 * Send 1 Byte to LCD with device_addr on I2C.
 * Write cycle to LCD: set E=1 and send 1 bite, set E=0 and send 1 bite again.
 * @param[in] byte high nibble (7-4 data bits): DB7_DB6_DB5_DB4, low nibble (3-0 configure bits): LED_E_RW_RS.
 * @param[out] statuses_t STATUS_OK: data successfully transmitted, STATUS_FAIL: transmit data failed.
 */
static statuses_t I2C_WriteByte(uint8_t byte) {

	E_SET(byte);
	is_i2c_master_tx_completed = false;
	HAL_I2C_Master_Transmit_IT(&hi2c2, DEVICE_ADDR, &byte, 1);
	if (I2C_WaitTxCompletedStatus() == STATUS_FAIL) {
		return STATUS_FAIL;
	}

	timeout_counter = 0;
	E_RESET(byte);
	is_i2c_master_tx_completed = false;
	HAL_I2C_Master_Transmit_IT(&hi2c2, DEVICE_ADDR, &byte, 1);
	if (I2C_WaitTxCompletedStatus() == STATUS_FAIL) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * Send 1 Byte in 4bit mode.
 * Usage 2 cycles of WriteByte: first - for high nibble, second - for low nibble.
 * @param[in] byte high nibble (7-4 data bits): DB7_DB6_DB5_DB4, low nibble (3-0 configure bits): LED_E_RW_RS.
 * @param[in] mode RS line. Send command: mode=0, send data (char): mode=1.
 * @param[out] statuses_t STATUS_OK: data successfully transmitted, STATUS_FAIL: transmit data failed.
 */
static statuses_t SendByte(uint8_t byte, uint8_t mode) {
	uint8_t high_nib = byte & MASK_HIGH_4BIT;
	uint8_t low_nib = (byte << 4) & MASK_HIGH_4BIT;
	if (I2C_WriteByte((high_nib) | mode) == STATUS_FAIL) {
		return STATUS_FAIL;
	}

	if (I2C_WriteByte((low_nib) | mode) == STATUS_FAIL) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * Send Command to LCD.
 * Usage SendByte function with mode=0.
 * @param[in] cmd 1 byte command from Datasheet on LCD.
 */
static void SendCommand(uint8_t cmd) {
	SendByte(cmd, RS0_COMMAND);
}

/**
 * Send Char to LCD.
 * Usage SendByte function with mode=1.
 * @param[in] ch 1 byte symbol.
 */
static void SendChar(uint8_t ch) {
	SendByte(ch, RS1_DATA);
}

/**
 * Initialization of LCD.
 * Send 3 bytes with 8BIT_MODE, 1 byte with 4BIT_MODE,
 * send other commands of LCD's settings.
 */
void MT16S2H_Init() {

	/*========Power on========*/
	/*========Wait for more than 20 ms after Vcc ========*/
	HAL_Delay(20);
	/*========Function set ( Interface is 8 bits long.)========*/
	I2C_WriteByte(LCD_FUNCTION_SET | LCD_8BIT_MODE);
	/*========Wait for more 40 us========*/
	HAL_Delay(1);
	/*========Function set ( Interface is 8 bits long.)========*/
	I2C_WriteByte(LCD_FUNCTION_SET | LCD_8BIT_MODE);
	HAL_Delay(1);
	/*========Function set ( Interface is 8 bits long.)========*/
	I2C_WriteByte(LCD_FUNCTION_SET | LCD_8BIT_MODE);
	HAL_Delay(1);

	/*========Function set ( Interface is 4 bits long.)========*/
	I2C_WriteByte(LCD_FUNCTION_SET | LCD_4BIT_MODE);
	HAL_Delay(1);

	/* Display Control */
	MT16S2H_SetDisplayControlMode(DISPLAY_OFF);
	MT16S2H_SetDisplayControlMode(CURSOR_OFF);
	MT16S2H_SetDisplayControlMode(BLINK_OFF);

	MT16S2H_Clear();
	MT16S2H_SetDisplayControlMode(DISPLAY_ON);

}

/**
 * Set Display Control Mode using commands from datasheet.
 * mode = DISPLAY_ON: Display turn on.
 * mode = DISPLAY_OFF: Display turn off.
 * mode = BLINK_ON: Turn on blinking of symbol.
 * mode = BLINK_OFF: Turn off blinking of symbol.
 * mode = CURSOR_ON: Cursor turn on (underline).
 * mode = CURSOR_OFF: Cursor turn off.
 * @param[in] mode mode from enum display_control_mode_t.
 */
void MT16S2H_SetDisplayControlMode(display_control_mode_t mode) {
	switch (mode) {
	case DISPLAY_ON:
		dpControl |= LCD_DISPLAY_ON;
		break;
	case DISPLAY_OFF:
		dpControl &= ~LCD_DISPLAY_ON;
		break;
	case BLINK_ON:
		dpControl |= LCD_BLINK_ON;
		break;
	case BLINK_OFF:
		dpControl &= ~LCD_BLINK_ON;
		break;
	case CURSOR_ON:
		dpControl |= LCD_CURSOR_ON;
		break;
	case CURSOR_OFF:
		dpControl &= ~LCD_CURSOR_ON;
		break;
	}
	SendCommand(LCD_DISPLAY_CONTROL | dpControl);
}

/**
 * Clear text on LCD.
 */
void MT16S2H_Clear() {
	SendCommand(LCD_CLEAR_DISPLAY);
	HAL_Delay(1);
}

/**
 * Set Cursor to start (col=0, row=0).
 */
void MT16S2H_Home() {
	SendCommand(LCD_RETURN_HOME);
}

/**
 * Set a position of the cursor. LCD 16*2.
 * @param[in] col index of column from 0 to 15.
 * @param[in] row index of row from 0 to 1.
 */
void MT16S2H_SetCursor(uint8_t col, uint8_t row) {
	int row_offsets[] = { 0x00, 0x40 };
	SendCommand(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

/**
 * Print text to LCD.
 * @param[in] str text for printing.
 */
void MT16S2H_PrintStr(char *str) {
	while (*str != '\0') {
		SendChar(*str++);
	}
}
