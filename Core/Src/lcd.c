#include "lcd.h"
#include "i2c.h"

#define SLAVE_ADDRESS_LCD 0xA0
#define Adress 0x38 << 1

//extern uint8_t addr = 0;

void lcd_init(void) {
	// 8 bit
	/*
	 HAL_Delay(30);
	 lcd_send_cmd(0b00110000);
	 HAL_Delay(1);
	 lcd_send_cmd(0b00110000);
	 HAL_Delay(1);
	 lcd_send_cmd(0b00110000);
	 HAL_Delay(1);

	 lcd_send_cmd(0b00111000); // function set: DL=1 (8 bit), P = 0
	 HAL_Delay(1);
	 lcd_send_cmd(0b00000001);   // clear display
	 HAL_Delay(1);
	 lcd_send_cmd(0b00001110); // display on: D=1, C=1, B=1
	 */

	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd(0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd(0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);  // 4bit mode
	HAL_Delay(10);

//	lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	lcd_send_cmd(0b00101000);
	HAL_Delay(1);
	lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd(0x01);  // clear display
	HAL_Delay(2);
	lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd(0x0C);

}

void lcd_send_data(char data) {

	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t*) data, 4, 100);
}

void lcd_send_cmd(char cmd) {

//	uint16_t command_word = 0;
//	command_word = cmd;  // rs=0
//	uint8_t cmd_t[2];
//	cmd_t[0] = (command_word >> 8) & 0xFF;
//	cmd_t[1] = (command_word) & 0xFF;
//	HAL_I2C_Master_Transmit(&hi2c2, addr, (uint8_t*) cmd_t, 2, 100);

	char data_u, data_l;
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	uint8_t data_t[4];
	data_t[0] = data_u | 0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[1] = data_u | 0x08;  //en=0, rs=0 -> bxxxx1000
	data_t[2] = data_l | 0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[3] = data_l | 0x08;  //en=0, rs=0 -> bxxxx1000
	HAL_I2C_Master_Transmit(&hi2c2, Adress, (uint8_t*) data_t, 4, 100);

//	HAL_I2C_Master_Transmit(&hi2c2, addr, (uint8_t*) &cmd, 8, 100);
}

//void get_device_addr(uint8_t *addr) {
//	HAL_StatusTypeDef status;
//	for (int i = 1; i < 128; i++) {
//		status = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (i << 1), 4, 100);
//		if (status == HAL_OK) {
//			*addr = i;
//		}
//	}
//}

