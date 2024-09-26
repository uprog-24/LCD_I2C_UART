#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdint.h>

void lcd_init(void);
void lcd_send_data(char data);
void lcd_send_cmd(char cmd);
void get_device_addr(uint8_t *addr);

#endif /* INC_LCD_H_ */
