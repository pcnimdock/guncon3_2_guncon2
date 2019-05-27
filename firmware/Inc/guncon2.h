
#ifndef _GUNCON2_H
#define _GUNCON2_H

#include "stdint.h"
#define BUTTON_TRIGGER			0x2000
#define BUTTON_A				0x0008
#define BUTTON_B				0x0004
#define BUTTON_C				0x0002
#define BUTTON_SELECT			0x4000
#define BUTTON_START			0x8000
#define DPAD_UP      			0x0010
#define DPAD_DOWN				0x0040
#define DPAD_LEFT 				0x0080
#define DPAD_RIGHT 				0x0020

#define GCON_BUFF_SIZE 8
struct GUNCON2
{
uint16_t x;
uint16_t y;
uint16_t btn;
uint8_t progressive;
int16_t offset_x;
int16_t offset_y;
uint8_t buffer[GCON_BUFF_SIZE];
uint8_t calibration;
};

void gcon_save_data(uint8_t *data);
void set_progressive();
void set_offset_x(int16_t x);
void set_offset_y(int16_t y);
uint8_t gcon_get_BTN(uint16_t button);


#endif
