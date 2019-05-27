#include "guncon2.h"

struct GUNCON2 gcon;

void gcon_prepare_data()
{
gcon.buffer[0]=gcon.btn&0xFF;
gcon.buffer[1]=gcon.btn/256;
gcon.buffer[2]=gcon.x&0xFF;
gcon.buffer[3]=gcon.x/256;
gcon.buffer[4]=gcon.y&0xFF;
gcon.buffer[5]=gcon.y/256;
gcon.buffer[6]=0;
gcon.buffer[7]=0;
}
void gcon_save_data(uint8_t *data)
{

	gcon.x=((((uint16_t) *(data+3))<<8)|(*(data+2)))-gcon.offset_x;
	gcon.y=((((uint16_t) *(data+5))<<8)|(*(data+4)))-gcon.offset_y;
	gcon.btn=((((uint16_t) *(data+1))<<8)|(*(data+0)));
	if(gcon.progressive){gcon.btn|=0x01;}
	gcon_prepare_data();
	if((gcon_get_BTN(BUTTON_TRIGGER)&gcon_get_BTN(BUTTON_SELECT))&&(gcon.calibration==0))
	{gcon.calibration=1;
	gcon.btn|=BUTTON_SELECT;
	gcon.buffer[1]|=0x40;
	}
	else
	{gcon.calibration=0;}

}

void set_progressive()
{gcon.progressive=1;}

void set_offset_x(int16_t x)
{gcon.offset_x=x;}

void set_offset_y(int16_t y)
{gcon.offset_y=y;}

uint8_t gcon_get_BTN(uint16_t button)
{
uint8_t temp;
temp=((gcon.btn&button)?0:1);
return(temp);
}


