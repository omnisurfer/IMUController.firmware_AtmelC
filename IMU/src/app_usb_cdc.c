/*
 * app_usb.c
 *
 * Created: 12/30/2014 4:46:00 PM
 *  Author: Daniel
 */ 
#include <asf.h>
#include "conf_usb.h"
#include "app_usb_cdc.h"

//look into Interface for application with multi CDC interfaces support for more than one CDC port
static bool app_usb_cdc_open = false;

bool callback_usb_cdc_enable(void)
{
	app_usb_cdc_open = true;
	return true;
}
void callback_usb_cdc_disable(void)
{
	app_usb_cdc_open = false;
}

void app_usb_cdc(Byte data)
{
	if (app_usb_cdc_open) {
		//int temp = udi_cdc_getc();
		udi_cdc_putc(data);
						
		//udi_cdc_putc('A');
	}
}

//this does not check boundries so can be very dangerous!
void app_usb_cdc_write_char_string(char* string, int length)
{	
	for(uint_fast8_t i = 0; i < length; i++)
	{
		if(*(string + i) == '\0')
			break;
			
		else
			udi_cdc_putc(*(string + i));
	}	
}