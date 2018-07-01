/*
 * app_usb.h
 *
 * Created: 12/30/2014 4:47:00 PM
 *  Author: Daniel
 */ 


#ifndef APP_USB_H_
#define APP_USB_H_

bool callback_usb_cdc_enable(void);

void callback_usb_cdc_disable(void);

void app_usb_cdc(Byte);

void app_usb_cdc_write_char_string(char* string, int length);

#endif /* APP_USB_H_ */