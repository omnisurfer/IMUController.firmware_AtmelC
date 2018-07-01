/*
 * service_twi.h
 *
 * Created: 1/1/2015 7:13:25 PM
 *  Author: Daniel
 *
 * NOTE!!! May want to consider rolling own two wire driver for the controller:
 * The ASF version blocks and for me sometimes just hangs waiting for the bus to be available even though nothing on my end has even started to use the I2C bus.
 * http://asf.atmel.com/bugzilla/show_bug.cgi?id=3248
 * http://www.robotroom.com/Atmel-AVR-TWI-I2C-Multi-Master-Problem.html
 */ 


#ifndef SERVICE_TWI_H_
#define SERVICE_TWI_H_

void service_twi_initialize(void);

void service_twi_read_register(int device_address, Byte* register_address, int register_address_length, Byte* data_receive_buffer, int data_receive_buffer_lenght);

void service_twi_write_register(int device_address, Byte* register_address, int register_address_length, Byte* data_send_buffer, int data_send_buffer_lenght);

#endif /* SERVICE_TWI_H_ */