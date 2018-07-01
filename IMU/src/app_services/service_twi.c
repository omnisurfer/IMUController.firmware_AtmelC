/*
 * service_twi.c
 *
 * Created: 1/1/2015 7:13:10 PM
 *  Author: Daniel
 */ 

#include <asf.h>
#include "conf_twim.h"
#include "../app_services/service_twi.h"

TWI_MASTER_t master_handle;

twi_options_t master_options =
{	
	.speed = TWI_SPEED,
	.chip = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(32000000, TWI_SPEED)
};

void service_twi_initialize(void)
{
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDOR_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDOR_gc;
		
	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &master_options);
	twi_master_enable(&TWI_MASTER);
}

void service_twi_read_register(int device_address, Byte* register_address, int register_address_length, Byte* data_receive_buffer, int data_receive_buffer_length)
{
	//I can't use sizeof on a pointer as it does not know what it is pointing at. Need to store the size somehow or pass it separately
	//http://stackoverflow.com/questions/492384/how-to-find-the-sizeofa-pointer-pointing-to-an-array
	/*
	int register_address_length = sizeof(*register_address)/sizeof(register_address[0]);
	int data_receive_buffer_length = sizeof(*data_receive_buffer)/sizeof(data_receive_buffer[0]);
	*/
	
	Byte _register_address[register_address_length];
	
	for(int i = 0; i < register_address_length; i++)
		_register_address[i] = *(register_address + i);
								
	twi_package_t packet = {
		.addr_length	= register_address_length,
		.chip			= device_address,
		.addr[0]		= _register_address[0],
		.addr[1]		= _register_address[1],
		.addr[2]		= _register_address[2],
		.buffer			= data_receive_buffer,
		.length			= data_receive_buffer_length,
		.no_wait		= false
	};		
				
	twi_master_read(&TWI_MASTER, &packet);	
}

void service_twi_write_register(int device_address, Byte* register_address, int register_address_length, Byte* data_send_buffer, int data_send_buffer_length)
{
	Byte _register_address[register_address_length];
	
	for(int i = 0; i < register_address_length; i++)
		_register_address[i] = *(register_address + i);
								
	twi_package_t packet = {
		.addr_length	= register_address_length,
		.chip			= device_address,
		.addr[0]		= _register_address[0],
		.addr[1]		= _register_address[1],
		.addr[2]		= _register_address[2],
		.buffer			= data_send_buffer,
		.length			= data_send_buffer_length,
		.no_wait		= false
	};		
				
	twi_master_write(&TWI_MASTER, &packet);
}

