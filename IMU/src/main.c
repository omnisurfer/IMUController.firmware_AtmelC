/**
 * \file
 *
 * \brief Main functions for the IMU unit based on Adafruit 10-DOF (PID 1604) breakout board
 *	and XMEGA-C3 Xplained (ATxmega384C3) evaluation board
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include "conf_clock.h"
#include "conf_usb.h"
#include "conf_twim.h"

#include "../app_services/service_twi.h"
#include "../app_components/sensors/bmp180/bmp180.h"
#include "app_usb_cdc.h"

//could probably combine these into one function that selects between the the type of reading
int app_sensors_sample_temperature(float *result);
int app_sensors_sample_pressure(float *result);

//global debug float

int main (void)
{
	irq_initialize_vectors();
	cpu_irq_enable();
	sysclk_init();	
	board_init();
	
	udc_start();
	
	//twi service setup
	service_twi_initialize();		
							
	// Insert application code here, after the board has been initialized.
	
	//initialize bmp180 pressure sensor
	//bmp180_init();
					
	while(true)
	{						
		uint8_t data;
		
		/*
		//Test code for receiving serial data from CDC

		if(udi_cdc_is_rx_ready())
		{
			data = udi_cdc_getc();
			udi_cdc_putc(data);
		}
		*/
									
		float result = 0.00f;	
														
		if(app_sensors_sample_temperature(&result))
		{	
			char temp_text[] = "Temp (C): ";
			
			app_usb_cdc_write_char_string(temp_text, sizeof(temp_text));
			
			//max digits length DOES NOT include the 2 additional spaces for the newline and carriage return; these are added after the pressure is read.
			//it as space added to make the readings look nice on a single line.
			int max_digits_length = 12;
			char result_buffer[max_digits_length];
															
			snprintf(result_buffer, max_digits_length, "%3.2f ", result);
						
			app_usb_cdc_write_char_string(result_buffer, sizeof(result_buffer));					
		}
		
		if(app_sensors_sample_pressure(&result))
		{
			char press_text[] = "Press (Pa): ";
			
			app_usb_cdc_write_char_string(press_text, sizeof(press_text));
			
			//to print floats need to set CFLAGS (see threads below). The ATxmega384C MCU emulates floating point math in software unfortunately :(
			//http://www.nerdkits.com/forum/thread/1701/
			//http://www.pocketmagic.net/avr-studio-5-snprintf-vsnprintf-problem-with-floats/
			//http://winavr.scienceprog.com/avr-gcc-tutorial/using-sprintf-function-for-float-numbers-in-avr-gcc.html
			//max digits length also includes 2 additional spaces for the newline and carriage return							
			int max_digits_length = 13;
			char result_buffer[max_digits_length];
			
			snprintf(result_buffer, max_digits_length, "%3.2f\r\n", result);
			
			app_usb_cdc_write_char_string(result_buffer, sizeof(result_buffer));
		}								
	}
}

//these functions will likely be placed in their own C file but for debugging they are in main.
int app_sensors_sample_temperature(float *result)
{		
	int _status = bmp180_measurement_status();
	
	switch(_status)
	{
		case SENSOR_IDLE:						
		case TEMPERATURE_PENDING:
		case TEMPERATURE_BUSY:
			bmp180_measure_temperature();			
		break;
		
		case TEMPERATURE_DONE:
		{			
			*result = bmp180_read_temperature();
			return 1;
		}
		break;
		
		case PRESSURE_PENDING:
		case PRESSURE_BUSY:
		case PRESSURE_DONE:				
		break;
		
		default:
		{
			char _string[4];
			_string[0] = 'E';
			_string[1] = _status + 48;
			_string[2] = '\r';
			_string[3] = '\n';
			app_usb_cdc_write_char_string(_string, sizeof(_string));						
		}		
		break;
	}
				
	return 0;
}

int app_sensors_sample_pressure(float *result)
{		
	int _status = bmp180_measurement_status();
	
	switch(_status)
	{		
		case SENSOR_IDLE:
		case PRESSURE_PENDING:
		case PRESSURE_BUSY:
			bmp180_measure_pressure();		
		break;
			
		case PRESSURE_DONE:
		{
			*result = bmp180_read_pressure();
			return 1;					
		}
		break;
									
		case TEMPERATURE_PENDING:
		case TEMPERATURE_BUSY:						
		case TEMPERATURE_DONE:			
		break;		
			
		default:
		{
			char _string[4];
			_string[0] = 'E';
			_string[1] = _status + 48;
			_string[2] = '\r';
			_string[3] = '\n';
			app_usb_cdc_write_char_string(_string, sizeof(_string));
		}
		break;
	}
		
	return 0;
}