/*
 * bmp180.c
 *
 * Created: 1/3/2015 10:49:34 PM
 *  Author: Daniel
 */ 
#include <compiler.h>

#include "bmp180.h"
#include "app_services/service_twi.h"

Byte _address[1] = {0x00},
	_command[1] = {0x00};
				
measurement_status _measurement_status = SENSOR_IDLE;

float _B5 = 0.0f;

bmp180_calibration_coefficients_register _coefficients =
{
	0, //int16_t AC1
	0, //int16_t AC2
	0, //int16_t AC3
	0, //uint16_t AC4
	0, //uint16_t AC5
	0, //uint16_t AC6
	0, //int16_t B1
	0, //int16_t B2
	0, //int16_t MB
	0, //int16_t MC
	0, //int16_t MD
};

void bmp180_set_measurement_status(measurement_status _status)
{
	_measurement_status = _status;
}

Byte bmp180_read_ctrl_meas(void)
{		
	_address[0] = BMP180_REG_CTRL_MEAS;
	Byte _data[1] = {0x00};
	
	service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _data, sizeof(_data));
			
	return _data[0];
}

//////////////////////////////////////////////////////////////////////////
//Public functions
//////////////////////////////////////////////////////////////////////////

int bmp180_init(void)
{
	//write code to read in the calibration coefficients here
	
	//buffer to hold 11 16-bit words (22 bytes)
	Byte _tempBuffer[(BMP180_REG_CALIB21 - BMP180_REG_CALIB0) + 1];
	
	Byte *_pointerToTemp = _tempBuffer;
		
	for(int i = 0; i < sizeof(_tempBuffer); i++)
	{
		_tempBuffer[i] = 0;
	}
	
	//read the 11 coefficients (22 bytes) starting at the base of the calibration byte addresses
	_address[0] = BMP180_REG_CALIB0;
					
	service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _pointerToTemp, sizeof(_tempBuffer));
	
	_coefficients.AC1_ba[1] = _tempBuffer[0];
	_coefficients.AC1_ba[0] = _tempBuffer[1];		
	
	_coefficients.AC2_ba[1] = _tempBuffer[2];
	_coefficients.AC2_ba[0] = _tempBuffer[3];
	
	_coefficients.AC3_ba[1] = _tempBuffer[4];
	_coefficients.AC3_ba[0] = _tempBuffer[5];
	
	_coefficients.AC4_ba[1] = _tempBuffer[6];
	_coefficients.AC4_ba[0] = _tempBuffer[7];
	
	_coefficients.AC5_ba[1] = _tempBuffer[8];
	_coefficients.AC5_ba[0] = _tempBuffer[9];
	
	_coefficients.AC6_ba[1] = _tempBuffer[10];
	_coefficients.AC6_ba[0] = _tempBuffer[11];
	
	_coefficients.B1_ba[1] = _tempBuffer[12];
	_coefficients.B1_ba[0] = _tempBuffer[13];
	
	_coefficients.B2_ba[1] = _tempBuffer[14];
	_coefficients.B2_ba[0] = _tempBuffer[15];
	
	_coefficients.MB_ba[1] = _tempBuffer[16];
	_coefficients.MB_ba[0] = _tempBuffer[17];
	
	_coefficients.MC_ba[1] = _tempBuffer[18];
	_coefficients.MC_ba[0] = _tempBuffer[19];
	
	_coefficients.MD_ba[1] = _tempBuffer[20];
	_coefficients.MD_ba[0] = _tempBuffer[21];
		
	return 0;
}

void bmp180_soft_reset(void)
{			
	_address[0] = BMP180_REG_SOFT_RESET;
	_command[0] = BMP180_CMD_SOFT_RESET;
	
	service_twi_write_register(BMP180_DEV_ADDR, _address, sizeof(_address), _command, sizeof(_command));	
}

int bmp180_measurement_status(void)
{
	return _measurement_status;
}

/////////////////////////
//Temperature functions//
/////////////////////////

int bmp180_measure_temperature(void)
{	
	//get the status of the Start of Conversion (Sco) bit
	Byte _ctrl = bmp180_read_ctrl_meas();
	
	Byte _status = bmp180_measurement_status();

	switch(_status)									
	{
		case SENSOR_IDLE:
			//set the control register to read temperature
			_address[0] = BMP180_REG_CTRL_MEAS,
			_command[0] = (_ctrl & 0x20) | BMP180_CMD_MEAS_TEMPERATURE;
				
			service_twi_write_register(BMP180_DEV_ADDR, _address, sizeof(_address), _command, sizeof(_command));
				
			bmp180_set_measurement_status(TEMPERATURE_PENDING);
		break;
			
		case TEMPERATURE_PENDING:
			//Sco is high if it is in the process of measuring something, otherwise low when "done
			if((_ctrl & 0x20) == 0x20)
			{
				bmp180_set_measurement_status(TEMPERATURE_BUSY);
			}
		break;
			
		case TEMPERATURE_BUSY:
			if((_ctrl | 0xDF) == 0xDF)
			{
				bmp180_set_measurement_status(TEMPERATURE_DONE);
			}
		break;
			
		default:
			//need to check that no pressure measurements are underway before setting error
			//bmp180_set_measurement_status(TEMPERATURE_ERROR);
		break;
	}
			
	return bmp180_measurement_status();
}

float bmp180_read_temperature(void)
{				
	float _temperatureFloat = 0.00f;
					
	if(bmp180_measurement_status() == TEMPERATURE_DONE)
	{	
			Byte _temperature_lsb[1] = {0x00},
			_temperature_msb[1] = {0x00};
			
			_address[0] = BMP180_REG_OUTLSB;
			
			//read the LSB data register
			service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _temperature_lsb, sizeof(_temperature_lsb));
									
			_address[0] = BMP180_REG_OUTMSB;
			
			//read the MSB data register
			service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _temperature_msb, sizeof(_temperature_msb));											
			
			//calculate the temperature using formula found on page 15
			
			//UT and T
			_temperatureFloat = (_temperature_msb[0] << 8) | _temperature_lsb[0];
			
			float X1 = (_temperatureFloat - (float)_coefficients.AC6) * ((float)_coefficients.AC5/32768),
				X2 = ((float)_coefficients.MC * 2048) / (X1 + (float)_coefficients.MD);
			
			_B5 = (X1 + X2);
														
			_temperatureFloat = ((_B5 + 8)/16) * 0.1;
															
			//temperature read is no longer occupying the sensor, return to idle
			bmp180_set_measurement_status(SENSOR_IDLE);
	}
			
	return _temperatureFloat;
}

/////////////////////////
//Pressure functions//
/////////////////////////

int bmp180_measure_pressure(void)
{		
	//check if a measurement is in progress
	Byte _ctrl = bmp180_read_ctrl_meas();
	
	Byte _status = bmp180_measurement_status();
	
	switch(_status)
	{
		case SENSOR_IDLE:
			//set the control register to read pressure, defaulting to OSS0 for now
			_address[0] = BMP180_REG_CTRL_MEAS,
			_command[0] = (_ctrl & 0x20) | BMP180_CMD_MEAS_PRESURE_OSS0;
			
			service_twi_write_register(BMP180_DEV_ADDR, _address, sizeof(_address), _command, sizeof(_command));
			
			bmp180_set_measurement_status(PRESSURE_PENDING);
		break;
			
		case PRESSURE_PENDING:
			//Sco is high if it is in the process of measuring something, otherwise low when "done
			if((_ctrl & 0x20) == 0x20)
			{
				bmp180_set_measurement_status(PRESSURE_BUSY);
			}
		break;
			
		case PRESSURE_BUSY:
			if((_ctrl | 0xDF) == 0xDF)
			{
				bmp180_set_measurement_status(PRESSURE_DONE);
			}
		break;
			
		default:
			//need to check that no temp measurements are underway before setting an error
			//bmp180_set_measurement_status(PRESSURE_ERROR);
		break;
	}
		
	return bmp180_measurement_status();
}

float bmp180_read_pressure(void)
{		
	float _pressureFloat = 0.0f;
	
	//hard coding _B5 for debug
	//float _B5 = 4289.0f;
	
	if(bmp180_measurement_status() == PRESSURE_DONE)
	{
		Byte _pressure_lsb[1] = {0x00},
		_pressure_msb[1] = {0x00};
		
		_address[0] = BMP180_REG_OUTLSB;
		
		//read the LSB data register
		service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _pressure_lsb, sizeof(_pressure_lsb));				
		
		_address[0] = BMP180_REG_OUTMSB;
		
		//read the MSB data register
		service_twi_read_register(BMP180_DEV_ADDR, _address, sizeof(_address), _pressure_msb, sizeof(_pressure_msb));			
		
		//calculate the temperature using formula found on page 15
		
		//UP and P
		_pressureFloat = (_pressure_msb[0] << 8) | _pressure_lsb[0];
				
		float _p = 0.00f, 
			_B6 = _B5 - 4000,
			
			X1 = ((float)_coefficients.B2 * ((_B6 * _B6) / 4096)) / 2048,			
			X2 = ((float)_coefficients.AC2 * _B6) / 2048,			
			X3 = X1 + X2,				
			B3 = (((float)_coefficients.AC1 * 4 + X3) + 2) / 4;
								
			X1 = ((float)_coefficients.AC3 * _B6) / 8192;			
			X2 = ((float)_coefficients.B1 * (_B6 * _B6 / 4096)) / 65536;			
			X3 = ((X1 + X2) + 2) / 4;			
			
		unsigned long _B4 = ((float)_coefficients.AC4 * (unsigned)(X3 + 32768)) / 32768;
				
		//note _B7 has a scaling option that is tied to the oversample (OSS). I am assuming 0 oversampling for now
		unsigned long _B7 = ((unsigned)_pressureFloat - B3) * (50000);	
				
			if(_B7 < 0x80000000)
				_p = (_B7 * 2) / _B4;
			else
				_p = (_B7 / _B4) * 2;
			
			X1 = (_p/256) * (_p/256);			
			X1 = (X1 * 3038) / 65536;			
			X2 = (-7357 * _p) / 65536;
			
			_p = _p + ((X1 + X2 + 3791) / 16);
								
		_pressureFloat = _p;
		
		bmp180_set_measurement_status(SENSOR_IDLE);							
	}
			
	return _pressureFloat;
}

