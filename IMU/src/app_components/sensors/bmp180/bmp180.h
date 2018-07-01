/*
 * bmp180.h
 *
 * Created: 1/3/2015 8:46:12 PM
 *  Author: Daniel
 */ 

/**
 * \file
 *
 * \brief BMP180 digital pressure sensor driver
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


#ifndef BMP180_H_
#define BMP180_H_

//! \name Device address defines
//@{
#define BMP180_DEV_ADDR				0x77
//@}

//! \name Register address defines
//@{
#define BMP180_REG_OUTXLSB			0xF8
#define BMP180_REG_OUTLSB			0xF7
#define BMP180_REG_OUTMSB			0xF6
#define BMP180_REG_CTRL_MEAS		0xF4
#define BMP180_REG_SOFT_RESET		0xE0
#define BMP180_REG_ID				0xD0
#define BMP180_REG_CALIB21			0xBF
#define BMP180_REG_CALIB0			0xAA

//@}

//! \name Command byte defines
//@{
#define BMP180_CMD_SOFT_RESET		0xB6
#define BMP180_CMD_ID_RETURN		0x55
#define BMP180_CMD_MEAS_TEMPERATURE	0x2E
#define BMP180_CMD_MEAS_PRESURE_OSS0 0x34
#define BMP180_CMD_MEAS_PRESURE_OSS1 0x74
#define BMP180_CMD_MEAS_PRESURE_OSS3 0xF4
//@}


/************************************************************************/
/* bmp180_control_register - ref Table 6 & 8 - Internal use by driver only!	                                              
	\param oss			Controls oversampling ratio of the pressure measurement (00b: 1x, 01b: 2x, 10b: 4x, 11b: 8x)
	\param sco			Start of Conversion. 1 = conversion in progress, 0 = conversion complete
	\param meas_control	Temp = 0x2E, Pressure (oss=0) 0x34, Pressure (oss=1) 0x74, Pressure (oss=2) 0xB4, Pressure (oss=8) 0xF4
						 00 1 0 1110b		  00 1 1 0100b			 01 1 1 0100b			10 1 1 0100b		   11 1 1 0100b
						 b7,6  b5    b4,b3,b2,b1,b0 				
						 [oss] [sco] [temp/pressure]
									 0x0E = Temperature, 0x14 = Pressure
*/
/************************************************************************/
typedef struct bmp180_control_register {
	uint8_t oss : 2;
	uint8_t sco : 1;
	uint8_t meas_control : 5;
} bmp180_control;

typedef enum {
	GENERAL_ERROR = 0, //-3,
	PRESSURE_ERROR,// = -2,
	TEMPERATURE_ERROR,// = -1,
	SENSOR_IDLE,// = 0,
	TEMPERATURE_PENDING,// = 1,
	TEMPERATURE_BUSY,
	TEMPERATURE_DONE,
	PRESSURE_PENDING,
	PRESSURE_BUSY,	
	PRESSURE_DONE
} measurement_status;

/*
http://www.cs.usfca.edu/~wolber/SoftwareDev/C/CStructs.htm
*/
typedef struct {
	union 
	{
		int16_t AC1;
		Byte AC1_ba[2];
	};
	
	union
	{
		int16_t AC2;
		Byte AC2_ba[2];
	};
	
	union
	{
		int16_t AC3;
		Byte AC3_ba[2];
	};
	
	union
	{
		uint16_t AC4;
		Byte AC4_ba[2];
	};
	
	union
	{
		uint16_t AC5;
		Byte AC5_ba[2];	
	};
	
	union
	{
		uint16_t AC6;
		Byte AC6_ba[2];	
	};
	
	union
	{
		int16_t B1;
		Byte B1_ba[2];
	};
	
	union
	{
		int16_t B2;
		Byte B2_ba[2];
	};
	
	union
	{
		int16_t MB;
		Byte MB_ba[2];	
	};
	
	union
	{
		int16_t MC;
		Byte MC_ba[2];	
	};
	
	union
	{
		int16_t MD;
		Byte MD_ba[2];		
	};
	
	//uint_fast16_t test;
			
} bmp180_calibration_coefficients_register;

int bmp180_init(void);

void bmp180_soft_reset(void);

//! \name BMP180 measurement status function
//@{
/**
 * \brief Provides status as to whether or not the sensor is currently tasked with taking a sensor measurement.
 *
 * If a measurement operation is in progress, the returned value will indicate which measurement is underway:
 * -3 = general sensor error
 * -2 = pressure sensor measurement error
 * -1 = temperature sensor measurement error
 * 0 = idle, no measurements in progress
 * 1 = temperature sensor measurement in progress
 * 2 = pressure sensor measurement in progress
 * 3 = temperature sensor measurement complete
 * 4 = pressure sensor measurement complete
 *
 * \retval int
 */
int bmp180_measurement_status(void);

/////////////////////////
//Temperature functions//
/////////////////////////

//! \name BMP180 measure temperature function 
//@{
/**
 * \brief Sends a command sequence to start a temperature sensor measurement
 *
 * This function will start a request (may take up to 20ms) for the temperature and will return a 2 if the data is ready.
 * The function does not wait in a loop. It can be called in a loop or checked at some pre-defined interval. Requesting pressure
 * data before a valid temperature is returned will overwrite the temperature data with pressure data. Calling sensor read
 * functions concurrently may result in data being invalid as the sensor is being re-tasked before it can return a sample
 *
 * \retval status of the read: 0 idle, 1 read in progress, 2 read success, 3 read error
 */
int bmp180_measure_temperature(void);

//! \name BMP180 read temperature function 
//@{
/**
 * \brief Sends a command sequence to read the MSB and LSB bytes in the temperature register and returns the result as a float
 * in degress C.
 *    
 * \retval float
 */
float bmp180_read_temperature(void);

/////////////////////////
//Pressure Functions//
/////////////////////////

//! \name BMP180 measure pressure function 
//@{
/**
 * \brief Sends a command sequence to start a pressure sensor measurement
 *
 * This function will start a request (may take up to XXms) for the pressure and will return a TBD if the data is ready.
 * The function does not wait in a loop. It can be called in a loop or checked at some pre-defined interval. Note that 
 * the temperature should be read before requesting pressure data so that the pressure reading will be temperature compensated.
 *
 * \retval status TBD
 */
int bmp180_measure_pressure(void);

float bmp180_read_pressure(void);

#endif /* BMP180_H_ */