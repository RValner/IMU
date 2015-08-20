/*
 * L3G4200D.cpp
 * based on http://bildr.org/2011/06/l3g4200d-arduino/
 * Created: 21.07.2015 16:42:21
 */ 

#include <util/delay.h>
#include "L3G4200D.h"

uint8_t L3G4200D_Address = 105; //I2C address of the L3G4200D

uint8_t CTRL_REG1 = 0x20;
uint8_t CTRL_REG2 = 0x21;
uint8_t CTRL_REG3 = 0x22;
uint8_t CTRL_REG4 = 0x23;
uint8_t CTRL_REG5 = 0x24;
uint8_t ReadGyroXYZ = 0xA8;

const double RawToDeg = 0.007629394; //500 (deg/s) / 2^16
int scale = 500;

float toDegSec(float in, int scale)
{
	return in/65536.0*scale;
}

void getGyroValues(int *x, int *y, int *z)
{
	
	uint8_t buf[6];
	regRead(L3G4200D_Address, &ReadGyroXYZ, buf, 6);
	
	*x = ((buf[1] << 8) | buf[0]);
	*y = ((buf[3] << 8) | buf[2]);
	*z = ((buf[5] << 8) | buf[4]);
}

void getGyroValues(float *x, float *y, float *z)
{
	int xi = 0;
	int yi = 0;
	int zi = 0;
	
	getGyroValues( &xi, &yi, &zi);
	
	*x = toDegSec((float)xi, 500);
	*y = toDegSec((float)yi, 500);
	*z = toDegSec((float)zi, 500);
}

void setupL3G4200D(int scale)
{
	//From  Jim Lindblom of Sparkfun's code

	// Enable x, y, z and turn off power down:
	uint8_t byteToWrite1 = 0b00001111;
	uint8_t byteToWrite2 = 0b00001000;
	uint8_t byteToWrite3 = 0b00000000;
	uint8_t byteToWrite4 = 0b00010000;
	uint8_t byteToWrite5 = 0b00110000;
	uint8_t byteToWrite6 = 0b00000000;
	uint8_t byteToWrite7 = 0b00000000;
	
	regWrite(L3G4200D_Address, CTRL_REG1, byteToWrite1);
	
	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	regWrite(L3G4200D_Address, CTRL_REG2, byteToWrite6);

	// Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	regWrite(L3G4200D_Address, CTRL_REG3, byteToWrite2);

	// CTRL_REG4 controls the full-scale range, among other things:

	if(scale == 250)
	{
		regWrite(L3G4200D_Address, CTRL_REG4, byteToWrite3);
	}
	else
	{
		regWrite(L3G4200D_Address, CTRL_REG4, byteToWrite4);
	}
	
	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	regWrite(L3G4200D_Address, CTRL_REG5, byteToWrite7);
}

void calibrateGyro(float *xCalib, float *yCalib, float *zCalib, int scale)
{
	int xGyro = 0;
	int yGyro = 0;
	int zGyro = 0;
	
	*xCalib = 0;
	*yCalib = 0;
	*zCalib = 0;
	
	uint8_t loopCount = 20;
	
	for (int i=0; i<loopCount; i++){
		getGyroValues(&xGyro, &yGyro, &zGyro);
		
		*xCalib += xGyro;
		*yCalib += yGyro;
		*zCalib += zGyro;
		_delay_ms(5);
	}
	
	*xCalib = toDegSec(*xCalib/loopCount, scale);
	*yCalib = toDegSec(*yCalib/loopCount, scale);
	*zCalib = toDegSec(*zCalib/loopCount, scale);
}