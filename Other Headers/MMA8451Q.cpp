/*
 * MMA8451Q.cpp
 * Based on: http://www.kerrywong.com/2012/01/09/interfacing-mma8453q-with-arduino/
 * Created: 17.07.2015 16:41:43
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include "MMA8451Q.h"
#include "XmegaTWI.h"
#include "math.h"

uint8_t REG_STATUS = 0x00; //(R) Real time status
uint8_t REG_OUT_X_MSB = 0x01; //(R) [7:0] are 8 MSBs of 10-bit sample
uint8_t REG_OUT_X_LSB = 0x02; //(R) [7:6] are 2 LSBs of 10-bit sample
uint8_t REG_OUT_Y_MSB = 0x03; //(R) [7:0] are 8 MSBs of 10-bit sample
uint8_t REG_OUT_Y_LSB = 0x04; //(R) [7:6] are 2 LSBs of 10-bit sample
uint8_t REG_OUT_Z_MSB = 0x05; //(R) [7:0] are 8 MSBs of 10-bit sample
uint8_t REG_OUT_Z_LSB = 0x06; //(R) [7:6] are 2 LSBs of 10-bit sample
uint8_t REG_SYSMOD = 0x0b; //(R) Current system mode
uint8_t REG_INT_SOURCE = 0x0c; //(R) Interrupt status
uint8_t REG_WHO_AM_I = 0x0d; //(R) Device ID (0x3A)
uint8_t REG_XYZ_DATA_CFG = 0x0e; //(R/W) Dynamic range settings
uint8_t REG_HP_FILTER_CUTOFF = 0x0f; //(R/W) cut-off frequency is set to 16Hz @ 800Hz
uint8_t REG_PL_STATUS = 0x10; //(R) Landscape/Portrait orientation status
uint8_t REG_PL_CFG = 0x11; //(R/W) Landscape/Portrait configuration
uint8_t REG_PL_COUNT = 0x12; //(R) Landscape/Portrait debounce counter
uint8_t REG_PL_BF_ZCOMP = 0x13; //(R) Back-Front, Z-Lock trip threshold
uint8_t REG_P_L_THS_REG = 0x14; //(R/W) Portrait to Landscape trip angle is 29 degree
uint8_t REG_FF_MT_CFG = 0x15; //(R/W) Freefall/motion functional block configuration
uint8_t REG_FF_MT_SRC = 0x16; //(R) Freefall/motion event source register
uint8_t REG_FF_MT_THS = 0x17; //(R/W) Freefall/motion threshold register
uint8_t REG_FF_MT_COUNT = 0x18; //(R/W) Freefall/motion debounce counter
uint8_t REG_TRANSIENT_CFG = 0x1d; //(R/W) Transient functional block configuration
uint8_t REG_TRANSIENT_SRC = 0x1e; //(R) Transient event status register
uint8_t REG_TRANSIENT_THS = 0x1f; //(R/W) Transient event threshold
uint8_t REG_TRANSIENT_COUNT = 0x20; //(R/W) Transient debounce counter
uint8_t REG_PULSE_CFG = 0x21; //(R/W) ELE, float_XYZ or Single_XYZ
uint8_t REG_PULSE_SRC = 0x22; //(R) EA, float_XYZ or Single_XYZ
uint8_t REG_PULSE_THSX = 0x23; //(R/W) X pulse threshold
uint8_t REG_PULSE_THSY = 0x24; //(R/W) Y pulse threshold
uint8_t REG_PULSE_THSZ = 0x25; //(R/W) Z pulse threshold
uint8_t REG_PULSE_TMLT = 0x26; //(R/W) Time limit for pulse
uint8_t REG_PULSE_LTCY = 0x27; //(R/W) Latency time for 2nd pulse
uint8_t REG_PULSE_WIND = 0x28; //(R/W) Window time for 2nd pulse
uint8_t REG_ASLP_COUNT = 0x29; //(R/W) Counter setting for auto-sleep
uint8_t REG_CTRL_REG1 = 0x2a; //(R/W) ODR = 800 Hz, STANDBY mode
uint8_t REG_CTRL_REG2 = 0x2b; //(R/W) Sleep enable, OS Modes, RST, ST
uint8_t REG_CTRL_REG3 = 0x2c; //(R/W) Wake from sleep, IPOL, PP_OD
uint8_t REG_CTRL_REG4 = 0x2d; //(R/W) Interrupt enable register
uint8_t REG_CTRL_REG5 = 0x2e; //(R/W) Interrupt pin (INT1/INT2) map
uint8_t REG_OFF_X = 0x2f; //(R/W) X-axis offset adjust
uint8_t REG_OFF_Y = 0x30; //(R/W) Y-axis offset adjust
uint8_t REG_OFF_Z = 0x31; //(R/W) Z-axis offset adjust

uint8_t FULL_SCALE_RANGE_2g = 0x0;
uint8_t ACCELaddress = 0x1C;			// Accelerometer address

void standbyMode()
{
	uint8_t reg;
	uint8_t activeMask = 0x01;
	
	regRead(ACCELaddress, &REG_CTRL_REG1, &reg, 1);
	reg &= ~activeMask;
	
	regWrite(ACCELaddress, REG_CTRL_REG1, reg);
}

void activeMode()
{
	uint8_t reg;
	uint8_t activeMask = 0x01;
	
	regRead(ACCELaddress, &REG_CTRL_REG1, &reg, 1);
	reg |= activeMask;
	regWrite(ACCELaddress, REG_CTRL_REG1, reg);
}

void hiResMode()
{
	uint8_t reg;
	uint8_t fastModeMask = 0x02;
	
	regRead(ACCELaddress, &REG_CTRL_REG1, &reg, 1);
	reg &= ~fastModeMask;
	regWrite(ACCELaddress, REG_CTRL_REG1,  reg);
}

void getAccXYZ(int *x, int *y, int *z)
{
	uint8_t buf[6];
	
	regRead(ACCELaddress, &REG_OUT_X_MSB, buf, 6);
	*x = buf[0] << 6 | buf[1] >> 2;
	*y = buf[2] << 6 | buf[3] >> 2;
	*z = buf[4] << 6 | buf[5] >> 2;
	
	if (*x > 8192) *x = *x - 16384;
	if (*y > 8192) *y = *y - 16384 ;
	if (*z > 8192) *z = *z - 16384;
	
}

void getAccXYinAngles(float *Xout,  float *Yout)
{
	int x, y, z;
	
	getAccXYZ(&x, &y, &z);
	*Xout = 57.3 * (float)atan2((double)x, (double)z); // 57.3 ~ 180/3.1416
	*Yout = 57.3 * (float)atan2((double)y, (double)z);
}

// Initialize MMA8451Q
void InitMMA8451Q()
{
	//Standby mode
	standbyMode(); //register settings must be made in standby mode
	
	regWrite(ACCELaddress, REG_XYZ_DATA_CFG, FULL_SCALE_RANGE_2g);
	
	uint8_t value1 = 0x20;
	uint8_t value2 = 0x2;
	
	regWrite(ACCELaddress, REG_CTRL_REG1, value1); //0x28 - 12.5 Hz, 0x20 - 50 Hz
	
	regWrite(ACCELaddress, REG_CTRL_REG2, value2); // 0x0 - normal mode (50 Hz OS=4, 12.5 Hz OS=16), 0x2 - high res mode (50 Hz OS=32, 12.5 Hz OS=124)
	hiResMode(); //this is the default setting and can be omitted.
	//lowResMode(); //set to low res (fast mode), must use getAccXYZ(,,,false) to retrieve readings.
	activeMode();
}

//
void calibrateAccelerometer(float *x, float *y)
{
	int xi, yi, zi;
	int nxi = 0, nyi = 0, nzi = 0;
	
	for (char n=0; n<20; n++)
	{
		getAccXYZ(&xi, &yi, &zi);
		
		nxi += xi;
		nyi += yi;
		nzi += zi;
		
		_delay_ms(10);
	}
	
	*x = 180.0/3.1416 * atan2( ((float)nxi)/20, ((float)nzi)/20);
	*y = 180.0/3.1416 * atan2( ((float)nyi)/20, ((float)nzi)/20);
}