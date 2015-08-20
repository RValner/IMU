#include <avr/io.h>
#include <stdio.h>
#include "twi_master_driver.h"
#include "XmegaTWI.h"
#include "MMA8451Q.h"
#include "L3G4200D.h"
#include "KalmanFilter.h"
#include "math.h"

extern "C" 
{
	#include "usart_driver.h"
}
/*! Defining number of uint8_ts in buffer. */
#define NUM_uint8_tS 8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */

//USART
#define USART USARTC0

float xAcc = 0;			// Accelerometer variables
float yAcc = 0;
float zAcc = 0;

char XstringBuffer[30];					// String buffer for USART messages

uint8_t enabled = 0;

// Gyro
//int scale = 500;						//If you change scale, change "raw to deg" in L3G4200D.h

float xGyro;
float yGyro;
float zGyro;

void initUART()
{
	PORTC.DIRSET = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	
	/* Enable RXC interrupt. */
	//USART_RxdInterruptLevel_Set(&USART, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	//USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);
	
	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
}

//Stringi saatmise funktsioon
void transmitString(char* string)
{
	while(*string)
	{
		while(!USART_IsTXDataRegisterEmpty(&USART));
		USART_PutChar(&USART, *string++);
	}
	
}

int main(void)
{
	//CTRLA = 0x01;
	PORTC_DIR = 0xFF;
	
	uint8_t receivedData;
	
	// Create kalman filter objects 
	kalmanFilter kalmanX(0.09, 0.003, 0.01, 0.1);
	kalmanFilter kalmanY(0.09, 0.003, 0.01, 0.1);
	
	// Init UART 
	initUART();
	
	// Initialize TWI master
	initTWI();
	
	// Enable LO interrupt level
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
	
	// Initialize gyro and accelerometer
	InitMMA8451Q();
	
	setupL3G4200D(500); // Configure L3G4200  - 250, 500 or 2000 deg/sec
	
	// Calibrate the accelerometer and gyro
	calibrateAccelerometer(&xAcc, &yAcc);
	calibrateGyro(&xGyro, &yGyro, &zGyro, 500);
	
	// Initialize kalman filter objects
	kalmanX.init(-1*yGyro, xAcc);
	kalmanY.init(xGyro, yAcc);
	
	// -------MAIN LOOP-------- //
	
    while(1)
    {
		
		// Get accelerometer and gyro readings
        getAccXYinAngles(&xAcc, &yAcc);
		getGyroValues(&xGyro, &yGyro, &zGyro);
		
		// Pass the aqcuired variables to kalman filter
		kalmanX.iterate(-1*yGyro, xAcc);
		kalmanY.iterate(xGyro, yAcc);
		
		// Transmit data over UART
		//sprintf(XstringBuffer, "A: %0.2f G: %0.2f K: %0.2f\n", (double)yAcc, gyroIntegrate, (double)kalmanY.getScalar());
		
		//sprintf(XstringBuffer, "Elus %d", 1);
		//sprintf(XstringBuffer, "x%0.2f\n", (double)kalmanX.getScalar());
		
		sprintf(XstringBuffer, "x%0.2f y%0.2f\n", (double)kalmanX.getScalar(),(double)kalmanY.getScalar());
		transmitString(XstringBuffer);
		
		_delay_ms(1);
    }
}

/*! TWIC Master Interrupt vector. */


ISR(TWIC_TWIM_vect)
{
	TWIInterrtuptHandler();
}


/*
ISR(USARTC0_RXC_vect)
{
	if(USART_GetChar(&USART) == 's')
	{
		enabled = 1;
	}
	
	else if(USART_GetChar(&USART) == 'x')
	{
		enabled = 0;
	}
}
*/