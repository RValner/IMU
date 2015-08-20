/*
 This code demonstrates how to use the KalmanFilter library. In this example, kalman filter
 is used to get filtered tilt information that can be used for quadcopter stabilisation,
 servo positioning etc. Filtered data is "sent" over UART for further use.
*/

#include "KalmanFilter.h"

// Dummy library imports for demonstrarional purposes
#include "arbitraryAccelerometer.h"
#include "arbitraryGyroscope.h"

float accelReading;         // Variable for storing processed accelerometer data.    Unit: deg
float gyroReading;          // Variable for storing processed gyroscope data.        Unit: deg/s

char stringBuffer[50];     // Char buffer array for sending strings over UART

/* Create Kalman Filter object with parameters:
 kalmanFilter(float timeStep, float rateBiasProcessNoise, float scalarProcessNoise, float varianceOfScalar)
 Units in this example: (seconds, deg/s, deg, deg)
*/

kalmanFilter kalmanObject(0.09, 0.003, 0.01, 0.1);

int main()
{
    // Accelerometer and gyro are initialised
    initAccelerometer();
    initGyroscope();

    // accelReading and gyroReading variables are initialised, in other words
    // multiple measurements are taken and mean value is returned ino the variables
    accelReading = returnAccelCalibration;
    gyroReading = returnGyroCalibration;

    // Initialise Kalman Filter object
    kalmanObject.init(gyroReading, accelReading);

    // Main loop
    while(true)
    {
        // Get accelerometer and gyro readings
        accelReading = returnAccelDeg;
        gyroReading = returnGyroDegS;

		// Pass the aqcuired variables to kalman filter
		kalmanObject.iterate(gyroReading, accelReading);

		// Send the filtered data over UART
		sprintf(XstringBuffer, "The angle is %0.2f deg; Angle rate is %0.2f\n", (double)kalmanObject.getScalar(),(double)(gyroReading - kalmanObject.getRateBias() ));
		transmitString(XstringBuffer);
    }
}
