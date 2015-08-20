/*
 * MMA8451Q.h
 * Based on: http://www.kerrywong.com/2012/01/09/interfacing-mma8453q-with-arduino/
 * Created: 17.07.2015 16:33:48
 */ 


#ifndef MMA8451Q_H_
#define MMA8451Q_H_

void standbyMode();

void activeMode();

void hiResMode();

void getAccXYZ(int *x, int *y, int *z);

void getAccXYinAngles(float *Xout,  float *Yout);

//Not really a calibration, but a function to get initial values
void calibrateAccelerometer(float *x, float *y);

// Initialize MMA8451Q
void InitMMA8451Q();

#endif /* MMA8451Q_H_ */