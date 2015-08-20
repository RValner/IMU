/*
 * L3G4200D.h
 * based on http://bildr.org/2011/06/l3g4200d-arduino/
 * Created: 21.07.2015 16:27:43
 */ 


#ifndef L3G4200D_H_
#define L3G4200D_H_

#include "XmegaTWI.h"

float toDegSec(float in, int scale);

void getGyroValues(int *x, int *y, int *z);

void getGyroValues(float *x, float *y, float *z);

void setupL3G4200D(int scale);

void calibrateGyro(float *xCalib, float *yCalib, float *zCalib, int scale);

#endif /* L3G4200D_H_ */