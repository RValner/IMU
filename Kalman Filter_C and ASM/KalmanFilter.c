#include "KalmanFilter.h"

// Constructor of KalmanFilter "object"
KalmanFilter* kalmanFilter_Construct(float timeStep, float rateBiasProcessNoise, float scalarProcessNoise, float varianceOfScalar)
{
    KalmanFilter* filter = malloc(sizeof(KalmanFilter)); // Allocate memory

	filter->angle       = 0;
	filter->rateBias    = 0;
	filter->angleRate   = 0;
    filter->dt          = timeStep;
    memset(filter->P, 0, 4*sizeof filter->P[0][0]);
    memset(filter->K, 0, 2*sizeof filter->K[0]);
	filter->Q_rateBias  = rateBiasProcessNoise;
	filter->Q_angle     = scalarProcessNoise;
	filter->u_angle     = varianceOfScalar;

    return filter;
}

// Set the initial state for the rate, rate bias and scalar values
void kalmanFilter_Init(KalmanFilter* filter, float rate, float scalar)
{
    filter->angle       = scalar;
    filter->angleRate   = rate;
	filter->rateBias    = rate;
}

// Destructor of KalmanFilter "object"
void kalmanFilter_Destroy(KalmanFilter* filter) // destroys the vector and frees any allocated data
{
	if (filter)
		free(filter);           // destroy this object
}