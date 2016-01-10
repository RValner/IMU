#pragma once // include guard

// Structure for holding Kalman filter variables and pointers
typedef struct _KalmanFilter
{
    float angle;
    float rateBias;                 // Bias of rate variable
    float angleRate;			    // Rate of change of whatever input
    float dt;						// Timestep in seconds
    float P[2][2];                  // Covariance matrix
    float K[2];                     // Kalman gain
    float innovation;
	float S;
    float Q_rateBias;				// Variance of rate bias estimation process noise
    float Q_angle;					// Variance of scalar estimation process noise
    float u_angle;					// Variance of scalar measurement
} KalmanFilter;

// Constructor of KalmanFilter "object"
KalmanFilter* kalmanFilter_Construct(float timeStep, float rateBiasProcessNoise, float scalarProcessNoise, float varianceOfScalar);

// Set the initial state for the rate, rate bias and scalar values
void kalmanFilter_Init(KalmanFilter* filter, float rate, float scalar);

// Destructor of KalmanFilter "object"
void kalmanFilter_Destroy(KalmanFilter* filter);

// Iteration function, updates the scalar and rate bias
void kalmanFilter_Iterate(KalmanFilter* filter, float* measuredRate, float* measuredScalar);