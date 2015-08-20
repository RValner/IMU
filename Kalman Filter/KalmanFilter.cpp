/*
 * KalmanFilter.cpp
 *
 * Created: 16.07.2015 16:51:47
 *  Author: Robert
 */

#include "KalmanFilter.h"

// Constructor
/*
	timeStep - time between iteration cycles in seconds
	rateBiasProcessNoise -
*/
kalmanFilter::kalmanFilter(float timeStep, float rateBiasProcessNoise, float scalarProcessNoise, float varianceOfScalar)
{
	dt = timeStep;
	Q_rateBias = rateBiasProcessNoise;
	Q_angle = scalarProcessNoise;
	u_angle = varianceOfScalar;
}

// Destructor
kalmanFilter::~kalmanFilter(){}

// Initialize - this function initializes the rate and scalar variables
void kalmanFilter::init(float rate, float scalar)
{
	rateBias = rate;
	angle = scalar;
}

// Get rate
float kalmanFilter::getRateBias()
{
	return rateBias;
}

// Get scalar
float kalmanFilter::getScalar()
{
	return angle;
}

// Iterate
void kalmanFilter::iterate(float measuredRate, float measuredScalar)
{
	// Prediction
	angleRate = measuredRate - rateBias;
	angle += angleRate*dt;

	// Prediction covariance matrix
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_rateBias * dt;

	// Innovation
	innovation = measuredScalar - angle;

	// Innovation error
	S = P[0][0] + u_angle;

	// Kalman gain
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Posteriori angle
	angle += K[0] * innovation;
	rateBias += K[1] * innovation;

	// Posteriori covariance matrix
	P[0][0] -= K[0] * P[0][0];
	P[0][1] -= K[0] * P[0][1];
	P[1][0] -= K[1] * P[0][0];
	P[1][1] -= K[1] * P[0][1];
}
