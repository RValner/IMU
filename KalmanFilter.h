/*
 * KalmanFilter.h
 *
 * Created: 16.07.2015 15:53:46
 *  Author: Robert Valner
 *
* * *---DESCRIPTION---
*
* This library gives tools for 1D Kalman filtering with 2 main inputs and is intended for
* multi purpose use:
* "scalar" - this input represents the scalar part of the filter which can be height, angle,
*		pressure etc.
* "rate" - this input represents the process quantity of the filter which can be linear
*		velocity, angular velocity, acceleration etc.
*
* In other words, the filer estimates the scalar quantity and a bias of process/rate quantity
* and the estimation model is:
*
*			estimatedScalar = previousScalar + timeStep*(measuredRate - rateBias)
			rateBias = rateBias + KalmanMagic

* * *---CREDITS---
*
* Based on a TKJ Electronics blog post:
* "A practical approach to Kalman filter and how to implement it" by Kristian Lauszus
* Link: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
*
* Also, link to a paper which explains how Kalman filter equations are derived
* in really intuitive manner:
* Link: http://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf

* Feel free to use, modify, be happy about or ridicule the code
 */


#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

class kalmanFilter
{
	public:

		// Constructor
		kalmanFilter(float timeStep, float rateBiasProcessNoise, float scalarProcessNoise, float varianceOfScalar);

		// Destructor
		~kalmanFilter();

		// Iteration function, updates the scalar and rate bias
		void iterate(float measuredRate, float measuredScalar);

		// Rate and scalar initialization
		void init(float rate, float scalar);

		// Get functions for rate and scalar
		float getRateBias();
		float getScalar();

	protected:

		float angle = 0, rateBias = 0;
		float angleRate = 0;			// Rate of change of whatever input
		float dt;						// Timestep in seconds
		float P[2][2] = {{0,0},{0,0}};
		float K[2] = {0,0};
		float innovation, S;

		float Q_rateBias;				// Variance of rate bias estimation process noise
		float Q_angle;					// Variance of scalar estimation process noise
		float u_angle;					// Variance of scalar measurement
};



#endif /* KALMANFILTER_H_ */
