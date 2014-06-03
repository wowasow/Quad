/*
 * Motor.h
 *
 *  Created on: Apr 26, 2014
 *      Author: wowas
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>

class Motors {
public:

	// initializes all PWMs
	static void initPWM(unsigned int pwm);

	// sets power in the range of 0 - 100 on the specific PWMs
	static void setPower(float power, uint8_t pwm);

	// for throttle calibration
	static void calibrate(uint8_t pwm);

private:
	const static int MIN_THROTTLE = 16000;
	const static int MAX_THROTTLE = 32000;

	static bool enabled;
};

#endif /* MOTOR_H_ */
