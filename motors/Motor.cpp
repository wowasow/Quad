#include "Motor.h"
#include "Motors.h"

Motor::Motor(unsigned int motor) {
	this->motor = motor;
	this->power = 0;

//	Motors::initPWM(this->motor); // change
//	Motors::calibrate(motor);
}

float Motor::getPower() const {
	return power;
}

void Motor::setPower(float power) {
	this->power = power;
	Motors::setPower(this->power, this->motor);
}
