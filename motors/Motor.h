#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>

class Motor {
public:

	const static unsigned int PWM0 = _BV(PB3);
	const static unsigned int PWM1 = _BV(PD7);
	const static unsigned int PWM2 = _BV(PD4);
	const static unsigned int PWM3 = _BV(PD5);

	Motor(unsigned int motor);

	float getPower() const;

	void setPower(float power);

private:
	unsigned int motor;
	float power;
};

#endif /* MOTOR_H_ */
