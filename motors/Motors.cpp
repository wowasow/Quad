#include "Motors.h"
#include "Motor.h"
#include <util/delay.h>
#include <avr/io.h>

void Motors::initPWM(unsigned int pwm) {

	// OC1B
	// fPWM = 400Hz
	// throttle 16000 - 32000
	if (pwm & Motor::PWM2) {
		if (enabled) {
			//NON Inverted PWM
			TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
			//PRESCALER=1 MODE 14(FAST PWM)
			TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10);
			ICR1 = 39999;
		}
		DDRD |= (1 << PD4);

		enabled = false;
	}

	// OC1A
	// fPWM = 400Hz
	// throttle 16000 - 32000
	if (pwm & Motor::PWM3) {
		if (enabled) {
			//NON Inverted PWM
			TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
			//PRESCALER=none MODE 14(FAST PWM)
			TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10);
			ICR1 = 39999;
		}
		DDRD |= (1 << PD5);

		enabled = false;
	}
}

void Motors::setPower(float power, uint8_t pwm) {
	// power range: 16000 - 32000
	power = MIN_THROTTLE + (power / 100 * (MAX_THROTTLE - MIN_THROTTLE));

	if (pwm & Motor::PWM0)
		OCR0 = power;

	if (pwm & Motor::PWM1)
		OCR2 = power;

	if (pwm & Motor::PWM2)
		OCR1A = power;

	if (pwm & Motor::PWM3)
		OCR1B = power;

}

void Motors::calibrate(uint8_t pwm) {
	setPower(100, pwm);
	_delay_ms(8000);
	setPower(0, pwm);
	_delay_ms(8000);
}

bool Motors::enabled = true;
