/*****************************************************************
 LSM9DS0_Simple.ino
 SFE_LSM9DS0 Library Simple Example Code
 Jim Lindblom @ SparkFun Electronics
 Original Creation Date: February 18, 2014
 https://github.com/sparkfun/LSM9DS0_Breakout

 The LSM9DS0 is a versatile 9DOF sensor. It has a built-in
 accelerometer, gyroscope, and magnetometer. Very cool! Plus it
 functions over either SPI or I2C.

 This Arduino sketch is a demo of the simple side of the
 SFE_LSM9DS0 library. It'll demo the following:
 * How to create a LSM9DS0 object, using a constructor (global
 variables section).
 * How to use the begin() function of the LSM9DS0 class.
 * How to read the gyroscope, accelerometer, and magnetometer
 using the readGryo(), readAccel(), readMag() functions and the
 gx, gy, gz, ax, ay, az, mx, my, and mz variables.
 * How to calculate actual acceleration, rotation speed, magnetic
 field strength using the calcAccel(), calcGyro() and calcMag()
 functions.
 * How to use the data from the LSM9DS0 to calculate orientation
 and heading.

 Hardware setup: This library supports communicating with the
 LSM9DS0 over either I2C or SPI. If you're using I2C, these are
 the only connections that need to be made:
 LSM9DS0 --------- Arduino
 SCL ---------- SCL (A5 on older 'Duinos')
 SDA ---------- SDA (A4 on older 'Duinos')
 VDD ------------- 3.3V
 GND ------------- GND
 (CSG, CSXM, SDOG, and SDOXM should all be pulled high jumpers on
 the breakout board will do this for you.)

 If you're using SPI, here is an example hardware setup:
 LSM9DS0 --------- Arduino
 CSG -------------- 9
 CSXM ------------- 10
 SDOG ------------- 12
 SDOXM ------------ 12 (tied to SDOG)
 SCL -------------- 13
 SDA -------------- 11
 VDD -------------- 3.3V
 GND -------------- GND

 The LSM9DS0 has a maximum voltage of 3.6V. Make sure you power it
 off the 3.3V rail! And either use level shifters between SCL
 and SDA or just use a 3.3V Arduino Pro.

 Development environment specifics:
 IDE: Arduino 1.0.5
 Hardware Platform: Arduino Pro 3.3V/8MHz
 LSM9DS0 Breakout Version: 1.0

 This code is beerware. If you see me (or any other SparkFun
 employee) at the local, and you've found our code helpful, please
 buy us a round!

 Distributed as-is; no warranty is given.
 *****************************************************************/

// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include "utils/SFE_LSM9DS0.h"

#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "motors/Motors.h"
#include "motors/Motor.h"

extern "C" {
#include "uart/uart.h"
#include "utils/TWI.h"
#include "utils/Millis.h"
}

#include "math.h"

#define UART_BAUD_RATE 57600

//#define DEBUG
#define PRINT_SPEED 0 // 100 ms between prints

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x3A // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0xD6 // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

///////////////////////
// Example SPI Setup //
///////////////////////
/* // Uncomment this section if you're using SPI
 #define LSM9DS0_CSG  9  // CSG connected to Arduino pin 9
 #define LSM9DS0_CSXM 10 // CSXM connected to Arduino pin 10
 LSM9DS0 dof(MODE_SPI, LSM9DS0_CSG, LSM9DS0_CSXM);
 */

// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
#define PRINT_CALCULATED
//#define PRINT_RAW

void setup() {
//  Serial.begin(115200); // Start serial at 115200 bps
	// Use the begin() function to initialize the LSM9DS0 library.
	// You can either call it with no parameters (the easy way):
	uint16_t status = dof.begin(dof.G_SCALE_245DPS, dof.A_SCALE_2G,
			dof.M_SCALE_2GS, dof.G_ODR_95_BW_25, dof.A_ODR_25, dof.M_ODR_25);
}

float getPitchAcc(LSM9DS0 * dof) {
	dof->readAccel();
	float x = dof->calcAccel(dof->ax);
	float z = dof->calcAccel(dof->az);

	float cos = x / sqrt((z * z) + (x * x));
	float pitch = acos(cos);

	pitch *= 180.0 / M_PI;

	return pitch;
}

// for capturing time
long curr_time = 0;
long prev_time = 0;

// for getting track of a current angle between X and Z axises
double pitch_angle = 0;

// desired angle
double pitch_target_angle = 0;

#define INTEGRAL

#define DERIVATIVE

#define MOTORS


// critical k_p
const double k_c = -0.25;

// Kp
const double k_p = -0.15;

// Ki
const double k_i = -0.000002;

// Kd
const double k_d = -1;

// integral
double integral = 0;

// current error
double curr_pitch_err = 0;

// change
double change = 0;

// default power
double target_power = 30;

// the biggest change
double MAX_CHANGE = 30;

double lastError = 0;

double derivative = 0;

long diff = 0;

int main(void) {
#ifdef MOTORS

	/***************************************************/
	// MOTORS
	Motor motor0(Motor::PWM3);
	Motor motor1(Motor::PWM2);

	Motors::initPWM(Motor::PWM2 | Motor::PWM3);
	Motors::calibrate(Motor::PWM2 | Motor::PWM3);

	/**************************************************/
#endif

	TIME::initMillis();

#ifdef DEBUG
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
#endif

	// PA0 - for checking if the program reboots unintended
	// PA2 - for finding out when the program reads data
	// PA3 - if a pitch_angle overruns the threshold 1000
	// PA4 - for turning the motors off, switch activated by a low logical level
	// PA5 - indicates the motors being turned of by an external command
//	DDRA |= _BV(PA0) | _BV(PA2) | _BV(PA3) | _BV(PA5);
//	PINA |= _BV(PA4);
//	turnLed(_BV(PA0), 1000);

	setup();
	for (int i = 0; i < 100; i++) {
		pitch_angle = getPitchAcc(&dof);
	}

	sei();
#ifdef MOTORS

	motor0.setPower(target_power);
	motor1.setPower(target_power);
#endif

	while (1) {
#ifdef MOTORS
		if (!(PINA & _BV(PA4))) {
			motor0.setPower(0);
			motor1.setPower(0);

			PORTA |= _BV(PA5);
			while (1) {
				// stop the program
			}
		}

#endif

		curr_time = TIME::millis();
		diff = curr_time - prev_time;
		prev_time = curr_time;
//		diff /= 1000;

		dof.readAccel();
		dof.readGyro();
		pitch_angle = 0.98
				* (pitch_angle + dof.calcGyro(dof.ay) * (diff / 1000))
				+ 0.02 * getPitchAcc(&dof);
#ifdef DEBUG
		char buff[100];
		sprintf(buff, "pitch_angle, diff: %.2f, %.10f\n\r",
				(double) pitch_angle, (double) diff);
		uart_puts(buff);
#endif
//		turnLed(_BV(PA2), 0);

		// P controller
		curr_pitch_err = pitch_angle - pitch_target_angle;
		change = k_p * curr_pitch_err;
#ifdef INTEGRAL
		integral += curr_pitch_err;
		change += k_i * integral;
#endif

#ifdef DERIVATIVE
		derivative = curr_pitch_err - lastError;
		change += k_d * derivative;
		lastError = curr_pitch_err;
#endif

		change = change > MAX_CHANGE ? MAX_CHANGE : change;
#ifdef MOTORS
		motor0.setPower(target_power - change);
		motor1.setPower(target_power + change);
#endif
	}

}

ISR(BADISR_vect) {
	for (;;)
		uart_putc('!');
}
