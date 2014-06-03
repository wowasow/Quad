#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

namespace TIME {
volatile unsigned long curr_millis;

unsigned long millis() {
	return curr_millis;
}

void initMillis() {
	curr_millis = 0;

	// set timer OC0, CTC, prescaler 64
	TCCR0 |= _BV(WGM01) | _BV(COM01) | _BV(CS01) | _BV(CS00);

	// enable output compare match interrupt
	// counting milliseconds not seconds
	TIMSK |= _BV(OCIE0);

	OCR0 = 249;

	sei();

	DDRA |= _BV(PA1);

}

ISR(TIMER0_COMP_vect) {
	curr_millis++;
}
}
