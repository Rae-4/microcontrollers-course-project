/*
 * microcontrollers-project.c
 *	
 *	EE.ELE.250 2022 Microcontrollers course project work.
 *
 * Created: 29.3.2022 12.34.20
 * Author : Joonas Ikonen <joonas.ikonen@tuni.fi>
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


#define PRESCALE_0 (1<<CS20)
#define PRESCALE_64 (1<<CS22)
#define PRESCALE_1024 (1<<CS22) | (1<<CS21) | (1<<CS20)


volatile uint8_t switch_power_off = 0;
volatile uint8_t switch_temp_ext = 0;

void setup();
void power_down();
void wake_up();
void set_temp(uint8_t percent);

int main(void)
{
    setup();
	/* TODO: Initialize PWM counter */
	/* TODO: LED IO */
	/* TODO: Console USART */
	/* TODO: ADC */
	
    while (1) 
    {
		/* Check is power switch is turned off */
		if (switch_power_off)
			power_down();
		
		/* TODO: Update reference temp if necessary */
		/* TODO: Measure the temperature */
		/* TODO: Update the PWM for heating element */
		/* TODO: Update the LED state */
    }
}

void setup()
{	
	/* Power saving setup */
	PRR |= (1<<PRTWI);		/* Shut down TWI */
	PRR |= (1<<PRTIM2);		/* Shut down Timer/Counter2 */
	PRR |= (1<<PRTIM1);		/* Shut down Timer/Counter1 */
	PRR |= (1<<PRTIM0);		/* Shut down Timer/Counter0 */
	
	/* External interrupt for switches setup */
	sei();
	PCICR |= (1<<PCIE2);					/* Enable PCINT2 interrupt vector */
	PCMSK2 |= (1<<PCINT23) | (1<<PCINT22);	/* Enable interrupt for the pins */
	
	/* Timer/Counter0 PWM setup */
	DDRD |= (1<<PIND3);								/* Set PD3/OC2B pin as an output */
	TCCR2A |= (1<<COM2B1) | (1<<WGM20);				/* Set output modes */
	TCCR2B |= PRESCALE_0;	/* Set 1024 prescaler (1<<CS22) | (1<<CS21) | */
	set_temp(50);
}

/** Set the reference temperature for the heating element as 0-100% */
void set_temp(uint8_t percent)
{
	OCR2B = 0xFF * ((float)percent/100.0);
}

/** Go to power down sleep mode
 * Disables the PCINT23 interrupt that shares the same interrupt vector as
 * the on/off switches interrupt.
 */
void power_down()
{
	/* TODO: disable ADC */
	PRR |= (1<<PRADC);	/* Shut down ADC */
	/* TODO: Analog comparator off */

	/* Go to sleep as recommended in avr/sleep.h */
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	cli();
	if (switch_power_off)	/* Check again for debounce */
	{	
		/* Sleep */
		PCMSK2 &= ~(1<<PCINT23);	/* Disable interrupt in PCINT23 pin */
		sleep_enable();
		sleep_bod_disable();
		sei();
		sleep_cpu();
		
		/* Wake up */
		sleep_disable();
		PCMSK2 |= (1<<PCINT23);		/* Re-enable interrupt in PCINT23 pin */
	}
	sei();
}

/* TODO: ISR For console */
/* TODO: ISR For potentiometer ADC conversion done */
/* TODO: ISR For heating element ADC conversion done */


/** ISR for the switches in PCINT22/23 pins 
 * PIND6/PCINT22: Power on/off, normal operation/power down.
 * PIND7/PCINT23: Temperature control from potentiometer/terminal
 */
ISR(PCINT2_vect)
{
	switch_power_off = PIND & (1<<PIND6);
	switch_temp_ext = PIND & (1<<PIND7);
}
