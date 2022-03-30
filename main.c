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

/* PID control */
#define KP 1.0
#define KD 0.5
#define KI 0.2
#define DT 1.0
#define A0 KP + KI*DT + KD/DT
#define A1 -KP - 2*KD/DT
#define A2 KD/DT

/* PWM prescaler values */
#define PRESCALE_0 (1<<CS20)
#define PRESCALE_64 (1<<CS22)
#define PRESCALE_1024 (1<<CS22) | (1<<CS21) | (1<<CS20)

/* ADC channels */
#define ADC_POT 6
#define ADC_TEMP 7

#define ADC_MAX 1023	/* 10 bit maximum value */

volatile uint8_t switch_power_off = 0;
volatile uint8_t switch_temp_ext = 0;

void setup();
void power_down();
void wake_up();
int8_t set_temp(uint8_t ref, uint8_t temp);
void read_ad(uint8_t channel, uint8_t *ad_result);
void set_led(int8_t state);

int main(void)
{
    uint8_t temp_ref = 0;
	uint8_t temp = 0;
	int8_t led_status = 0;

	setup();
	/* TODO: LED IO */
	/* TODO: Console USART */
	
	
    while (1) 
    {
		/* Check is power switch is turned off */
		if (switch_power_off)
			power_down();
		
		read_ad(ADC_POT, &temp_ref);	/* Read reference temperature */
		read_ad(ADC_TEMP, &temp);		/* Read measured temperature */
		
		led_status = set_temp(temp_ref, temp);
		set_led(led_status);
    }
}

void setup()
{	
	/* Power saving setup */
	PRR |= (1<<PRTWI);		/* Shut down TWI */
	PRR |= (1<<PRTIM2);		/* Shut down Timer/Counter2 */
	PRR |= (1<<PRTIM1);		/* Shut down Timer/Counter1 */
	PRR |= (1<<PRTIM0);		/* Shut down Timer/Counter0 */
	PRR &= ~(1<<PRADC);		/* Enable ADC */
	
	/* External interrupt for switches setup */
	sei();
	PCICR |= (1<<PCIE2);					/* Enable PCINT2 interrupt vector */
	PCMSK2 |= (1<<PCINT23) | (1<<PCINT22);	/* Enable interrupt for the pins */
	
	/* Timer/Counter0 PWM setup */
	DDRD |= (1<<PIND3);						/* Set PD3/OC2B pin as an output */
	TCCR2A |= (1<<COM2B1) | (1<<WGM20);		/* Set output modes */
	TCCR2B |= PRESCALE_0;					/* Set the prescaler */
	
	/* ADC for potentiometer setup */
	ADCSRA |= 0x07;							/* Prescaler to 128 */
	ADCSRA |= (1<<ADEN);					/* Enable the conversion */
}

/** 
 * Read ADC channel, blocks while the conversion completes. 
 * channel: ADC_POT or ADC_TEMP to select the channel.
 * ad_result: ADC result as 0-100 % of the full range of conversion.
 */
void read_ad(uint8_t channel, uint8_t *ad_result)
{
	uint16_t result;
	ADMUX = (ADMUX & 0x0F) | channel;	/* Select the channel */
	ADCSRA |= (1<<ADSC);				/* Start conversion */
	while (ADCSRA & (1<<ADSC));			/* Wait for conversion to complete */
	result = ADCL;						/* Read bits 7:0 */
	result += (uint16_t)ADCH << 8;		/* Read bits 9:8 */
	*ad_result = 100 * result / ADC_MAX;	/* Scale the result to percentage */
}

/** Set the reference temperature for the heating element as 0-100% */
int8_t set_temp(uint8_t ref, uint8_t temp)
{
	static int error[3] = {0};
	static uint8_t pwm = 0;
	error[2] = error[1];
	error[1] = error[0];
	error[0] = ref - temp;
	pwm = pwm + A0*error[0] + A1*error[1] + A2*error[2];
	
	OCR2B = pwm;
	return ref-temp;
}

void set_led(int8_t state)
{
	/* - for cooling, + for heating, 0 for ok */
}
/**
 * Go to power down sleep mode
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


/** ISR for the switches in PCINT22/23 pins 
 * PIND6/PCINT22: Power on/off, normal operation/power down.
 * PIND7/PCINT23: Temperature control from potentiometer/terminal
 */
ISR(PCINT2_vect)
{
	switch_power_off = PIND & (1<<PIND6);
	switch_temp_ext = PIND & (1<<PIND7);
}
