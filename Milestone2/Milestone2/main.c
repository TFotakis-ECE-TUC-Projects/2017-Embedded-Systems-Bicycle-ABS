#define F_CPU 16000000 //16MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Initializes PWM signal on PB6 with no prescaling
void PWMinit(){
	DDRD |= (1<<DDD6); // Set PD6 as Output for OC0A
	TCCR0A = 1<< WGM02 | 1<<WGM01 | 1<<WGM00; // Set Timer/Counter0 to Fast PWM mode of operation with TOP=OCRA, Update of OCRA at BOTTOM and TOV Flag Set on TOP
	TCCR0A |= 1<< COM0A1; // Non-Inverting Mode - Clear OC0A on compare match, set OC0A at BOTTOM
	TCCR0B = 1 << CS00; // No prescaler
	OCR0A = 0; // Set TOP value to 0
}

// Sets the TOP register's value
void setPWM(int value){
	OCR0A = value;
}

// Sets the PWM duty cycle to the value of the ADC0, when every conversion finishes
ISR (ADC_vect){
	cli(); // Clear Interrupts
	setPWM(ADCH);
	_delay_ms(1);
	sei(); // Set Interrupts
}

void ADCinit(){
	ADMUX = 1 << REFS0; // AVCC with external capacitor at AREF pin, ADC0 selected
	ADMUX |= 1<<ADLAR; // ADC Left Adjust Result to use ADCH register for 8-bit operations (ignore 2 Least Significant Bits)
	ADCSRA = 1 << ADEN; // Analog to Digital Enable
	ADCSRA |= 1<<ADATE; // Auto Trigger Enable Conversion
	ADCSRA |= 1<<ADIE; // ADC Conversion Complete Interrupt activated
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0; // Set prescaler to clk/128
	sei(); // Set Interrupts
	ADCSRA |= 1<<ADSC; // Start Conversions
	_delay_ms(1);
}

int main(void){
	PWMinit();
	ADCinit();
}