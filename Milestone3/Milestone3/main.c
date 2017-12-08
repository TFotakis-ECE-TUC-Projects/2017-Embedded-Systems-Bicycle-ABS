/*
*	Milestone3.c
*
*	Authors: Kritharakis Emmanuel, Fotakis Tzanis
*	Created on: 10 November 2017
*	AVR: Atmel ATMega328P
*	Created with: Atmel Studio 7
*
*	The code below uses PB1, PB2 and PD6 as PWM outputs using OCR1A, OCR1B and
*	OCR0A compare registers respectively. Also, it uses ADC0 (PC0) as input signal
*	from a potentiometer. It reads the potentiometer's position and controls each PWM's
*	duty cycle. More specifically, it linearly sets the PB1's (servo) and PD6's (LED) PWM duty
*	cycle to minimum and PB2's (servo) PWM duty cycle to maximum when the potentiometer
*	is at minimum position and the exact reverse when the potentiometer is at
*	maximum position. The whole functionality is interrupt driven, using the
*	ADC's "Conversion Completed" Interrupt.
*/

#define F_CPU 16000000UL //16MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint32_t microsFrontWheel=0;
volatile uint32_t microsRearWheel=0;
uint32_t delay=100000;
uint32_t startFrontWheel=0;
uint32_t startRearWheel=0;

// Initializes PWM signal on PD6 for the LED with no prescaling
void LEDPWMinit(){
	DDRD |= (1<<DDD6); // Set PD6 as Output for OC0A
	TCCR0A = 1<< WGM02 | 1<<WGM01 | 1<<WGM00; // Set Timer/Counter0 to Fast PWM mode of operation with TOP=OCRA, Update of OCRA at BOTTOM and TOV Flag Set on TOP
	TCCR0A |= 1<< COM0A1; // Non-Inverting Mode - Clear OC0A on compare match, set OC0A at BOTTOM
	TCCR0B = 1 << CS00; // No prescaler
	OCR0A = 0; // Set TOP value to 0
}

void MicrosTimerInit(){
	TCCR2A = 1<<WGM21; // Set Timer 2 to CTC mode, TOP = OCR2A, Immediate update of OCR2A, TOV Flag set on MAX, Normal port operation, OC2A disconnected
	//TCCR2A = 1<<COM2A1; // Non-Inverting Mode - Clear OC2A on compare match, set OC2A at BOTTOM
	TCCR2B = 1 << CS21; // Set prescaler to clk/8
	TIMSK2 = (1 << OCIE2A); // Enable CTC interrupt
	OCR2A = 20; // Set TOP value to 20
}

ISR(TIMER2_COMPA_vect){
	microsFrontWheel++;
	microsRearWheel++;
}

// Initializes PWM signal on PB1 & PB2 for front servo & back servo
void ServoPWMinit(){
	DDRB = 1<<DDB1 | 1<<DDB2; // Set PB1 & PB2 as outputs for OC1A and OC1B respectively
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //Non-Inverting mode - Set OC1A/OC1B on compare match when up-counting. Clear OC1A/OC1B on compare match when down counting.
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); // Set prescaler to clk/64, Fast PWM
	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	OCR1A=120; // Min = 120, Max = 590
	OCR1B=590; // Min = 120, Max = 590
}
// Sets the TOP register's value
void setPWM(int value){
	OCR0A = value;
	OCR1A = round(470/256*value)+120;
	OCR1B = round(470/256*(256-value))+120;
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

void Blink(){
	PORTC |= 1<<PORTC5;
	for(uint32_t i=0; i<delay;i++)
		_delay_us(10);
	PORTC = 0;
	for(uint32_t i=0; i<delay;i++)
		_delay_us(10);
}

ISR(INT0_vect){
	if(PIND & (1<<PORTD2)){
		startFrontWheel = microsFrontWheel;
	}else{
		delay = microsFrontWheel-startFrontWheel;
		microsFrontWheel=0;
	}
}

ISR(INT1_vect){
	if(PIND & (1<<PORTD3)){
		startRearWheel = microsRearWheel;
	}else{
		delay = microsRearWheel-startRearWheel;
		microsRearWheel=0;
	}
}

void PhotointerruptersInit(){
	EIMSK = 1<<INT1 | 1<<INT0; // Enable INT0 and INT1
	EICRA = 0<<ISC11 | 1<<ISC10 | 0<<ISC01 | 1<<ISC00; // Trigger INT0 and INT1 on Change
}

int main(void){
	DDRC |= 1<<PORTC5; // Set PC5 as Output
	PhotointerruptersInit();
	MicrosTimerInit();
	sei();
	while(1){
		Blink();
	}
	//LEDPWMinit();
	//ServoPWMinit();
	//ADCinit();
}