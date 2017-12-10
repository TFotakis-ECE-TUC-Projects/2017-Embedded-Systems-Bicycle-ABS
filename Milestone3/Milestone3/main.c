// Todo: Modify block comments
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

//min=310 max=1450
//calculatedMin=310+(1140-1024)/2=368
//calculatedMax=1140-(1140-1024)/2=1082


*/

// ------------------------------------------ Calibration ------------------------------------------------
#define F_CPU 16000000UL	// 16 MHz Clock Frequency
#define frontServoMaxPosition 1140
#define frontServoMinPosition 310
#define rearServoMaxPosition 1140
#define rearServoMinPosition 310
// -------------------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint32_t microsFrontWheel=0;
volatile uint32_t microsRearWheel=0;
uint32_t delay=100000;
uint32_t startFrontWheel=0;
uint32_t startRearWheel=0;
int lastValue = 0;

// Todo: Check prescaler - Add comments
void ADCinit(){
	ADMUX = 1 << REFS0; // AVCC with external capacitor at AREF pin, ADC0 selected
	ADMUX |= 1<<ADLAR; // ADC Left Adjust Result to use ADCH register for 8-bit operations (ignore 2 Least Significant Bits)
	ADCSRA = 1 << ADEN; // Analog to Digital Enable
	ADCSRA |= 1<<ADATE; // Auto Trigger Enable Conversion
	ADCSRA |= 1<<ADIE; // ADC Conversion Complete Interrupt activated
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0; // Set prescaler to clk/128
	ADCSRA |= 1<<ADSC; // Start Conversions
}

// Todo: Check Comments
void MicrosTimerInit(){
	TCCR2A = 1<<WGM21; // Set Timer 2 to CTC mode, TOP = OCR2A, Immediate update of OCR2A, TOV Flag set on MAX, Normal port operation, OC2A disconnected
	TCCR2B = 1 << CS21; // Set prescaler to clk/8
	TIMSK2 = 1 << OCIE2A; // Enable CTC interrupt
	OCR2A = 20; // Set TOP value to 20
}

// Todo: Add comments
void PhotointerruptersInit(){
	EIMSK = 1<<INT1 | 1<<INT0; // Enable INT0 and INT1
	EICRA = 0<<ISC11 | 1<<ISC10 | 0<<ISC01 | 1<<ISC00; // Trigger INT0 and INT1 on Change
}

// Todo: Cleanup
// Initializes PWM signal on PB1 & PB2 for front servo & back servo
void ServoPWMinit(){
	DDRB = 1<<DDB1 | 1<<DDB2; // Set PB1 & PB2 as outputs
	// for OC1A and OC1B respectively
	//TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //Non-Inverting mode - Set OC1A/OC1B on compare match when up-counting. Clear OC1A/OC1B on compare match when down counting.
	//TCCR1B|=(1<<WGM13)|(0<<WGM12)|(1<<CS11)|(1<<CS10); // Set prescaler to clk/64, Fast PWM
	//ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	//OCR1A=120; // Min = 120, Max = 590
	//OCR1B=590; // Min = 120, Max = 590
}

// Todo: Add comments
void setServoPosition(int value, int servo){
	switch(servo){
		case 0:
			value=frontServoMaxPosition-value;
			PORTB = 1<<PORTB1;
			_delay_us(frontServoMinPosition);
			break;
		case 1:
			value=rearServoMaxPosition-value;
			PORTB = 1<<PORTB2;
			_delay_us(rearServoMinPosition);
			break;
	}
	for(int j=0; j<value;j++)
	_delay_us(4);
	PORTB = 0;
	_delay_us(4);
}

// Todo: Check Existence
// Sets the TOP register's value
void setPWM(int value){
	if(lastValue-value>10 || lastValue-value<-10){
		//OCR0A = value;
		OCR1A = round(470/256*value)+120;
		OCR1B = round(470/256*(256-value))+120;
		lastValue=value;
	}
}

// Todo: Check Existence
void Blink(){
	DDRC |= 1<<DDC5; // Set PC5 as Output
	PORTC |= 1<<PORTC5;
	for(uint32_t i=0; i<delay;i++)
	_delay_us(10);
	PORTC = 0;
	for(uint32_t i=0; i<delay;i++)
	_delay_us(10);
}

// Sets the PWM duty cycle to the value of the ADC0, when every conversion finishes
ISR (ADC_vect){
	if (lastValue!=ADCH){
		setServoPosition(ADCH, 0);
		setServoPosition(ADCH, 1);
		lastValue=ADCH;
	}
	//ADCSRA |= 1<<ADSC; // Start Conversions
	//_delay_ms(1);
}

// Todo: Add comments
ISR(TIMER2_COMPA_vect){
	microsFrontWheel++;
	microsRearWheel++;
}

// Todo: Add comments
ISR(INT0_vect){
	if(PIND & 1<<PORTD2){
		startFrontWheel = microsFrontWheel;
		}else{
		delay = microsFrontWheel-startFrontWheel;
		microsFrontWheel=0;
	}
}

// Todo: Add comments
ISR(INT1_vect){
	if(PIND & 1<<PORTD3){
		startRearWheel = microsRearWheel;
		}else{
		delay = microsRearWheel-startRearWheel;
		microsRearWheel=0;
	}
}

int main(void){
	ADCinit();
	MicrosTimerInit();
	PhotointerruptersInit();
	ServoPWMinit();
	sei();
	// Todo: Check Existence
	//ADCSRA |= 1<<ADSC; // Start Conversions
	while(1);
}