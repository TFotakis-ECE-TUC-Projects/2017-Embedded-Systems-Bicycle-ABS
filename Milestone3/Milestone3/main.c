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

//min=310 us
//max=1450 us
//calculatedMin=310+(1140-1024)/2=368 us
//calculatedMax=1140-(1140-1024)/2=1082 us
*/

// ------------------------------------------ Calibration ------------------------------------------------
#define F_CPU 16000000UL	// 16 MHz Clock Frequency
#define frontWheelServoPort PORTB1
#define rearWheelServoPort PORTB2

#define frontServoMaxPosition 256
#define frontServoMinPosition 310
#define rearServoMaxPosition 256
#define rearServoMinPosition 310

#define wheelsPeriodDifferenceThreshold 0
// -------------------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint32_t microsFrontWheel=0;
volatile uint32_t microsRearWheel=0;
uint32_t delay=100000;
uint32_t startFrontWheel=0;
uint32_t startRearWheel=0;
int lastSliderPosition = 0;
uint32_t frontWheelPeriod = -1;
uint32_t rearWheelPeriod = -1;

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
void PhotoInterruptersInit(){
	EIMSK = 1<<INT1 | 1<<INT0; // Enable INT0 and INT1
	EICRA = 0<<ISC11 | 1<<ISC10 | 0<<ISC01 | 1<<ISC00; // Trigger INT0 and INT1 on Change
}

// Initializes PWM signal on PB1 & PB2 for front servo & back servo
void ServoPWMinit(){
	DDRB = 1<<DDB1 | 1<<DDB2; // Set PB1 & PB2 as outputs
}

int lastRetVal=0;
int checkWheelsFrequencies(){
	if (frontWheelPeriod == -1 || rearWheelPeriod == -1) return lastRetVal;
	int32_t difference = frontWheelPeriod-rearWheelPeriod;
	frontWheelPeriod=-1;
	rearWheelPeriod=-1;
	int retVal = 0;
	if(difference>wheelsPeriodDifferenceThreshold) retVal = 1;
	else if (difference<-wheelsPeriodDifferenceThreshold) retVal = -1;
	lastRetVal=retVal;
	return retVal;
}

// Sets the TOP register's value
// MinValue=0, MaxValue=235
void setPWM(int value, int port){
	if(value>235) value=235;
	else if(value<0) value=0;
	PORTB = 1<<port;
	_delay_us(500); // Min 500 - Max 2380
	for(int i=0; i<value;i++) _delay_us(8);
	PORTB = 0;
}

// Todo: Add comments
void setServoPosition(int value){
	int checkWheelsFrequenciesValue = checkWheelsFrequencies();
	uint8_t tmpValue=value;
	//tmpValue = tmpValue>127?127:tmpValue;
	tmpValue = checkWheelsFrequenciesValue == 1?0:tmpValue;
	setPWM(tmpValue, frontWheelServoPort);
			
	tmpValue=value;
	//tmpValue*=2;
	//tmpValue = tmpValue>250?250:tmpValue;
	tmpValue = checkWheelsFrequenciesValue == -1?0:tmpValue;
	setPWM(tmpValue, rearWheelServoPort);
}

// Sets the PWM duty cycle to the value of the ADC0, when every conversion finishes
ISR (ADC_vect){
	//if (lastSliderPosition==ADCH) return;
	//setServoPosition(256-ADCH);
	//setServoPosition(256);
	//for(int i=0; i<value;i++) _delay_us(8);
	// 256-ADCH-128
	int value = 128 - ADCH;
	setServoPosition(value);
	//lastSliderPosition=ADCH;
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
		frontWheelPeriod = microsFrontWheel-startFrontWheel;
		microsFrontWheel=0;
	}
}

// Todo: Add comments
ISR(INT1_vect){
	if(PIND & 1<<PORTD3){
		startRearWheel = microsRearWheel;
	}else{
		rearWheelPeriod = microsRearWheel-startRearWheel;
		microsRearWheel=0;
	}
}

int main(void){
	ADCinit();
	MicrosTimerInit();
	PhotoInterruptersInit();
	ServoPWMinit();
	sei();
	while(1);
}