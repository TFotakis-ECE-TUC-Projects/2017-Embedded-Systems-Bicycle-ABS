#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

int smallDelay=1;
int extraTime=0;

void PortInit(void){
	DDRB = 0x0D;
	PORTB = 0x0D;
	DDRB = 1<<PORTB1 | 1<< PORTB2 | 1<<PORTB3;
	PORTB = 1<<PORTB1 | 1<< PORTB2 | 1<<PORTB3;
	_delay_ms(1500);
	PORTD = 0x04;
	EIMSK = (1<<INT0);
	EICRA = 0<<ISC01 | 0<<ISC00;	// Trigger INT0 on Low
}

void WDT_Init(void){
	// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
	MCUSR = MCUSR & 0xF7;
	// Set the WDCE bit (bit 4) and the WDE bit (bit 3)
	// of WDTCSR. The WDCE bit must be set in order to
	// change WDE or the watchdog prescalers. Setting the
	// WDCE bit will allow updates to the prescalers and
	// WDE for 4 clock cycles then it will be reset by
	// hardware.
	WDTCSR = WDTCSR | 0x18;

	// Set the watchdog timeout prescaler value to 1024 K
	// which will yield a time-out interval of about 8.0 s.
	WDTCSR = 0x21;

	// Enable the watchdog timer interrupt.
	WDTCSR = WDTCSR | 0x40;
	MCUSR = MCUSR & 0xF7;
}

void TimerCounterInit(void){
	TCCR0A = 1<<WGM01; //Sets timer counter mode of operation (CTC - Clear timer on compare)
	OCR0A = 195;  // Number of real clock ticks
	TIMSK0  = 1<<OCIE0A; // Timer/Counter0 output compare match a interrupt enable
	TCCR0B = 1<<CS02 | 1<<CS00;  // CLKio /1024 (prescaler)
}

int main(void){
	PortInit();
	WDT_Init();
	TimerCounterInit();
	sei();
    while (1){
		PORTB = (1<<PORTB1);
		if(smallDelay==1)
			_delay_ms(50);
		else
			_delay_ms(200);
		PORTB = (0<<PORTB1);
		if(smallDelay==1)
			_delay_ms(50);
		else
			_delay_ms(200);
	}
}

ISR(INT0_vect){
	if(smallDelay==1)
	smallDelay=0;
	else
	smallDelay=1;
	cli();
	wdt_reset();
	_delay_ms(500);
	sei();
}

ISR(TIMER0_COMPA_vect){
	 extraTime++;
	 
	 if(extraTime > 100)
	 {
		 extraTime = 0;
		 PORTB ^= (1 << PORTB3);
	 }
}

//ISR(WDT_vect){
	//PORTB = 1<<PORTB2;
	//_delay_ms(100);
	//PORTB = 0<<PORTB2;
//}

