//Detect Coin, based on Dr. Jesus's code. 
//Make sure to reset the ucontroller before running this code for better result.
//Frequency is read at pin 15 of ATMEGA328P.
// This program shows how to measure the period of a signal using timer 1 free running counter.

#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define ISR_FREQ 100000L // Interrupt service routine tick is 10 us
#define OCR1_RELOAD ((F_CPU/ISR_FREQ)+1)
#define LEFT0 0b00000100 //neg
#define LEFT1 0b01000000 //pos
#define RIGHT0 0b00100000//neg
#define RIGHT1 0b00010000 //pos
#define DETECTOR 0b00000001
#define SERVO1 0b00000010
#define SERVO2 0b00000100
#define stop 0
#define forwards 1
#define backwards 2
#define right 3
#define left 4


volatile int ISR_pw=130, ISR_cnt=0, ISR_frc,ISR_pw2=150 ,ISR_SPEED=1500, STATE,flag_pickup=1;//pw2 for horzital part pw for vertical part  PB2 for DC part
unsigned int cnt = 0;

ISR(TIMER1_COMPA_vect)
{
	OCR1A = OCR1A + OCR1_RELOAD;
	
	ISR_cnt++;
	
	
	if(ISR_cnt<ISR_pw)
	{
		PORTB |= 0b000000010; // PB1=1
	}
	else
	{
		PORTB &= ~0b000000010; // PB1=0
	}
	if(ISR_cnt<ISR_pw2)
	{
  		PORTB |= 0b00000100; // PB2=1
	}
	else
	{
		PORTB &= ~0b000000100; // PB2=0
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		ISR_frc++;
	}
}

// Atomic read of TCNT1. This comes from page 93 of the ATmega328P datasheet.
unsigned int TIM16_ReadTCNT1( void )
{
	unsigned char sreg;
	unsigned int i;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	cli();
	/* Read TCNT1 into i */
	i = TCNT1;
	/* Restore global interrupt flag */
	SREG = sreg;
	return i;
}


#define PIN_PERIOD (PINB & 0b00000001)

// GetPeriod() seems to work fine for frequencies between 30Hz and 300kHz.
long int GetPeriod (int n)
{
	int i, overflow;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	overflow=0;
	TIFR1=1; // TOV1 can be cleared by writing a logic one to its bit location.  Check ATmega328P datasheet page 113.
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>5) return 0;}
	}
	overflow=0;
	TIFR1=1;
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>5) return 0;}
	}
	
	overflow=0;
	TIFR1=1;
	saved_TCNT1a=TCNT1;
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>1024) return 0;}
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>1024) return 0;}
		}
	}
	saved_TCNT1b=TCNT1;
	if(saved_TCNT1b<saved_TCNT1a) overflow--; // Added an extra overflow.  Get rid of it.

	return overflow*0x10000L+(saved_TCNT1b-saved_TCNT1a);
}

void wait_1ms(void)
{
	unsigned int saved_TCNT1;
	
	saved_TCNT1=TCNT1;
	
	while((TCNT1-saved_TCNT1)<(F_CPU/1000L)); // Wait for 1 ms to pass
}
void delay_ms (int msecs)
{	
	int ticks;
	ISR_frc=0;
	ticks=msecs/20;
	while(ISR_frc<ticks);
}
void waitms(int ms)
{
	while(ms--) wait_1ms();
}



void move_robot(int direction){
//forwards
		if(direction==forwards){
			PORTD |= LEFT1; // LEFT1=1
			PORTD |= RIGHT0; // RIGHT1=1
			PORTD &= ~LEFT0; // LEFT0=0
			PORTD &= ~RIGHT1; // RIGHT0=0
		}
		
		//backwards
		else if(direction==backwards){
			PORTD |= LEFT0; // LEFT0=1
			PORTD |= RIGHT1; // RIGHT0=1
			PORTD &= ~LEFT1; // LEFT1=0
			PORTD &= ~RIGHT0; // RIGHT1=0
		}
		
		//right turn
		else if(direction==right){
			PORTD |= LEFT1; // LEFT1=1
			PORTD |= RIGHT1; // RIGHT0=1
			PORTD &= ~LEFT0; // LEFT0=0
			PORTD &= ~RIGHT0; // RIGHT1=0
		}
		
		//left turn
		else if(direction==left){
			PORTD |= LEFT0; // LEFT0=1
			PORTD |= RIGHT0; // RIGHT1=1
			PORTD &= ~LEFT1; // LEFT1=0
			PORTD &= ~RIGHT1; // RIGHT0=0
		}
		
		//stopped
		else{
			PORTD &= ~LEFT0; // LEFT0=0
			PORTD &= ~RIGHT1; // RIGHT1=0
			PORTD &= ~LEFT1; // LEFT1=0
			PORTD &= ~RIGHT0; // RIGHT0=0
		}
		
}
void picking_up ()
{     sei();
      //if(flag_pick==1)
      //{//stop robot
	  move_robot(stop);
	  waitms(500);		
	  //go back 1 inch
	  move_robot(backwards);
	  waitms(450);
	  		
	  //stop robot so it can pick up coin
	  move_robot(stop);
      
      DDRB=0b00001110;
      waitms(1000);
      ISR_pw=130;
      waitms(1000);
      ISR_pw2=130;
      waitms(1000);
      ISR_pw2=190;
      waitms(1000);
      ISR_pw=240;// i dont know the value for vertical part;  for the bot part
      waitms(1000);
      PORTB |= 0b00001000;  //turn on DC part;
      ISR_pw=130;//    i dont know the value for vertical part; for up part
      waitms(1000);
      ISR_pw2=130;
      PORTB &= ~0b00001000;  //turn off DC part;
      waitms(2000);
      //flag_pickup=0;
      move_robot(forwards);
     // }
     // else
      //{DDRB=0b00000001;}// for the p_detect 
     // STATE=forwards;  
     cli(); 
}
unsigned int ReadChannel(unsigned char mux)
{
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // frequency prescaler
	ADMUX = mux; // channel select
	ADMUX |= (1<<REFS1) | (1<<REFS0); 
	ADCSRA |= (1<<ADSC); // Start conversion
	while ( ADCSRA & (1<<ADSC) ) ;
	ADCSRA |= (1<<ADSC); // a transformation single conversion?
	while ( ADCSRA & (1<<ADSC) );
	ADCSRA &= ~(1<<ADEN); // Disable ADC
	return ADCW;
}
 int p_detect(int channel)
 {
   unsigned int adc;
   unsigned long int v;
   //double voltage;
   
   
   
 //  DDRB |= 0x01;    // we have to change some thing on here
 //  PORTB |= 0x01;
   
   adc=ReadChannel(channel);
   v=(adc*5000L)/1023L;
  
   //printf("ADC[0]=0x%03x, %ld.%03ldV\r", adc, v/1000, v%1000);
   
 //  PORTB ^= 0x01;
   //_delay_ms(500);
   
   if(v%1000>500 || v/1000>1){
   		return 1;
   }
   else{
   		return 0;
   }
}

int main(void)
{
    
	long int count;
	int p_flag =0;
	int coin = 350; //coin is added to prev_f, if next_f is greater than prev_f+coin, then coin is detected.
	float T, prev_f, next_f;  //prev_f is the initial reading of the coin detector. next_f is the updated reading of the coin dectector. 
	//int p_flag =0;

	
	usart_init(); // Configure the usart and baudrate
	
	
	DDRD = (LEFT0|LEFT1|RIGHT0|RIGHT1);
	DDRB = (SERVO1|SERVO2); // Configure PB0 as input
	PORTB = DETECTOR; // Activate pull-up in PB1
//	DDRD  &= 0b01111111; // Configure PD7 as input
//  PORTD |= 0b10000000; // Activate pull-up in PD7
 
	TCCR1B |= _BV(CS10); // Check page 110 of ATmega328P datasheet
    TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A 
	count=GetPeriod(100);
	T=count/(F_CPU*100.0);
	prev_f=1/T;
	
	waitms(500); // Wait for putty to start
	printf("Period measurement using the free running counter of timer 1.\n"
	       "Connect signal to PB1 (pin 15).\n");
	
	while (1)
	{
		
		move_robot(forwards);
		
		
		p_flag=p_detect(0);
		printf("p_detect=%d\r\n",p_flag);
		if(p_flag==1)
        { move_robot(backwards);
        waitms(2000);
        move_robot(left);
       	waitms(1500);
       move_robot(forwards);
       }
       
     
   
		count=GetPeriod(100);
		if(count>0)
		{
			T=count/(F_CPU*100.0);
			next_f=1/T;
	        
			if(next_f > prev_f+coin)
			{	
				printf("\e[1;1H\e[2J");		//Clear screen
				printf("COIN!		\r");
				picking_up();
			}
			else
			{
			 printf("\e[1;1H\e[2J");		
			 printf("No Coin... 	\r");
			}
		}
		else
		{
			printf("NO SIGNAL                     \r");
		}
		waitms(200);
		
	}
}