//Arm motion and picks up coin


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

#define ISR_FREQ 100000L // Interrupt service routine tick is 10 us
#define OCR1_RELOAD ((F_CPU/ISR_FREQ)+1)

volatile int ISR_pw=60, ISR_pw2=60, ISR_cnt=0, ISR_frc;

// 'Timer 1 output compare A' Interrupt Service Routine
// This ISR happens at a rate of 100kHz.  It is used
// to generate the standard hobby servo 50Hz signal with
// a pulse width of 0.6ms to 2.4ms.
void delay_ms (int msecs)
{	
	int ticks;
	ISR_frc=0;
	ticks=msecs/20;
	while(ISR_frc<ticks);
}
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

void picking_up (void)
{
      ISR_pw=135; 		 //Horiztontal, PB1
      delay_ms(500); 
      ISR_pw2=240;  	 //Vertical, PB1
      delay_ms(500);
      
      //Turn on E-mag here			
      PORTD &= (~0b00001000);
      			
      //Sweeping Motion
      ISR_pw=108;		
      delay_ms(500);
      ISR_pw=170;
       delay_ms(500);
       
      //Raise arm just above the coin holder
      //Raise it incrementally to keep heavier coins from flying off.    
      for (int i =0; i<15; i++){
       ISR_pw2-=10;
       delay_ms(100);
      }
      
      
      //Place coin above coin holder
      //Turn arm above coin holder incrementally to keep heavier coins from flying off
      for (int i =0; i<7; i++){
       ISR_pw-=10;
       delay_ms(100);
      }
      
      	delay_ms(200);  //Wait for a bit before turning off magnet
      //Turn off E-mag here
      PORTD |= 0b00001000;
      
      //Return arm to rest postion
      ISR_pw2=60;
      delay_ms(500);
      ISR_pw=60;
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


void main (void)
{
	char buf[32];
	int pw;
	int commence_motion;
	
	DDRD |= 0b00001000;//DDRB sets data direction of pin: 1 for output, 0 for input. This in is used for power switching the E-mag.
	DDRB=0b00000110; // PB1 (pin 15) and PB2 (pin16) are configured as output

	PORTD |= 0b00000000; //E-mag is in PD3. Switch it off initially.
	
	TCCR1B |= _BV(CS10);   // set prescaler to Clock/1
	TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A
	
	sei(); // enable global interupt

	usart_init(); // configure the usart and baudrate
	
	// Give putty a chance to start
	delay_ms(500); // wait 500 ms
	
	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
    printf("Servo signal generator for the ATmega328p.\r\n");
    printf("By Jesus Calvino-Fraga (c) 2018.\r\n");
    printf("Pulse width between 60 (for 0.6ms) and 240 (for 2.4ms)\r\n");
	
	while(1)
    {
    
    	//printf("\x1b[2J\x1b[1;1H"); //Clear screen.
    	printf("Commence Motion?: ");
    	usart_gets(buf, sizeof(buf)-1); // wait here until data is received
 
    	printf("\n");
	    commence_motion=atoi(buf);
	    if( commence_motion == 1 )
	    {
	    	//Activate arm Motion
	    	printf("Yes\r\n");
	    	picking_up();			//Picks up coin
        }
        else
        {
        	printf("No\r\n");
        }
        
       delay_ms(500); // wait 500 ms
    }
}