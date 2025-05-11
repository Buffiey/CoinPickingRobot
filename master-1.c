#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "usart.h"
#include "Software_UART.h"

/* Pinout for DIP28 ATMega328P:

                           -------
     (PCINT14/RESET) PC6 -|1    28|- PC5 (ADC5/SCL/PCINT13)
       (PCINT16/RXD) PD0 -|2    27|- PC4 (ADC4/SDA/PCINT12)
       (PCINT17/TXD) PD1 -|3    26|- PC3 (ADC3/PCINT11)
      (PCINT18/INT0) PD2 -|4    25|- PC2 (ADC2/PCINT10)
 (PCINT19/OC2B/INT1) PD3 -|5    24|- PC1 (ADC1/PCINT9)
    (PCINT20/XCK/T0) PD4 -|6    23|- PC0 (ADC0/PCINT8)
                     VCC -|7    22|- GND
                     GND -|8    21|- AREF
(PCINT6/XTAL1/TOSC1) PB6 -|9    20|- AVCC
(PCINT7/XTAL2/TOSC2) PB7 -|10   19|- PB5 (SCK/PCINT5)
   (PCINT21/OC0B/T1) PD5 -|11   18|- PB4 (MISO/PCINT4)
 (PCINT22/OC0A/AIN0) PD6 -|12   17|- PB3 (MOSI/OC2A/PCINT3)
      (PCINT23/AIN1) PD7 -|13   16|- PB2 (SS/OC1B/PCINT2)
  (PCINT0/CLKO/ICP1) PB0 -|14   15|- PB1 (OC1A/PCINT1)
                           -------
*/
/*
stagnant x 330
stagnant y 390
y 678 left x 330
y 21 left x 330
x 670 y 390 down
x 0 y 390 up  

ntn 39473
5c 41095
10c 40540
25c 41379(maybe)-41666
1$ 41379-41095
2$  417

*/
#define DEF_FREQ 15000L
#include <stdint.h>

#include "lcd.h"
#include "lcd.c"
/* Pinout remains the same as provided */

#define DEF_FREQ 15000L
#define PRESCALER 64
#define OCR2_RELOAD ((F_CPU/(PRESCALER*2*DEF_FREQ))+1)

volatile uint8_t reload; // 8-bit reload value for Timer2

// Timer2 Compare A Interrupt Service Routine



// Timer2 Compare A Interrupt Service Routine
ISR(TIMER2_COMPA_vect)
{
//	TIMSK2 = (1 << OCIE2A); // i.e. TIMSK2 = 0x02
    OCR2A += reload; // Update OCR2A with 8-bit addition
    PORTC ^= 0b00100000; // Toggle PB0 and PB1
}

// 'Timer 1 output compare A' Interrupt Service Routine


void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	PORTD &= ~(BIT4); // 'set' pin to 0 is 'AT' mode.
	_delay_ms(10);
	SendString1(s);
	GetString1(buff, 40);
	PORTD |= BIT4; // 'set' pin to 1 is normal operation mode.
	_delay_ms(10);
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	PORTD &= ~(BIT4); // 'set' pin to 0 is 'AT' mode.
	_delay_ms(10);
	SendString1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing from the radio.
	_delay_ms(10);
	PORTD |= BIT4; // 'set' pin to 1 is normal operation mode.
}
void initUART(void)
{
	// Not necessary; initialize anyway
	DDRD |= _BV(PD1);
	DDRD &= ~_BV(PD0);

	// Set baud rate; lower byte and top nibble
	UBRR0H = ((_UBRR) & 0xF00);
	UBRR0L = (uint8_t) ((_UBRR) & 0xFF);

	TX_START();
	RX_START();

	// Set frame format = 8-N-1
	UCSR0C = (_DATA << UCSZ00);
	//UCSR0A bits: RXC0 TXC0 UDRE0 FE0 DOR0 UPE0 U2X0 MPCM0
	UCSR0A|=0x02; // U2X0 double speed
}

uint8_t getByte(void)
{
	// Check to see if something was received
	while (!(UCSR0A & _BV(RXC0)));
	return (uint8_t) UDR0;
}

void putByte(unsigned char data)
{
	// Stay here until data buffer is empty
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = (unsigned char) data;
}

void writeString(char *str)
{
	while (*str != '\0')
	{
		putByte(*str);
		++str;
	}
}

// delay functions
unsigned int cnt = 0;

void wait_1ms(void)
{
	unsigned int saved_TCNT1;
	
	saved_TCNT1=TCNT1;
	
	while((TCNT1-saved_TCNT1)<(F_CPU/1000L)); // Wait for 1 ms to pass
}

void waitms(int ms)
{
	while(ms--) wait_1ms();
}

void adc_init(void)
{
    ADMUX = (1<<REFS0); //Sets to 10 bit ADC, using AVCC as reference voltage
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(int channel)
{
    channel &= 0x7;
    ADMUX = (ADMUX & 0xf8)|channel;
     
    ADCSRA |= (1<<ADSC);
     
    while(ADCSRA & (1<<ADSC)); //as long as ADSC pin is 1 just wait.
     
    return (ADCW);
}

void PrintNumber(long int N, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (N>0) | (digits>0) )
	{
		buff[j--]=HexDigit[N%Base];
		N/=Base;
		if(digits!=0) digits--;
	}
	usart_pstr(&buff[j+1]);
}

void Configure_Pins(void)
{	
	DDRB|=0b00000001; // PB0 is output.

	DDRB  &= 0b11111101; // Configure PB1 as input
	PORTB |= 0b00000010; // Activate pull-up in PB1
	
	DDRD  &= 0b11111011; // Configure PB1 as input
	PORTD |= 0b00000100; // Activate pull-up in PB1
	
	DDRD|=0b11111000; //  PD3, PD4, PD5, PD6, and PD7 are outputs.
	PORTD |= 0x02; // PD2 as speaker output
	
	DDRB  &= 0b11111011; // Configure PB2 as input
	PORTB |= 0b00000100; // Activate pull-up in PB2

	DDRC  &= 0b11111101; // PC1 as input
	PORTC |= 0b00000010; // pullup in PC1
	
	DDRB|=0b00000001; // PB0 is output.
	DDRD|=0b11110000; // PD3, PD4, PD5, PD6, and PD7 are outputs.
//	DDRC|=0b00011000; // PD3, PD4, PD5, PD6, and PD7 are outputs.
	
	DDRC|=0b00111000;
	
		// Configure PB1 for input.  Information here:
	// http://www.elecrom.com/avr-tutorial-2-avr-input-output/
	DDRD  &= 0b11111011; 
	PORTD |= 0b00000100; 
//	DDRD  &= 0b11110111; 
//	PORTD |= 0b00001000; 
	
   
    PORTC |= 0b00100000; // Initialize PB0 high, PB1 low

}
void ConfigurePins(void){
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDDCAB\r\n");
	SendATCommand("AT+RFC117\r\n");

}



void main(void)
{

	unsigned int adcX, adcY;
	int offsetX, offsetY;
	unsigned long int vX, vY;

	char buff[80];
	
	
	char sendX;
	char sendY;
	char radbuff[80];
    int timeout_cnt=0;
    int cont1=0, cont2=100;
    
	float frequency = 0;
	float period;
	int check = 0;
	int pickcoin = 0;
	char buffc[32];
	//LCD_4BIT();
	
	TCCR2B |= _BV(CS22) | _BV(CS21); // Prescaler 64
    TIMSK2 |= _BV(OCIE2A); // Enable Timer2 Compare A interrupt
	
	sei();
	usart_init (); // configure the usart and baudrate

	adc_init();
	Configure_Pins(); // configure pins as inputs & outputs
	LCD_4BIT(); // initialize LCD
	Init_Software_Uart(); // Configure the sorftware UART

	// Turn on timer with no prescaler on the clock.  We use it for delays and to measure period.
	TCCR1B |= _BV(CS10); // Check page 110 of ATmega328P datasheet
	TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A
	
	// TIMSK2 = 0;
	
	 char buffspeak[32];
    unsigned long newF;
    
    reload = OCR2_RELOAD;

  //  DDRC = 0b00110000; // Set PB0 and PB1 as outputs
    PORTD |= 0b0001000; // Initialize PB0 high, PB1 low


    _delay_ms(500);
    
    
	waitms(500); // Wait for putty to start   // configure the hardware usart and baudrate

	_delay_ms(500); // Give putty a chance to start before we send information...
	printf("\r\nATMega328 JDY-40 Master Test\r\n");
	
	ReceptionOff();

	// To check configuration
	ConfigurePins();
//	reload=(F_CPU/(2L*100))+1; 
//	LCDprint("hello world",2,1);
	
	offsetX=0;
	offsetY=0;
	 
	while (1) // forever loop
	{
		
	
		if((PIND&0b00000100)==0){
      check = !check;
    }
    
	  newF = atol(buff)/10;
	  period = (1.0/newF)*1000000;
	  printf("%f",period);

 	if(  newF > 4300){
     TIMSK2 |= _BV(OCIE2A);
      
      }
     if(  newF < 4300 ){
     TIMSK2 = 0;
      
      }
      
        if(newF > (F_CPU / (PRESCALER * 2 * 1)))
        {
   
        }
        if(newF > 0)
        {
            uint32_t calc_reload = (F_CPU / (PRESCALER * 2UL * newF)) + 1;
            if (calc_reload > 255)
            {
                calc_reload = 255;
              
            }
            reload = (uint8_t)calc_reload;

       }
   
 

		adcX=adc_read(0);
		adcX=adcX-offsetX;
		vX=(adcX*5000L)/1023L;

		
		adcY=adc_read(1);
		adcY=adcY-offsetY;
		vY=(adcY*5000L)/1023L;

		 pickcoin = (PIND&0b00001000)?1:0;
		if(adcX <20){
			sendX = 'w';
		}
		else if(adcX >650){
			sendX = 's';
		}
		
		else if(adcY < 41){
			sendX = 'd';
		}
		else if(adcY >668){
			sendX = 'a';
		}
		else{
		if(pickcoin == 0){
     	 sendX ='f';
     	// waitms(1250);

     	}
     	else{
		sendX = 'n';
		}
		}
		
		if(check){
		sendX = 'z';
		
		}
		if(buff[0] == 'k'){
		check = 0;
		ConfigurePins();
		}
		
		
	
	
	sprintf(buff, "%c\n", sendX); // Construct a test message
		SendByte1('!'); // Send a message to the slave. First send the 'attention' character which is '!'
		// Wait a bit so the slave has a chance to get ready
		_delay_ms(5); // This may need adjustment depending on how busy is the slave
		SendString1(buff); // Send the test message
		
		if(++cont1>200) cont1=0; // Increment test counters for next message
		if(++cont2>200) cont2=0;
		
		_delay_ms(5); // This may need adjustment depending on how busy is the slave

		SendByte1('@'); // Request a message from the slave
		if(  newF > 4300){
    	 TIMSK2 |= _BV(OCIE2A);
      
      }
     	if(  newF < 4300){
     	TIMSK2 = 0;
      
      }
		
		timeout_cnt=0;
		while(1)
		{
			if(RXD_FLAG) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 15ms for the repply
			_delay_us(100); // 100us*250=25ms
		}
		
		if(RXD_FLAG) // Something has arrived from the slave
		{
			

			GetString1_timeout(buff, sizeof(buff)-1);
			if(strlen(buff)<=80) // Check for valid message size (5 characters)
			{
				printf("Slave says: %s\r\n", buff);
			
				LCDprint(buff,2,1);
			//	LCDprint("MAGNET DETECTOR",1,1);
			
			}
			else
			{
				printf("*** BAD MESSAGE ***: %s\r\n", 1/newF);
			}
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE\r\n", buff);
		}
		
		_delay_ms(5);  // Set the information interchange pace: communicate about every 50ms
	}

}
