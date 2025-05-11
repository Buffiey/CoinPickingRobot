
#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#define SYSCLK 72000000
#define BAUDRATE 115200L
#define MAGNET  P1_7
#define MOTOR_R1   P2_1
#define MOTOR_R2   P2_2
#define MOTOR_L1   P2_3
#define MOTOR_L2   P2_4
#define SHOULDER_SERVO   P2_5
#define ELBOW_SERVO   P2_6
#define DETECT_METAL P0_2
#define DETECT_Wire1 P1_1
#define DETECT_Wire2 P1_2
#define RELOAD_10MS (0x10000L-(SYSCLK/(12L*100L)))
#define SARCLK 18000000L
#define A 3800L

volatile unsigned int elbow_reload;
volatile unsigned int shoulder_reload;
volatile unsigned char pwm_state4=0;
volatile unsigned char pwm_state5=0;
volatile unsigned char count20ms;
volatile unsigned int overflow_count;
volatile unsigned int start_period;
unsigned int select;
unsigned int motor_r_select;
unsigned int motor_l_select;
unsigned int magnet_select;
unsigned int servo_select;
unsigned int test_servo_flag=1;
unsigned int magnet_detector_flag=1;
unsigned int perimeter_detector_flag=1;
unsigned int coins_picked=0;
float pulse_width;
float period, frequency;
unsigned int i;
unsigned int j;

idata char buff[20];
idata char buff3[20];

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	P0MDOUT |= 0b_1100_0000; 
	P2MDOUT |= 0b_0110_0000; 
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x41; // Enable crossbar and uart 1

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  
  	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;
	TMR5RL=RELOAD_10MS-0x10000L-(SYSCLK*0.5*1.0e-3)/12.0; // intialize the elbow servo to 0.5ms
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)

	// Initialize timer 4 for periodic interrupts
	SFRPAGE=0x10;
	TMR4CN0=0x00;
	TMR4RL=RELOAD_10MS-0x10000L-(SYSCLK*0.9*1.0e-3)/12.0; // intialize the shoulder servo to 0.9ms
	TMR4=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_0100; // Enable Timer4 interrupts
	TR4=1;         // Start Timer4 (TMR4CN0 is bit addressable)

	EA=1; 
	
	SFRPAGE=0x00;
	
  	P2_0=1; // 'set' pin to 1 is normal operation mode.

	return 0;
}

void Timer4_ISR (void) interrupt INTERRUPT_TIMER4 // ISR for the elbow servo
{
	SFRPAGE=0x10;
	TF4H = 0; // Clear Timer4 interrupt flag
	// Since the maximum time we can achieve with this timer in the
	// configuration above is about 10ms, implement a simple state
	// machine to produce the required 20ms period.
	switch (pwm_state4)
	{
		case 0:
			ELBOW_SERVO=1;
			TMR4RL=RELOAD_10MS;
			pwm_state4=1;
			count20ms++;
		break;
		case 1:
			ELBOW_SERVO=0;
			TMR4RL=RELOAD_10MS-elbow_reload;
			pwm_state4=2;
		break;
		default:
			ELBOW_SERVO=0;
			TMR4RL=elbow_reload;
			pwm_state4=0;
		break;
	}
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5 // ISR for the shoulder servo
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	switch (pwm_state5)
	{
		case 0:
		SHOULDER_SERVO=1;
			TMR5RL=RELOAD_10MS;
			pwm_state5=1;
			count20ms++;
		break;
		case 1:
		SHOULDER_SERVO=0;
			TMR5RL=RELOAD_10MS-shoulder_reload;
			pwm_state5=2;
		break;
		default:
		SHOULDER_SERVO=0;
			TMR5RL=shoulder_reload;
			pwm_state5=0;
		break;
	}
}


// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void TIMER0_Init(void)
{
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	TR0=0; // Stop Timer/Counter 0
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}





void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s, unsigned char n)
{
	char c;
	unsigned char cnt;
	
	cnt=0;
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		
		if (cnt<n)
		{
			cnt++;
			*s=c;
			s++;
		}
		else
		{
			*s=0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff)-1);
	waitms(10);
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	P2_0=1; // 'set' pin to 1 is normal operation mode.
}

void CheckConfiguration(void) {

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

void PickCoin(void){
	P0_7=1; 
	P0_6=1;	
	shoulder_reload=0x10000L-(SYSCLK*1.6*1.0e-3)/12.0; 
	elbow_reload=0x10000L-(SYSCLK*2.2*1.0e-3)/12.0;
	MAGNET=1;
	waitms(250);
	shoulder_reload=0x10000L-(SYSCLK*2.3*1.0e-3)/12.0; 
	waitms(250);
	for(i=1; i<17; i++){
		elbow_reload=0x10000L-(SYSCLK*(2.2-(0.1*i))*1.0e-3)/12.0;
		waitms(25);
	}
	waitms(250);
	for(i=1;i<12;i++){
		shoulder_reload=0x10000L-(SYSCLK*(2.3-(0.1*i))*1.0e-3)/12.0;
		waitms(25);
	}
	elbow_reload=0x10000L-(SYSCLK*1*1.0e-3)/12.0;
	waitms(500);
	MAGNET=0;
	elbow_reload=0x10000L-(SYSCLK*0.5*1.0e-3)/12.0;
	P0_7=0; 
	P0_6=0;	
	waitms(50);
	coins_picked++;
	
}

void PickCoinManual(void){
	P0_7=1; 
	P0_6=1;	
	shoulder_reload=0x10000L-(SYSCLK*1.6*1.0e-3)/12.0; 
	elbow_reload=0x10000L-(SYSCLK*2.2*1.0e-3)/12.0;
	MAGNET=1;
	waitms(250);
	shoulder_reload=0x10000L-(SYSCLK*2.3*1.0e-3)/12.0; 
	waitms(250);
	for(i=1; i<17; i++){
		elbow_reload=0x10000L-(SYSCLK*(2.2-(0.1*i))*1.0e-3)/12.0;
		waitms(25);
	}
	waitms(250);
	for(i=1;i<12;i++){
		shoulder_reload=0x10000L-(SYSCLK*(2.3-(0.1*i))*1.0e-3)/12.0;
		waitms(25);
	}
	elbow_reload=0x10000L-(SYSCLK*1*1.0e-3)/12.0;
	waitms(500);
	MAGNET=0;
	elbow_reload=0x10000L-(SYSCLK*0.5*1.0e-3)/12.0;
	coins_picked++;
	P0_7=0; 
	P0_6=0;	
	waitms(50);
	CheckConfiguration();
}

void PlaceCoin(void){
	shoulder_reload=0x10000L-(SYSCLK*1.1*1.0e-3)/12.0; 
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*1.8*1.0e-3)/12.0;
	MAGNET=1;
	waitms(250);
	shoulder_reload=0x10000L-(SYSCLK*0.9*1.0e-3)/12.0;
	waitms(500);
	shoulder_reload=0x10000L-(SYSCLK*1.1*1.0e-3)/12.0;
	waitms(1000);
	elbow_reload=0x10000L-(SYSCLK*1.6*1.0e-3)/12.0;
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*1.5*1.0e-3)/12.0;
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*1.4*1.0e-3)/12.0;
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*0.8*1.0e-3)/12.0;
	waitms(250);
	shoulder_reload=0x10000L-(SYSCLK*1.6*1.0e-3)/12.0;
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*1.7*1.0e-3)/12.0;
	waitms(250);
	MOTOR_R1=1;
	MOTOR_R2=0;
	MOTOR_L1=0;
	MOTOR_L2=1;
	waitms(1000);
	MOTOR_R1=0;
	MOTOR_R2=0;
	MOTOR_L1=0;
	MOTOR_L2=0;
	waitms(250);
	shoulder_reload=0x10000L-(SYSCLK*2.4*1.0e-3)/12.0; 
	waitms(100);
	MAGNET=0;
	waitms(250);
	elbow_reload=0x10000L-(SYSCLK*0.5*1.0e-3)/12.0;
	waitms(250);
	
	shoulder_reload=0x10000L-(SYSCLK*1.1*1.0e-3)/12.0;
	CheckConfiguration();
}

	

float GetPeriod(void){
	// Reset the counter
	TL0=0; 
	TH0=0;
	TF0=0;
	overflow_count=0;
	
	while(DETECT_METAL!=0); // Wait for the signal to be zero
	while(DETECT_METAL!=1); // Wait for the signal to be one
	TR0=1; // Start the timer
	while(DETECT_METAL!=0) // Wait for the signal to be zero
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
		}
	}
	while(DETECT_METAL!=1) // Wait for the signal to be one
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
		}
	}
	TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period!
	period=(overflow_count*65536.0+TH0*256.0+TL0)*(12.0/SYSCLK);
	return period;
}
void InitServo(void){
	elbow_reload=0x10000L-(SYSCLK*0.5*1.0e-3)/12.0; // intialize the elbow servo to 0.5ms
	waitms(500);
	shoulder_reload=0x10000L-(SYSCLK*1.1*1.0e-3)/12.0; // intialize the shoulder servo to 1.1ms
}
void stop_moving(void){
	MOTOR_R1=0;
	MOTOR_R2=0;
	MOTOR_L1=0;
	MOTOR_L2=0;
}

void start_moving(void){
	MOTOR_R1=1;
	MOTOR_R2=0;
	MOTOR_L1=1;
	MOTOR_L2=0;
}

void reverse_moving(void){
	MOTOR_R1=0;
	MOTOR_R2=1;
	MOTOR_L1=0;
	MOTOR_L2=1;
}

void move_back_a_lil(void){
	MOTOR_R1=0;
	MOTOR_R2=1;
	MOTOR_L1=0;
	MOTOR_L2=1;
	waitms(200); // change this value
	stop_moving();
}



void WaitWhileDetectingMagnet(int time){ // to check for a magnet while turning
	for(j=0; j<time/50; j++){ // every 50ms check for coin and pick it up if its there
		DetectAndPickWhileSpinning();
		waitms(50);
	}
}

void move_back_a_lil_more(void){
	MOTOR_R1=0;
	MOTOR_R2=1;
	MOTOR_L1=0;
	MOTOR_L2=1;
	WaitWhileDetectingMagnet(400); // change this value
	stop_moving();
}

void SwitchDirection(int direction){
	move_back_a_lil_more();
	if(direction==1){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(600);
	}
	else if(direction==2){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(550);
	}
	else if(direction==3){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(750);
	}
	else if(direction==4){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(1000);
	}
	else if(direction==5){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(350);
	}
	else if(direction==6){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(300);
	}
	else if(direction==7){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(500);
	}
	else if(direction==8){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(550);
	}
	else if(direction==9){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(600);
	}
	else if(direction==10){
		MOTOR_R1=1;
		MOTOR_R2=0;
		MOTOR_L1=0;
		MOTOR_L2=1;
		WaitWhileDetectingMagnet(700);
	}
	start_moving();
}

unsigned int DetectPerimeter(void){
	//printf("%d, %d\n\r", ADC_at_Pin(QFP32_MUX_P1_1), ADC_at_Pin(QFP32_MUX_P1_2));
	if((ADC_at_Pin(QFP32_MUX_P1_1)>50) || (ADC_at_Pin(QFP32_MUX_P1_2)>50)){ // change parameters here
		return 1;
	}
	else{
		return 0;
	}
	
}
void DetectAndPick(void){
	period=GetPeriod()*1000000.0; // multiply period by 10^6 cuz its kinda small

	if(start_period-period>0.0255){
		waitms(2);
		period=GetPeriod()*1000000.0; // multiply period by 10^6 cuz its kinda small
		
		if(start_period-period>0.0255){
				stop_moving(); // stop moving if metal is detected
				waitms(50);
				move_back_a_lil(); // move so coin is in the picking area
				waitms(50);
				PickCoin(); // pick coin if metal is detected
				waitms(50);
				start_moving(); // start moving again
			}
		}

}

void DetectAndPickWhileSpinning(void){
	period=GetPeriod()*1000000.0; // multiply period by 10^6 cuz its kinda small

	if(start_period-period>0.0255){
		waitms(2);
			period=GetPeriod()*1000000.0; // multiply period by 10^6 cuz its kinda small
			if(start_period-period>0.0255){
				stop_moving(); // stop moving if metal is detected
				waitms(50);
				move_back_a_lil(); // move so coin is in the picking area
				waitms(50);
				PickCoin(); // pick coin if metal is detected
				waitms(50);
				start_moving(); // start moving again
			}
		}
}

void VictoryDance(void){
	for(i=0; i<10; i++){
		reverse_moving();
		waitms(100);
		start_moving();
		waitms(100);	
	}
	stop_moving();
}

void main (void)
{
    unsigned int cnt=0;
    char c;
    char dirX;
    
    count20ms = 0;
	MAGNET = 0;
	P0_6=0;
	P0_7=0;
	stop_moving();
	
	TIMER0_Init();
	
	waitms(500);
	printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	UART1_Init(9600);
	InitPinADC(1, 1); // Configure P1.1 as analog input
	InitPinADC(1, 2); // Configure P1.2 as analog input
	InitADC(); 
	InitServo();
    waitms(2000); // Wait a second to give PuTTy a chance to start
    
	printf("\x1b[2J\x1b[1;1H"); 

	ReceptionOff();

	// To check configuration
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
	
	cnt=0;
	start_period=GetPeriod()*1000000.0;
	while(1)
	{		
		if(RXU1()) // Something has arrived
		{
			c=getchar1();
			
			if(c=='!') // Master is sending message
			{
				getstr1(buff, sizeof(buff)-1);
				if(strlen(buff)<=3)
				{
				//	printf("Master Says: %s\r\n", buff);
					
					dirX = buff[0];
					
					//printf("%c\n", dirX);
					
				
					
					if (dirX == 'w') {
					MOTOR_R1=1;
					MOTOR_R2=0;	
					MOTOR_L1=1;
					MOTOR_L2=0;
					P0_6=0;
					P0_7=0;
					}
					
					if (dirX == 'a') {
					MOTOR_R1=1;
					MOTOR_R2=0;	
					MOTOR_L1=0;
					MOTOR_L2=1;
					P0_6=1;
					P0_7=0;
					}
					
					if (dirX == 's') {
					MOTOR_R1=0;
					MOTOR_R2=1;	
					MOTOR_L1=0;
					MOTOR_L2=1;
					P0_6=1;
					P0_7=1;
					}
					
					if (dirX == 'd') {
					MOTOR_R1=0;
					MOTOR_R2=1;	
					MOTOR_L1=1;
					MOTOR_L2=0;
					P0_6=0;
					P0_7=1;
					}
					
					if (dirX == 'n') {
					MOTOR_R1=0;
					MOTOR_R2=0;	
					MOTOR_L1=0;
					MOTOR_L2=0;
					P0_6=0;
					P0_7=0;
					}
					//code for pic coin
					if (dirX == 'f') {
					PickCoinManual();	
					}
					if( dirX == 't'){
						PlaceCoin();
					}
					
					if(dirX == 'z'){ // automatic routine
						start_moving();
						waitms(5);
						DetectAndPick(); // check for coins and pick them up
						waitms(5);
						if(DetectPerimeter()){ // check if we reached the perimeter
						SwitchDirection((rand() % 10) + 1); // random turn
						}
						if(coins_picked==20){ // if we picked up 20 coins, stop moving
						coins_picked=0;
						VictoryDance();
						stop_moving();
						waitms(50);
						sprintf(buff3, "k");
						waitms(5); // The radio seems to need this delay...
						sendstr1(buff3);
						waitms(250);
						CheckConfiguration();
						}
						waitms(5);
					}	
					
				}
						
			}
			
			if(c=='@') // Master wants slave data
			{
				frequency = 1/ GetPeriod();
				sprintf(buff3, "%f\n", frequency);
				cnt++;
				waitms(5); // The radio seems to need this delay...
				sendstr1(buff3);
				
			}
	
			
		}
	}
}