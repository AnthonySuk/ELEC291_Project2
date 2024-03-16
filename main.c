//2024.3.15 23:07
#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/serial.h"
#include <math.h>
#include "UART2.h"

#define F_CPU 32000000L
#define DEF_F 100000L // 10us tick
//#define DEF_F 15000L suspicious

volatile int PWM_Counter = 0;
volatile unsigned int pwm1=100, pwm2=100, pwm3=100,pwm4=100;
volatile float f_init;
volatile int check_spk_count = 0;
volatile int speaker_on=0;
volatile float speaker_freq = 2048;


void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM2 interrupt 
// via the interrupt vector table defined in startup.c
void TIM2_Handler(void) 
{
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	PWM_Counter++;
	
	if(pwm1>PWM_Counter)
	{
		GPIOA->ODR |= BIT11;
	}
	else
	{
		GPIOA->ODR &= ~BIT11;
	}
	
	if(pwm2>PWM_Counter)
	{
		GPIOA->ODR |= BIT12;
	}
	else
	{
		GPIOA->ODR &= ~BIT12;
	}

	if(pwm3>PWM_Counter)
	{
		GPIOA->ODR |= BIT13;
	}
	else
	{
		GPIOA->ODR &= ~BIT13;
	}

	if(pwm4>PWM_Counter)
	{
		GPIOA->ODR |= BIT14;
	}
	else
	{
		GPIOA->ODR &= ~BIT14;
	}


	
	if (PWM_Counter > 2000) // THe period is 20ms
	{
		PWM_Counter=0;
		GPIOA->ODR |= (BIT11|BIT12);
	}   
}

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
// (Spk) PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//  (TX) PA2 -|8       25|- PA15
//  (RX) PA3 -|9       24|- PA14 (pwm4)
// (SET) PA4 -|10      23|- PA13 (pwm3)
//       PA5 -|11      22|- PA12 (pwm2)
//       PA6 -|12      21|- PA11 (pwm1)
//       PA7 -|13      20|- PA10 (Reserved for RXD)
//       PB0 -|14      19|- PA9  (Reserved for TXD)
//       PB1 -|15      18|- PA8	 (ADC for Oscillator)
//       VSS -|16      17|- VDD
//             ----------

void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	// Set up output pins
	RCC->IOPENR |= BIT0; // peripheral clock enable for port A
	GPIOA->MODER = (GPIOA->MODER & 0xfffffffc) | 0x00000001; // Make pin PA0 output (page 172, two bits used to configure: bit0=1, bit1=0)
    //GPIOA->OTYPER |= BIT0; // Open-drain
    GPIOA->OTYPER &= ~BIT11; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT22|BIT23)) | BIT22; // Make pin PA11 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT11; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT24|BIT25)) | BIT24; // Make pin PA12 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT12; // Push-pull
	
	GPIOA->MODER = (GPIOA->MODER & ~(BIT26|BIT27)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT13; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT28|BIT29)) | BIT28; // Make pin PA14 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT14; // Push-pull

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16;
	GPIOA->PUPDR &= ~(BIT17);

// JDY40 config begin
	GPIOA->MODER = (GPIOA->MODER & ~(BIT9|BIT8)) | BIT8; // Make pin PA4 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT4; // 'set' pin to 1 is normal operation mode.

	//Button set on PA1 (pin 7)
	GPIOA->MODER &= ~(BIT2 | BIT3); // Make pin PA1 (7) input
	// Activate pull up for pin PA1 (7) :
	GPIOA->PUPDR |= BIT2; 
	GPIOA->PUPDR &= ~(BIT3);
// JDY40 config end

	// Set up timer
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = F_CPU/DEF_F-1;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable    
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting

	// Set up timer
	RCC->APB2ENR |= BIT2;  // turn on clock for timer21 (UM: page 188)
	TIM21->ARR = F_CPU/(2L*speaker_freq);
	NVIC->ISER[0] |= BIT20; // enable timer 21 interrupts in the NVIC
	TIM21->CR1 |= BIT4;      // Downcounting    
	TIM21->DIER |= BIT0;     // enable update event (reload event) interrupt
	TIM21->CR1 |= BIT0;      // enable counting      
	
	__enable_irq();
}

#define PIN_PERIOD (GPIOA->IDR&BIT8)

// GetPeriod() seems to work fine for frequencies between 300Hz and 600kHz.
// 'n' is used to measure the time of 'n' periods; this increases accuracy.
long int GetPeriod (int n)
{
	TIM2->CR1 &= ~BIT0;
	TIM21->CR1 &= ~BIT0;
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	unsigned char overflow;
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	overflow = 0;
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT16)overflow++; 
		if(overflow>10)return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	overflow = 0;
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT16)overflow++; 
		if(overflow>10)return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	overflow = 0;
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SysTick->CTRL & BIT16)overflow++; 
			if(overflow>10)return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SysTick->CTRL & BIT16)overflow++; 
			if(overflow>10)return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	TIM2->CR1 |= BIT0;
	TIM21->CR1 |= BIT0;
	
	return 0xffffff-SysTick->VAL;
}

float get_frequency(void){
	long int count;
	float T, f;
	//float C1 = 10e-9;
	//float C2 = 100e-9;
	//float Ct;
	//float L;
	count=GetPeriod(100);
	if(count>0)
	{
		T=count/(F_CPU*100.0); // Since we have the time of 100 periods, we need to divide by 100
		f=1.0/T;
		//printf("%d\r", count);
		//Ct=C1*C2/(C1+C2);
		//f=1/(2pi*sqrt(LCt))
		//L=1.0/(4.0 * 3.14159*3.14159 * f * f * Ct);
		//printf("f=%.2fHz, count=%d, L=%fmH            \r", f, count, L*1000.0);
		
		//////printf("f=%.2fHz, count=%d            \r", f, count);
	}
	else
	{
		//printf("NO SIGNAL                     \r");
	}
	fflush(stdout); // GCC printf wants a \n in order to send something.  If \n is not present, we fflush(stdout)


	return f;
}

float get_ave_freq(void){
	float sum=0.0;

	for(int i=0;i<20;i++){
		
		sum+=get_frequency();
	}
	return sum/20.0;
}

void check_speaker(void){
	float f = get_ave_freq();
	float diff = f-f_init;
	//printf("%f %f %f\r", f, f_init,diff);
	if(f-f_init >= 230){
		speaker_freq = diff+1000.0;
		TIM21->ARR = F_CPU/(2L*speaker_freq);
		speaker_on=1;
	}
	else{
		speaker_on=0;
	}
}

// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM2 interrupt 
// via the interrupt vector table defined in startup.s
void TIM21_Handler(void) 
{
	TIM21->SR &= ~BIT0; // clear update interrupt flag
	if(speaker_on){
		GPIOA->ODR ^= BIT0; // Toggle PA0
	}
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT4); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT4; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

int main(void)
{	
	char buff[16];
    char xbuf[16];
    char ybuf[16];
    double xpwmtest;
    double ypwmtest;
    double xpwm;	//we get the x
    double ypwm;	//we get the y
	int cnt=0;
	int j;
	int p=0;
	int flag = 0;
	int strLength;

	Hardware_Init();
	initUART2(9600);

	delayms(1000); // Give putty a chance to start before we send characters with printf()

    printf("(outputs are PA11 and PA12, pins 21 and 22).\r\n");
    printf("By Jesus Calvino-Fraga (c) 2018-2023.\r\n\r\n");
    
    f_init = get_ave_freq();
    
//JDY40 init begin
	SendATCommand("AT+DVIDABCD\r\n");  
	SendATCommand("AT+RFIDCDBA\r\n");
	//SendATCommand("AT+VER\r\n");
	//SendATCommand("AT+BAUD4\r\n");
	// SendATCommand("AT+RFID\r\n");
	// SendATCommand("AT+DVID\r\n");
	//SendATCommand("AT+RFC001\r\n");
	// SendATCommand("AT+POWE9\r\n");
	// SendATCommand("AT+CLSSA0\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	// SendATCommand("AT+DVIDABCD\r\n");  
	// SendATCommand("AT+RFIDCDBA\r\n");

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	cnt=0;
//JDY40 init end
    


	while (1)
	{
    	fflush(stdout);
    	//egets_echo(buf, 31); // wait here until data is received
  		//printf("\r\n");
  		waitms(200);
		if(ReceivedBytes2()>0) // Something has arrived
		{
			strLength = egets2(buff, sizeof(buff)-1);
		
			
			//buf[4]='\0';
			//printf("%d",xpwm);
	    	//fflush(stdout);
	    	//egets_echo(buf, 31); // wait here until data is received
	 		//printf("\r\n");
			if(strLength >= 13){
				p = 0;
				flag=0;
				
				for(int i=0;i<=15;i++){
					if(flag){
						ybuf[p]=buff[i];
						
						p++;
					}
					else{
						xbuf[i]=buff[i];
					}
					
					if(buff[i]==','){
						flag=1;
						
					}
				}
				
				xpwmtest=atof(xbuf);
				ypwmtest=atof(ybuf);
				
				//for(int i=5;i<=9;i++){
				//	ybuf[i-5]=buff[i];
				//}
				//buf[8]='\0';
				if(xpwmtest < 5){
					xpwm = xpwmtest;
				}
				if(ypwmtest < 5){
					ypwm = ypwmtest;
				}
				
			
			}
		}
		//pwm 1 and 2 controlling left wheel while pwm 3 and 4 controlling right.
		//pwm1 and pwm 3 will control moving forward
		///////////// jia y xi shu
		if(xpwm>2.37){
			//car go forward
			//printf("%lf",xpwm);
			//printf("%lf",ypwm);
			pwm2=0;
			pwm4=0;
			//moving to the right
			if(ypwm>2.43){
			pwm1=((xpwm-2.37)*500.0)+((ypwm-2.43)*400);
			pwm3=((xpwm-2.37)*500.0);//can moderate coeffcient here
			}else if(ypwm<2.43){//moving to left
			pwm3=((xpwm-2.37)*500.0)+((2.43-ypwm)*400);
			pwm1=((xpwm-2.37)*500.0);//can moderate coeffcient here
			}else{//not y compennet
				pwm3=((xpwm-2.37)*600.0);
				pwm1=((xpwm-2.37)*600.0);
			}
		}
	//no y component	}
		else if(xpwm<2.37){
			//car go back
			pwm1=0;
			pwm3=0;

			//moving to the right
			if(ypwm>2.43){
			pwm2=((2.37-xpwm)*500.0)+((ypwm-2.43)*400);
			pwm4=((2.37-xpwm)*500.0);//can moderate coeffcient here
			}else if(ypwm<2.43){//moving to left
			pwm4=((2.37-xpwm)*500.0)+((2.43-ypwm)*400);
			pwm2=((2.37-xpwm)*500.0);//can moderate coeffcient here
			}else{//not y compennet
				pwm2=((2.37-xpwm)*600.0);
				pwm4=((2.37-xpwm)*600.0);
			}




		}else{
			//car stay still or moving in y component
			if(ypwm==2.43){
				pwm1=0;
				pwm2=0;
				pwm3=0;
				pwm4=0;

			}else if(ypwm>2.43){//moving to the right
				pwm1=(ypwm-2.43)*800;
				pwm2=0;
				pwm3=0;
				pwm4=0;
			}else{
				pwm1=0;
				pwm2=0;
				pwm3=(2.43-ypwm)*800;
				pwm4=0;
			}

			
		}
		check_speaker();
		//printf("p1 %d,p2 %d,p3 %d,p4 %d\r\n",pwm1,pwm2,pwm3,pwm4);
	}
}