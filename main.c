//-----------------------------------------------------------------------------
// ***********************************LCR METER********************************
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//********************SHIKHAR TANDON(1001397516)***************************

#include <stdint.h>
#include <stdbool.h>
#include<string.h>
#include<strings.h>
#include<stdio.h>
#include<ctype.h>
#include "tm4c123gh6pm.h"
//#include "graphics_lcd.h"



#define VCC 3.3
#define VCE_SAT 0.021
#define RESOLUTION 4096.0//resolution
#define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))//PF3 GREEN LED

#define LOWSIDE_R      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))//PA3
#define INTEGRATE      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))//PA2
#define MEAS_C         (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))//PE0

#define MEAS_LR        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))//PA5
#define HIGHSIDE_R     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))//PE4



#define MAX_CHARS 80
int count;
int count1,i,j;
int flag,caution,probe;
double inductance,capacitance,resistance;
char* str1;
char str[MAX_CHARS];
char str_del[MAX_CHARS];
int field_offset[20];
char field_type[20];
char* getString();
void ParseString(char*,int[],int);
bool isCommand(char*,int);
bool isResistance(char*,int);
bool isCapacitance(char*,int);
bool first;
uint16_t readAdc0Ss3();
void emptyString();
float atof(char*);

void val_of_RC_const();
void val_of_voltage();
void val_of_filtered_voltage();
void val_of_inductor();
void val_of_esr();
void val_of_capacitor();
void val_of_resistor();
extern void ResetISR(void);




uint32_t frequency = 0;
float time = 0;
bool timeUpdate = false;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void setTimerMode()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer 32/64-Bit Wide General-Purpose
													// Timer 5 Run Mode Clock Gating
													// Control
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    //WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)
}


void WideTimer5Isr()
   {
   	//if (timeMode)

   		time = WTIMER5_TAV_R;                        // read counter input
   	    WTIMER5_TAV_R = 0;                           // zero counter for next edge
   	    time /= 40;                                  // scale to us units
   		timeUpdate = true;                           // set update flag
   		GREEN_LED ^= 1;                              // status

   	WTIMER5_ICR_R = TIMER_ICR_CAECINT;  // clear interrupt flag

   }




// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,C,D,E,F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOB;

    // Configure GREEN LED
    GPIO_PORTF_DIR_R |= 0x08;  // make bit 1 an outputs
    GPIO_PORTF_DR2R_R |= 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x08;  // enable LED



    // Configure UART0 pins
    	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
        GPIO_PORTA_DIR_R |= 3;//0x38;
    	GPIO_PORTA_DEN_R |= 3;//0x3B;                           // default, added for clarity
    	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
        GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

         //PORTA 2,3,5
              GPIO_PORTA_DIR_R |= 0x2C;//0x38;
              GPIO_PORTA_DEN_R |= 0x2C;
              GPIO_PORTA_DR2R_R|= 0x2C;

    //CONFIGURE PE0,PE4(PE4-> RESET for LCD )
    GPIO_PORTE_DIR_R |= 0x11;
    GPIO_PORTE_DEN_R |= 0x11;
    GPIO_PORTE_DR2R_R |= 0x11;

    // Configure A0 and ~CS for graphics LCD
       GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
       GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
       GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

       // Configure SSI2 pins for SPI configuration (FOR LCD)
       SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
       GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
       GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
       GPIO_PORTB_AFSEL_R |= 0x90;                      // select alternative functions for MOSI, SCLK pins
       GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
       GPIO_PORTB_DEN_R |= 0x90;                        // enable digital operation on TX, CLK pins
       GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

       // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
       SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
       SSI2_CR1_R = 0;                                  // select master mode
       SSI2_CC_R = 0;                                   // select system clock as the clock source
       SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
       SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
       SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

       //init LCD
       initGraphicsLcd();



   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure AN0 as an analog input(PE3)
        SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    	//GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
        GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
        GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
        ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
        ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
        ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
        ADC0_SSMUX3_R = 0;                               // set first sample to AN0
        ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
        ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


        //-------------------wtimer5 shorted with analog comparator output------------//
        //configure PD6 pin which is wtimer5 pin (compare capture)
        GPIO_PORTD_DEN_R |= 0x40;
        GPIO_PORTD_AFSEL_R |=0x40;
        GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;
        GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;


        setTimerMode();

            //configure analog comparator
            SYSCTL_RCGCACMP_R|=SYSCTL_RCGCACMP_R0;//Analog Comparator Module 0 ENABLE

            //CO+ CO- PC6 and PC7
            GPIO_PORTC_DEN_R &=~0xC0;
        	GPIO_PORTC_AFSEL_R |=0x00;
        	GPIO_PORTC_AMSEL_R |=0xC0;

        	//unlock PF0
        	GPIO_PORTF_LOCK_R=0x4C4F434B;//GPIO_LOCK_KEY;
        	GPIO_PORTF_CR_R=0x01;//GPIO_LOCK_LOCKED;

        	//CONFIGURE PF0(analog comparator output-->PF0)
        	GPIO_PORTF_AFSEL_R |=0x01;
        	GPIO_PORTF_DIR_R|=0x01;
            GPIO_PORTF_DEN_R|=0x0F;//0x01;
            GPIO_PORTF_DR2R_R|=0x0F;
        	GPIO_PORTF_PCTL_R= GPIO_PCTL_PF0_C0O;

        	COMP_ACREFCTL_R=COMP_ACREFCTL_EN|COMP_ACREFCTL_VREF_M;//enable reference stuff|setting the reference voltage(0x0000000F)2.164V--Resistor Ladder Enable|Resistor Ladder Range|Resistor Ladder Voltage Ref;
        	COMP_ACCTL0_R=0x00000402;//Analog comparator control register//TOEN bit=1 and CINV bit = 1,COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_CINV;//0x402;
     //////-------------------wtimer5 shorted with analog comparator output------------//


}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while ((UART0_FR_R & UART_FR_TXFF));
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t m;
     for (m = 0; m < strlen(str); m++)
	  putcUart0(str[m]);
}

void putsUart1(char* str_del)
{
	uint8_t p;
    for (p = 0; p < strlen(str_del); p++)
	  putcUart0(str_del[p]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while ((UART0_FR_R & UART_FR_RXFE));
	return UART0_DR_R & 0xFF;
}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
 {
	// Initialize hardware
    initHw();

    while(1)
        {

     putsUart0("\rtype the string:");
    // Toggle green LED every second


       GREEN_LED = 1;
      waitMicrosecond(1000000/2);
      GREEN_LED = 0;
      __asm("MOV  R13, R0");
      setGraphicsLcdTextPosition(0,0);
      putsGraphicsLcd("Welcome");
      emptyString();//empty main,field offset and field type string

      str1=getString();

      ParseString(str1,field_offset,i);



          if(isCommand("inductance",0)==1)
          {
        	  putsUart0("\rCommand for Inductance");


          }

          else if(isCommand("resistance",0)==1)
          {
              putsUart0("\rCommand for Resistance");

          }

          else if(isCommand("capacitance",0)==1)
          {
               putsUart0("\rCommand for Capacitance");

          }

          else if(isCommand("io",2)==1)
          {
        	  putsUart0("\rCommand for IO");
          }

          else if(isCommand("voltage",0)==1)
          {
               putsUart0("\rCommand for voltage");
          }

          else if(isCommand("voltage_f",0)==1)
          {
              putsUart0("\rCommand for voltage filter");
          }

          else if(isCommand("test",0)==1)
          {

          }

          else if(isCommand("resistor",0)==1)
          {

          }

          else if(isCommand("capacitor",0)==1)
          {

          }

          else if(isCommand("esr",0)==1)
          {

          }

          else if(isCommand("inductor1",0)==1)
           {

           }

          else if(isCommand("auto",0)==1)
           {

           }

          else if(isCommand("reset",0)==1)
           {

           }

          else
        	  putsUart0("\rOther command");
    }

}


char* getString()
{

    first=true;

	count=0;
	i=0;


	while(count<MAX_CHARS)
	      {

	      char ch= getcUart0();
	      if(ch==10)
	    	  continue;
	      if(ch=='\b'){//if BACKSPACE
	    	  if(count>0)
	    	  {
	              if(count-1==field_offset[i-1])//if character at field_offset index is to be backspaced
	              {
	            	  field_offset[i-1]='\0';
	            	  i=i-1;
	              }

	    		 //necessary backspace
	    		  {
	    			  count-=1;
	  			      str[count]='\0';
	  		      }

	    		  continue;
	    	  }
	    	  else{

	    		  continue;
	    	  }
	      }
	      else if(ch=='\r')//if CARRIAGE RETURN
	      {
	    	  str[count]=ch;
	    	  str[count]='\0';//if CARRIAGE RETURN -->END OF COMMAND
              if((str[count-1]==' ' || str[count-1]==';' || str[count-1]==':' || str[count-1]==',' || str[count-1]=='\t' ))
	    	  {
            	  while(str[count-1]==' ' || str[count-1]==';' || str[count-1]==':' || str[count-1]==',' || str[count-1]=='\t')
	    		  {
            		  str[count-1]='\0';
            		  count-=1;
            	  }
              }
	    	  break;
	      }
	      else if(ch==' '|| ch==';' || ch==':' || ch=='\t' || ch==',')//if character is a delimiter

	      {

	    	  str[count]=ch;
	           count+=1;
	    	  continue;
	      }
	      else //if normal character
	      {
	           if(str[count-1]==' ' || str[count-1]==',' || str[count-1]=='\t' || str[count-1]==':' || str[count-1]==';')//if previous character was a delimiter
	            {

	    	  str[count]=ch;//store character in array
	    	  field_offset[i]=count;//count as a field offset
	    	  i++;

	    	  flag=0;
	    	  probe=0;
	    	    }
	    	  //END OF CHARACTER INPUT

	           else//CHECK FOR CASE INSENSITIVITY
	           {
	        	   if(ch>='A' && ch<='Z')
	        	   str[count]=(ch)+32;
	        	   else
	        	   str[count]=ch;

	        	   if(( (str[count-1]>='0' && str[count-1]<='9')||str[count-1]=='.') && (str[count]>='a' && str[count]<='z') )//throw an error if an alphabet follows a number
	        	           	   {
	        	           		   caution=1;
	        	           	   }
	           }
	      }

	      if(count==0)//if first character is not a delimiter
	      {
	    	  if(str[0]!=' ' || str[0]!=',' || str[0]!=':' || str[0]!=';' || str[0]!='\t')
	    	             {
	    		          field_offset[i]=count;
	    		      	  i++;
	    	             }
	      }
	      count+=1;
	    }
	      if(caution!=1)
	      {


	   putsUart0(str);
	   putsUart0("\r");

	      }
	      else
	    	  putsUart0("\rthere is an error!");

	      return str;

}


/////////////////////////////////////////////////////PARSE THE STRING///////////////////////////////////////////////////
void ParseString(char *str,int field_offset[],int i)
{
	int t=0;
	int q;
     j=0;

while(t<i)//putting NULL at every delimiter position
   {
	  q=1;
	 while(str[field_offset[t]-q]==' ' || str[field_offset[t]-q]==',' || str[field_offset[t]-q]==':' ||str[field_offset[t]-q]==';' || str[field_offset[t]-q]=='\t')   //till all delimiters are made NULL

	  {

	   str[field_offset[t]-q]='\0';
	   q++;

	  }
       t=t+1;
   }



   for(t=0;t<i;t++)  //printing string between 2 NULL chars
   {
	   putsUart0(&str[field_offset[t]]);
	   putcUart0('\r');
   }

   for(t=0;t<i;t++)//making field_type array
      {
   if((str[field_offset[t]]>='a' && str[field_offset[t]]<='z') || (str[field_offset[t]]>='A' && str[field_offset[t]]<='Z'))
      	  {
      		  field_type[j]='a';
      		  j++;
      	  }
      	  else if((str[field_offset[t]]>='0' && str[field_offset[t]]<='9') ||str[field_offset[t]]>='.')
      	  {
      		  field_type[j]='n';
      		  j++;
      	  }
      }
   for(t=0;t<j;t++)//printing field_type array
         {
	   putcUart0(field_type[t]);
         }
  putcUart0('\r');
}

///////////////////////////////////////////////////////IS_COMMAND//////////////////////////////////////////////////////
_Bool isCommand(char* command,int ArgCount)
{

	uint16_t raw,raw1,raw3;
	float an_voltage, iirTemp,inductance,time2;
	char ana[10];

                    ///////////////LOWSIDE_R ON/OFF//////////////////////
	if(strcasecmp(command,&str1[field_offset[0]])==0 && ArgCount<i)
	{

	  if(strcasecmp("lowside_r",&str1[field_offset[1]])==0)
	  {
		  if(strcasecmp("on",&str1[field_offset[2]])==0)
		  LOWSIDE_R=1;
		  else if(strcasecmp("off",&str1[field_offset[2]])==0)
		  LOWSIDE_R=0;
		  else
		  {
          putsUart0("\nthere is an error!");
          flag=1;
		  }

       }
                 //////////////INTEGRATE ON/OFF //////////////////////////////
	  else if(strcasecmp("integrate",&str1[field_offset[1]])==0)
	  	  {
		     int x=strcasecmp("off",&str1[field_offset[2]]);
	  		  if(strcasecmp("on",&str1[field_offset[2]])==0)
	  		  {
	  			int temp=0;
	  			  INTEGRATE=1;
	  		  temp=1;
	  	  }
	  		  else if(strcasecmp("off",&str1[field_offset[2]])==0)
	  		  {
		      INTEGRATE=0;
	  		  //RED_LED=1;
	  		  }
	  		  else
	  		  {
	  		   putsUart0("\nthere is an error!");
	  		   flag=1;
	  		   }
	  	  }

	  else if(strcasecmp("meas_c",&str1[field_offset[1]])==0)
	  	  	  {
	  	  		  if(strcasecmp("on",&str1[field_offset[2]])==0)
		          MEAS_C=1;
	  	  		  else if(strcasecmp("off",&str1[field_offset[2]])==0)
		          MEAS_C=0;
	  	  		else

	  	  		{
	  	  		 putsUart0("\nthere is an error!");
	  	  		 flag=1;
	  	  		}
	  	  	  }

	  else if(strcasecmp("meas_lr",&str1[field_offset[1]])==0)

	  	  	  	  {
	  	  	  		  if(strcasecmp("on",&str1[field_offset[2]])==0)
		              MEAS_LR=1;
	  	  	  		  else if(strcasecmp("off",&str1[field_offset[2]])==0)
		              MEAS_LR=0;
	  	  	  	      else
	  	  	  		  {
	  	  	  	       putsUart0("\nthere is an error!");
	  	  	  	       flag=1;
	  	  	  		   }
	  	  	  	  }

	  else if(strcasecmp("highside_r",&str1[field_offset[1]])==0)
	  	  	  	  	  {
	  	  	  	  		  if(strcasecmp("on",&str1[field_offset[2]])==0)
	  	  	  	  		  {
		                    HIGHSIDE_R=1;
	  	  	  	  		  }
	  	  	  	  		  else if(strcasecmp("off",&str1[field_offset[2]])==0)
		                  HIGHSIDE_R=0;
	  	  	  	  	      else
	  	  	  	  		   {
	  	  	  	  	        putsUart0("\nthere is an error!");
	  	  	  	  	        flag=1;
	  	  	  	  		   }
	  	  	  	  	  }

	  else if(strcasecmp("voltage",&str1[field_offset[0]])==0)
	  {
		  val_of_voltage();
	  }

	  else if(strcasecmp("voltage_f",&str1[field_offset[0]])==0)
	  	  {
             val_of_filtered_voltage();
          }

	  else if(strcasecmp("test",&str1[field_offset[0]])==0)
	  	  	  {

		          val_of_RC_const();
	  	  	  }

	  else if(strcasecmp("resistor",&str1[field_offset[0]])==0)
	  	  	  	  {
		               val_of_resistor();
	  	  	  	  }


	  else if(strcasecmp("capacitor",&str1[field_offset[0]])==0)
	  	  	  	  {

		                val_of_capacitor();
	  	  	  	  }


	  else if(strcasecmp("esr",&str1[field_offset[0]])==0)
	 	  	  	  	  {

		                  val_of_esr();

	 	  	  	  	  }

	  else if(strcasecmp("inductor1",&str1[field_offset[0]])==0)
	  	 	  	  	  	  {
                             val_of_inductor();

	  	 	  	  	  	  }


	  else if(strcasecmp("auto",&str1[field_offset[0]])==0)
	 	  	  	 	  	  	  	  {

	 	                    	                      //CAPACITOR check

		                                                  LOWSIDE_R=0;
		                                                  MEAS_LR=0;
		                                                MEAS_C=1;
		                                                HIGHSIDE_R=1;

                                                       raw1=readAdc0Ss3();
                                              		  waitMicrosecond(1000);
                                              		  raw3=readAdc0Ss3();
                                             		  waitMicrosecond(10000000);
                                             		  raw=readAdc0Ss3();
                                             		 sprintf(ana, "%lu", raw);
                                             		 putsUart0(ana);
                                             		 if(raw>2400 && raw-raw3>1)
                                             		 {
                                                   	  putsUart0("\r");
                                                	  putsUart0("its a capacitor");
                                                	  putsGraphicsLcd("its a capacitor");
                                                	  val_of_capacitor();
                                             		 }

                                                    //INDUCTOR_RESOLUTIONISTOR check
                                                    else{
                                                    	  MEAS_C=0;
                                                    	  HIGHSIDE_R=0;
                                                    	  LOWSIDE_R=1;
                                                    	  MEAS_LR=1;
                                                    	  raw1=readAdc0Ss3();
                                             		  waitMicrosecond(5000000);
                                             		  raw=readAdc0Ss3();
                                             		 sprintf(ana, "%lu", raw);
                                             		 putsUart0("\r");
                                             		 putsUart0(ana);
                                             		 if(raw>3000 && raw<4000)
                                             		 {
                                             			 putsUart0("\r");
                                             			 putsUart0("its a inductor");
                                                   	  putsGraphicsLcd("its a inductor");
                                             			 val_of_inductor();
                                             		 }
                                             		 else
                                             		 {
                                             			putsUart0("\r");
                                             	     	 putsUart0("its a resistor");
                                                      	  putsGraphicsLcd("its a inductor");
                                             	     	 val_of_resistor();

                                             		 }
	 	  	  	 	  	  	  	  }





	 	  	  	 	  	}

	  else if(strcasecmp("reset",&str1[field_offset[0]])==0)
	  	 	 {
		         ResetISR();//resets the full hardware
	  	 	 }

	  else if(ArgCount==0)
	  {   return 1;
	  }
	  else{
		  flag=1;
	  }
        if(flag==1)
        	return 0;
        else
		return 1;
	}
	else
		return 0;
}

void val_of_RC_const()
{
	int coun=0;;
	char rc[20];
	float time2;
	                  MEAS_C=0;
			          LOWSIDE_R=1;
			          INTEGRATE=1;
			          waitMicrosecond(100);//wait to charge the whole capacitor
			          LOWSIDE_R=0;
			          WTIMER5_CTL_R |= TIMER_CTL_TAEN;//turn -ON counter
			          HIGHSIDE_R=1;
	                   while(1)
	                  {
	                	   if(timeUpdate)
	                	   {

	                		timeUpdate=false;

	                		time2=time/1.125;

	                	   sprintf(rc,"%.4f",time2);

	                	   putsUart0("\r");
	                	   putsUart0("Time Constant=");
	                	   putsUart0(rc);
	                	   putsUart0("usec");
	                	   waitMicrosecond(100);
	                	   HIGHSIDE_R=0;
	     		          LOWSIDE_R=1;//to discharge the capacitor of 1u
	     		         waitMicrosecond(100);
	     		        setGraphicsLcdTextPosition(0,coun);
	     		        putsGraphicsLcd(rc);
	     		        coun++;
	                	   }
	                  }
}

void val_of_voltage()//read voltage between DUT1 and DUT2
{
	int coun1=0;
	uint16_t raw;
	float an_voltage;
	char raw_n[20],vol[20];//array to display to UART
	while(1)
			  {
			  MEAS_LR=0;
			  MEAS_C=1;
			  raw = readAdc0Ss3();
			  putsUart0("\rraw value");
			  sprintf(raw_n, "%u", raw);
			  putsUart0(raw_n);
			  an_voltage = ((raw / (RESOLUTION) * VCC)-VCE_SAT);
			  putsUart0("\rAnalog value");
			  sprintf(vol, "%f", an_voltage);
			  putsUart0(vol);
			  waitMicrosecond(1000000/2);
			  setGraphicsLcdTextPosition(0,coun1);
			  putsGraphicsLcd(vol);
			  coun1++;
			  }
}

void val_of_filtered_voltage()//read filtered voltage between DUT1 and DUT2
{
	int coun2=0;
	 uint16_t raw;
	 float an_voltage;
	 char vol1[20],raw_n1[20];//array to display to UART
	while(1)
			  	 {
			  		MEAS_LR=0;
			  		MEAS_C=1;

			  		raw = readAdc0Ss3();
			  		putsUart0("\rraw value");
			  		sprintf(raw_n1, "%u", raw);
			  		putsUart0(raw_n1);
			  		INTEGRATE=1 ;
			  		an_voltage = ((raw / (RESOLUTION) * VCC)-VCE_SAT);
			  	     putsUart0("\rAnalog filtered value");
			  		 sprintf(vol1, "%f",an_voltage );
			  		putsUart0(vol1);
			  		 waitMicrosecond(1000000/2);
			  		setGraphicsLcdTextPosition(0,coun2);
			          putsGraphicsLcd(vol1);
			          coun2++;
			  	 }
}


void val_of_inductor()
{
	int coun3=0;
	uint32_t r_esr;
     uint16_t raw;
     char ind[20];//array to display to UART

               LOWSIDE_R=1;
			   MEAS_LR=1;
			   waitMicrosecond(1000000);
			   raw = readAdc0Ss3();

			   r_esr=(((RESOLUTION*10)*3.3)/raw)-33;
			   MEAS_LR=0;
			   waitMicrosecond(1000000);
			   WTIMER5_CTL_R |= TIMER_CTL_TAEN;
			   MEAS_LR=1;

			              while(1){
			                           if (timeUpdate)
			                             {

			                               timeUpdate=false;

			                               inductance=(((33+r_esr)*time))/1.4;
			                               sprintf(ind,"%.4f",inductance);

			                               putsUart0("\r");
			                               putsUart0("Inductance=");
			                               putsUart0(ind);
		                                   putsUart0("uH");
			                               waitMicrosecond(1000000);

			                               setGraphicsLcdTextPosition(0,coun3);
			                                putsGraphicsLcd(ind);
			                                coun3++;
			                              }


		  	 	  	  	  	  }
}

void val_of_esr()
{

	          char raw_n2[20];
	          char esr[20];
	          uint32_t r_esr;
	          uint16_t raw;
	                          MEAS_C=0;
		                       MEAS_LR=1;
		 	  		           LOWSIDE_R=1;
		 	  		          waitMicrosecond(100000);
		 	  		           WTIMER5_CTL_R |= TIMER_CTL_TAEN;//turn -ON counter
		 	  		          raw=readAdc0Ss3();
		 	  		       sprintf(raw_n2,"%lu",raw);
		 	  		          putsUart0(raw_n2);
		 	  		          r_esr=(((RESOLUTION*10)*VCC)/raw)-33;//(R/(R+R_esr))*(V_out+VCE_sat)=V_out
		 	  		           sprintf(esr,"%lu",r_esr);
		 	  		           putsUart0("\r");
		 	  		            putsUart0("Series resistance=");
		 	  		           putsUart0(esr);
		 	  		           MEAS_LR=0;
		 	  		        waitMicrosecond(100000);
		 	  		     setGraphicsLcdTextPosition(0,0);
		 	  		     putsGraphicsLcd(esr);
}

void val_of_capacitor()
{
	     //uint16_t raw;
	     int coun5=0;
	     char cap[20];
	       MEAS_LR=0;
		   LOWSIDE_R=1;
          waitMicrosecond(100000);
	      LOWSIDE_R=0;
	      WTIMER5_CTL_R |= TIMER_CTL_TAEN;//turn -ON counter
		   HIGHSIDE_R=1;
		   MEAS_C=1;
		         while(1)
		           {
		                  	   if(timeUpdate)
		                  	   {
		                  		   //WTIMER5_ICR_R = TIMER_ICR_CAECINT;
		                  		timeUpdate=false;
		                  		time=(time/(1.38))/100000;
		                  	   sprintf(cap,"%.4f",time);

		                  	   putsUart0("\r");
		                  	   putsUart0("Capacitance=");
		                  	   putsUart0(cap);
		                  	 putsUart0("uF");
		                  	   waitMicrosecond(100000);
		                  	   HIGHSIDE_R=0;
		       		          LOWSIDE_R=1;
		       		         waitMicrosecond(100000);

		       		      setGraphicsLcdTextPosition(0,coun5);
		       		      	  putsGraphicsLcd(cap);
		       		      	  coun5++;
		                  	   }
		            }
}

void val_of_resistor()
{
	int coun6=0;
	float time2;
	char res[20];

	     MEAS_C=0;
	     LOWSIDE_R=1;
		 INTEGRATE=1;
		 waitMicrosecond(100);//giving some time to charge the 1uF capacitor
		 LOWSIDE_R=0;
		  WTIMER5_CTL_R |= TIMER_CTL_TAEN;//turn -ON counter
		  MEAS_LR=1;//path from MEAS_LR to INEGRATE thru 1uF capacitor  T=RC C=1uF
		         while(1)
		            {
		                  	   if(timeUpdate)
		                  	   {

		                  		timeUpdate=false;

		                  		time2=time/1.12;
		                  	   sprintf(res,"%.4f",time2);

		                  	   putsUart0("\r");
		                  	   putsUart0("Resistance=");
		                  	   putsUart0(res);
		                  	   putsUart0("ohms");
		                  	   waitMicrosecond(200);
		                  	   HIGHSIDE_R=0;
		       		          LOWSIDE_R=1;
		       		         waitMicrosecond(200);
		       		      setGraphicsLcdTextPosition(0,coun6);
		       		      putsGraphicsLcd(res);
		       		      coun6++;
		                  	   }
		            }
}





uint16_t readAdc0Ss3()
      {
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
      }

void emptyString()
{
	int u,v,w;
	for(u=0;u<MAX_CHARS;u++)
	{
		str[u]='\0';

	}
	for(v=0;v<i;v++)
		field_offset[v]='\0';
	for(w=0;w<j;w++)
        field_type[w]='\0';
}

