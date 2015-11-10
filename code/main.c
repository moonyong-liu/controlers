
#include "io430.h"

#include "mcp2515.h"
#include "protocol.h"
#include "global_data.h"
#include "string.h"
#include "logic_control.h"


#define LOL  200   /* led light length of time */
#define LOBT 300  /* length of beep time     300 ms*/
#define LOBTc 5000 /* length of beep time    5000 ms*/

_RDataTmp* CanBuf;        // canbuff pointer
/* extern variable of interrupt.c */

// CAN
extern uint8 FlagINT;
extern uint8 FlagRX0BF ;
extern uint8 FlagRX1BF;
extern uint8 FlagTX0RST;
extern uint8 FlagTX1RST;
extern uint8 FlagTX2RST;

/* local variable */



/* Golbal variables of led light */
uint16 FlagLLT;                            // flag led light time
uint8 FlagLLTU2TX;                          // Flag led light time uart2 transmit

uint8 FlagLLTU3TX;                       // FLAG led light time uart3 transmit
uint16 FlagLLTz;                         // flag led light time z 

uint8 FlagLLTU1RX;                      // flag led light length of time of uart1 receives
uint16 FlagLLTx;                         // flag led light legth of time by z

uint8 FlagLLTU0RX;                        // flag led light length of time of uart0 receives
uint16 FlagLLTc;                            // flag led light length of time by c 

uint8 FlagLLTU2RX;                         // flag led light length of time of uart2 receives
uint16 FlagLLTv;                              // flag led light length of time by c

uint8 FlagLLTU0TX;                           // flag led light length of time of uart0 transmit
uint16 FlagLLTb;                                 // flag led ligth length of time by b

uint8 FlagLLTU1TX;                           // flag led light length of time of uart1 transmit
uint16 FlagLLTn;                                 // flag led light length of time by n

uint8 FlagBrath;                            // flag breathe


uint16 FLagNCPC;                           // flag now config package count
uint16 FlagLCPC;                            // flag last config package count

/* golbal varialbes of beep flags */
uint8 BeepONOFF;                              // beep on of off flag 
uint8 BeepOn;                                  // beep on flag
uint8 BeepOff;                                 // beep off flag 
uint8 Flag1sBe1ci=1;                          // 1 second beep 1 time flag
uint16 FlagBeex;                                // count how many "ms" is
uint8 Flag5sBe1ci=1;                         // 5sencond beep 1time flag
uint16 FlagBeec;                               // count how man ms is

unsigned long long UartRTime;
void PINInit(void)
{
  P1DIR =0;		// Set P1 to in direction
  P1REN =0xff;	// Enable P1 internal resistance
  P1IE  =0;
  P1IES =0;
  P1IFG =0;
  P1IN  =0;
  P1OUT =0;

  P2DIR =0;		// Set P2 to in direction
  P2REN =0xff;	// Enable P2 internal resistance
  P2IE  =0;
  P2IES =0;
  P2IFG =0;
  P2IN  =0;
  P2OUT =0;
  
  P3DIR =0;		// Set P3 to in direction
  P3REN =0xff;	// Enable P3 internal resistance
  P3IN  =0;
  P3OUT =0;
  
  P4DIR =0;		// Set P4 to in direction
  P4REN =0xff;	// Enable P4 internal resistance
  P4IN  =0;
  P4OUT =0;
  
  P5DIR =0;		// Set P5 to in direction
  P5REN =0xff;	// Enable P5 internal resistance
  P5IN  =0;
  P5OUT =0;
  
  P6DIR =0;		// Set P6 to in direction
  P6REN =0xff;	// Enable P6 internal resistance
  P6IN  =0;
  P6OUT =0;
  
  P7DIR =0;		// Set P7 to in direction
  P7REN =0xff;	// Enable P7 internal resistance
  P7IN  =0;
  P7OUT =0;
  
  P8DIR =0;		// Set P8 to in direction
  P8REN =0xff;	// Enable P9 internal resistance
  P8IN  =0;
  P8OUT =0;
  
  P9DIR =0;		// Set P9 to in direction
  P9REN =0xff;	// Enable P9 internal resistance
  P9IN  =0;
  P9OUT =0;
  
  P10DIR =0;		// Set P10 to in direction
  P10REN =0xff;	// Enable P10 internal resistance
  P10IN  =0;
  P10OUT =0;
  
  PJDIR =0;		// Set PJ to in direction
  PJREN =0xff;	// Enable PJ internal resistance
  PJIN  =0;
  PJOUT =0;
}



void SetVcoreUp (unsigned int level)
{
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  // Wait till SVM is settled
  //fuck while ((PMMIFG & SVSMLDLYIFG) == 0);
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG)) 
  while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
}

int main( void )
{
  WDTCTL = WDTPW+WDTHOLD;//Stop wdt
  WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192+WDTHOLD;//Stop wdt
  PINInit();
  SetVcoreUp(PMMCOREV_1);
  SetVcoreUp(PMMCOREV_2);
  SetVcoreUp(PMMCOREV_3);			// Set VCore to 1.8MHz for 20MHz
  P5SEL |= 0x0C;					// Port select XT2
  UCSCTL6 &= ~XT2OFF;				// Enable XT2
  UCSCTL3 |= SELREF_2;             
  UCSCTL4 |= SELA_2;				// ACLK=REFO,SMCLK=DCO,MCLK=DCO
  // Loop until XT1,XT2 & DCO stabilizes
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);    // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;			// Clear fault flags
  }while (SFRIFG1&OFIFG);			// Test oscillator fault flag
  UCSCTL6 &= ~XT2DRIVE0;			// Decrease XT2 Drive accord                             // to expected frequency
  UCSCTL4 |= SELS_5 + SELM_5;		// SMCLK=MCLK=XT2
  __enable_interrupt();
  can_init();
  logic_init();
  //P6SEL = 0xFF;            // Enable A/D channel inputs
  TA0CCTL0 = CCIE;		   // CCR0 interrupt enabled
  TA1CCTL0 = CCIE;		   // CCR0 interrupt enabled
  //P1OUT &= ~0x10;   ///REALYON
  //P1OUT |= 0x10;
  WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192;		// Run WDT  
  while(1)
  {  
    WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192;// Feed WDT
    CanBuf = can_working_colect();
    if( CanBuf != NULL ){
      handle_data_logic( CanBuf );
    }
    exe_working();
  }
}


