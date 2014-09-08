/*****************************************************************************/
//	main.c
//  
//	 Created on:
//	     Author:
//
/*****************************************************************************/


/*****************************************************************************/
//	Includes
/*****************************************************************************/
//Put includes here
#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>			// Include for ltoa function
/*****************************************************************************/
//	End Includes


/*****************************************************************************/
//	Defines
/*****************************************************************************/
//Put defines here
#define STOP_WATCHDOG					(WDTCTL = WDTPW + WDTHOLD)
#define LED_BLINK_INTERVAL				(50000)
#define __CLMSPDBAFE_DEMO_VERSION__		"1.0"
/*****************************************************************************/
//	End Defines


/*****************************************************************************/
//	Global Constant Declarations
/*****************************************************************************/
//Put Global Constant Declarations here
const uint8_t COMPILE_VERSION[16] = __CLMSPDBAFE_DEMO_VERSION__;
const uint8_t COMPILE_DATE[16] = __DATE__;
const uint8_t COMPILE_TIME[16] = __TIME__;
const uint8_t INTRO_1[] =
  "This is the Casco Logix MSP430AFE253 Development Board (CL-MSPDB-AFE)!\r\n";
const uint8_t INTRO_2[] = "Visit www.cascologix.com for more information.\r\n";
const uint8_t VERSION_INFO[] = "Version: ";
const uint8_t CREATED_INFO[] = "Created: ";
const uint8_t SW1_MSG[] = "Switch 1 pressed\r\n";
const uint8_t SW2_MSG[] = "Switch 2 pressed\r\n";
/*****************************************************************************/
//	End Global Constant Declarations


/*****************************************************************************/
//	Global Variable Declarations
/*****************************************************************************/
//Put Global Variable Declarations here
uint32_t results[5];
uint8_t * pMsg = 0;
uint8_t strNum[16];
/*****************************************************************************/
//	End Global Variable Declarations


/*****************************************************************************/
//	Function Prototypes
/*****************************************************************************/
//Put function prototypes here
void serialPrintBlocking (uint8_t * pString);
void serialPrintNonBlocking (uint8_t * pString);
void introMessage (void);
void basic_clock_init (void);
void XT2_clock_init (void);
void UART_init (void);
void timerA_init (void);
void SD24_init (void);
void SD24_startConversion (void);
void SD24_getTemp (uint32_t * adcReading);
void SD24_getBatt (uint32_t * adcReading);
void SD24_getShunt (uint32_t * adcReading);
void switch_init (void);
void LED_init (void);
void doSw1Action (void);
void doSw2Action (void);
/*****************************************************************************/
//	End Function Prototypes


/*****************************************************************************/
//	Main Definition
/*****************************************************************************/
void main(void)
{
	STOP_WATCHDOG;							// Stop watchdog timer

	basic_clock_init();						// Initialize basic clock system
	XT2_clock_init();						// Initialize XT2 clock
	UART_init();							// Initialize UART
	switch_init();							// Initialize switch inputs
	LED_init();								// Initialize LED output
	timerA_init();							// Initialize timer A module
	SD24_init();

	introMessage();							// Send intro message

	_EINT();								// Set global interrupt enable

	for(;;)
	{
		LPM0;								// Enter low power mode 0

		serialPrintBlocking((uint8_t*)"\r\n");// Print carriage return and newline
		ltoa(results[0], (char*)strNum);	// Convert integer to ascii text
		serialPrintBlocking(strNum);		// Print value to serial port
	}
}
/*****************************************************************************/
//	End Main Definition


/*****************************************************************************/
//	Function Definitions
/*****************************************************************************/
//Put function definitions here
void serialPrintBlocking (uint8_t * pString)
{
	IE1 &= ~UTXIE0;							// Disable USART0 RX/TX interrupt
	IFG1 |= UTXIFG0;						// Set interrupt flag

	while(*pString)
	{
		while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
		U0TXBUF = *pString;					// Write next char to buffer
		pString++;
	}

	IFG1 &= ~UTXIFG0;						// Clear interrupt flag
	IE1 |= UTXIE0;							// Enable USART0 RX/TX interrupt
}


void serialPrintNonBlocking (uint8_t * pString)
{
	pMsg = pString;							// Set pointer to point to string
	U0TXBUF = *pMsg;						// Write first char to TXBUF
	pMsg++;									// Index to the next char
											// TX interrupt handles the rest
}


void introMessage (void)
{
	serialPrintBlocking((uint8_t*)INTRO_1);
	serialPrintBlocking((uint8_t*)INTRO_2);
	serialPrintBlocking((uint8_t*)VERSION_INFO);
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)COMPILE_VERSION);
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)", ");
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)CREATED_INFO);
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)COMPILE_DATE);
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)", ");
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)COMPILE_TIME);
	while (!(IFG1 & UTXIFG0));			// Block until TX interrupt flag set
	serialPrintBlocking((uint8_t*)"\r\n");
}


void basic_clock_init (void)
{
	uint16_t idx;
	do
	{
		IFG1 &= ~OFIFG;						// Clear OSCFault flag
		for (idx = 0x47FF; idx > 0; idx--);	// Time for flag to set
	}
	while((IFG1 & OFIFG));					// OSCFault flag still set?
}


void XT2_clock_init (void)
{
	BCSCTL3 = XT2S_2;						// Set frequency range of XT2
	BCSCTL1 &= ~XT2OFF;						// Enable XT2

	uint16_t idx;
	do
	{
		IFG1 &= ~OFIFG;						// Clear OSCFault flag
		for(idx = 0x47FF; idx > 0; idx--);	// Time for flag to set
	}
	while((IFG1 & OFIFG));					// OSCFault flag still set?

	BCSCTL2 = SELM_2 | SELS | DIVS_3;		// Select XT2 clock source for
											//   MCLK and MCLK/8 for SMCLK
}


void UART_init (void)
{
	P1DIR |= BIT3;							// P1.3,1.4 = USART0 TXD/RXD
	P1SEL |= BIT3 | BIT4;					// P1.3,1.4 = USART0 TXD/RXD

	ME1 |= UTXE0 + URXE0;					// Enable USART0 TXD/RXD
	U0CTL |= CHAR;							// 8-bit character
	U0TCTL |= SSEL1;						// Clock source is SMCLK
	U0BR0 = 9;								// 1MHz 115200
	U0BR1 = 0x00;							// 1MHz 115200
	U0MCTL = 0x00;							// 1MHz 115200 modulation
	U0CTL &= ~SWRST;						// Initialize USART state machine
	IE1 |= URXIE0 | UTXIE0;					// Enable USART0 RX/TX interrupt
}


void timerA_init (void)
{
	P1SEL |= BIT7;							// P1.7 option select
	P1SEL2 |= BIT7;							// P1.7 option select

	CCR1 = LED_BLINK_INTERVAL;				// Set CCR1 interval
	CCTL1 = OUTMOD_4 + CCIE;				// CCR1 toggle
	//CCTL1 = OUTMOD_4 + CCIE;				// CCR1 toggle, interrupt enabled
	TACTL = TASSEL_2 +  MC_2 + ID_3;		// SMCLK, Contmode, Clk/8
	//TACTL = TASSEL_2 +  MC_2 + TAIE;		// SMCLK, Contmode, int enabled
}


void SD24_init (void)
{
	SD24CTL = SD24REFON | SD24SSEL0;		// 1.2V ref, SMCLK
	SD24CCTL0 = SD24SNGL | SD24GRP;			// Single conv, group with CH1
	SD24CCTL1 = SD24SNGL | SD24GRP;			// Single conv, group with CH2
	SD24CCTL2 = SD24SNGL | SD24IE;			// Single conv, enable interrupt

	uint16_t idx;
	for (idx = 0x3600; idx > 0; idx--);		// Delay for 1.2V ref startup
}


void SD24_startConversion (void)
{
	SD24CCTL2 |= SD24SC;
}


void SD24_getAVCC (uint32_t * adcReading)
{
	// Set to channel for AVCC sensor
	SD24INCTL0 = SD24INCH_5;				// Select AVCC Channel
	SD24CCTL2 |= SD24SC;					// Start Conversion
	while(~(SD24CCTL0 & SD24IFG));			// Wait for conversion to finish
	*adcReading = SD24MEM0;					// Get ADC result
	SD24INCTL0 = SD24INCH_0;				// Set channel back to 0
}


void SD24_getTemp (uint32_t * adcReading)
{
	// Set to channel for Temperature sensor
	SD24INCTL0 = SD24INCH_6;				// Select temperature sensor channel
	SD24CCTL2 |= SD24SC;					// Start Conversion
	while(~(SD24CCTL0 & SD24IFG));			// Wait for conversion to finish
	*adcReading = SD24MEM0;					// Get ADC result
	SD24INCTL0 = SD24INCH_0;				// Set channel back to 0
}


void SD24_getShunt (uint32_t * adcReading)
{
	SD24INCTL0 = SD24INCH_7;				// Select shunt channel
	SD24CCTL2 |= SD24SC;					// Start Conversion
	while(~(SD24CCTL0 & SD24IFG));			// Wait for conversion to finish
	*adcReading = SD24MEM0;					// Get ADC result
	SD24INCTL0 = SD24INCH_0;				// Set channel back to 0
}


void switch_init (void)
{
	P1REN |= BIT2 | BIT0;					// Enable resistor on P1.0 and P1.2
	P1OUT |= BIT2 | BIT0;					// Set resistors as pullups
	P1IES |= BIT2 | BIT0;					// Set IES to falling edge
	P1IFG = 0;								// Ensure flags are cleared
	P1IE |= BIT2 | BIT0;					// Enable pin interrupt
}


void LED_init (void)
{
	P1DIR |= BIT7;							// Set P1.7 as output
}
/*****************************************************************************/
//	End Function Definitions


/*****************************************************************************/
//	Interrupt Service Routines
/*****************************************************************************/
//Put Interrupt Service Routines here
#pragma vector=USART0RX_VECTOR
__interrupt void USART0_RX_ISR (void)
{
	TXBUF0 = RXBUF0;						// RXBUF0 to TXBUF0
}


#pragma vector=USART0TX_VECTOR
__interrupt void USART0_TX_ISR (void)
{
	if(pMsg)								// Ensure pointer is not null
	{
		if(*pMsg)							// Check for string null terminator
		{
			U0TXBUF = *pMsg;				// Write byte
			pMsg++;							// Index to next byte in string
		}
	}

	pMsg = 0;								// Reset pointer to null in case
											//   ISR gets called again after
											//   completion of string, but
											//   before a new string is assigned
}


// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A0_ISR (void)
{
	_NOP();									// Placeholder to set breakpoint
}


// Timer_A1 Interrupt Vector (TAIV) handler
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A1_ISR (void)
{
	switch(TAIV)
	{
		case  2:
			CCR1 = +LED_BLINK_INTERVAL;		// CCR1
			SD24_startConversion();
			break;
		case  4:
			_NOP();							// CCR2
			break;
		case 10:
			_NOP();							// Overflow
			break;
	}
}


#pragma vector=SD24_VECTOR
__interrupt void SD24A_ISR (void)
{
	switch (SD24IV)
	{
		case 2:								// SD24MEM Overflow
			break;
		case 4:								// SD24MEM0 IFG
			break;
		case 6:								// SD24MEM1 IFG
			break;
		case 8:								// SD24MEM2 IFG
			results[0] = SD24MEM0;			// Save CH0 results (clears IFG)
			results[1] = SD24MEM1;			// Save CH1 results (clears IFG)
			results[2] = SD24MEM2;			// Save CH2 results (clears IFG)
			LPM0_EXIT;						// Exit LPM0 to go back to main
											//   loop and print value to serial
			break;
	}
}


#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
	volatile uint8_t validatedFlags;

	validatedFlags = P1IFG & P1IE;

	switch(validatedFlags)
	{
		case BIT0:
			P1IFG &= ~BIT0;						// Clear flag
			serialPrintNonBlocking((uint8_t*)SW2_MSG);
			//doSw2Action();
			break;

		case BIT1:
			P1IFG &= ~BIT1;						// Clear flag
			break;

		case BIT2:
			P1IFG &= ~BIT2;						// Clear flag
			serialPrintNonBlocking((uint8_t*)SW1_MSG);
			//doSw1Action();
			break;

		case BIT3:
			P1IFG &= ~BIT3;						// Clear flag
			break;

		case BIT4:
			P1IFG &= ~BIT4;						// Clear flag
			_NOP();
			break;

		case BIT5:
			P1IFG &= ~BIT5;						// Clear flag
			break;

		case BIT6:
			P1IFG &= ~BIT6;						// Clear flag
			break;

		case BIT7:
			P1IFG &= ~BIT7;						// Clear flag
			break;

		default:
			_NOP();
			break;
	}
}


#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR (void)
{
	switch(P2IFG)
	{
		case BIT0:
			P2IFG &= ~BIT0;						// Clear flag
			break;

		case BIT1:
			P2IFG &= ~BIT1;						// Clear flag
			break;

		case BIT2:
			P2IFG &= ~BIT2;						// Clear flag
			break;

		case BIT3:
			P2IFG &= ~BIT3;						// Clear flag
			break;

		case BIT4:
			P2IFG &= ~BIT4;						// Clear flag
			_NOP();
			break;

		case BIT5:
			P2IFG &= ~BIT5;						// Clear flag
			break;

		case BIT6:
			P2IFG &= ~BIT6;						// Clear flag
			break;

		case BIT7:
			P2IFG &= ~BIT7;						// Clear flag
			break;

		default:
			_NOP();
			break;
	}
}


#pragma vector=WDT_VECTOR
__interrupt void Watchdog_ISR (void)
{
	_NOP();
}


#pragma vector=NMI_VECTOR
__interrupt void NMI_ISR (void)
{
	_NOP();
}


#pragma vector=unused_interrupts
__interrupt void Unused_ISRs (void)
{
	_NOP();
}
/*****************************************************************************/
//	End Function Definitions
