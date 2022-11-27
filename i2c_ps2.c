/*********************************************************************
 *
 *                Microchip C30 Firmware - I2C_PS2
 *
 *********************************************************************
;    Filename:	    I2C_Ps2.c	(convertitore da tastiera PS/2 a I2C/RS232) *
;    Date:          27/11/22                                          *
;    File Version:  1.00																              *
;                                                                     *
;    Author:        Dario Greggio                                     *
;    Company:       DA (Cyberdyne (ADPM Synthesis sas))                   *
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC24
 * Compiler:        C30 2.30.01+
 * Company:         Microchip Technology, Inc.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CinziaG 							11/2022 		ischia GODO CAZZO!!!! morte ai carabinieri cancro ai terroni
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#ifdef __XC16
#include <xc.h>
#else
#include <p24fxxxx.h>
#endif
#include <stdio.h>
#include <string.h>


#include "i2c_ps2.h"
#include <libpic30.h>
#include "io_cfg.h"             // I/O pin mapping

#include "ps2.h"
#include "keyboard.h"



#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)

// PIC24EP512GP202 Configuration Bit Settings

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = ON            // Alternate I2C1 pins (I2C1 mapped to ASDA1/ASCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS4096         // Watchdog Timer Postscaler bits (1:4,096)
#pragma config WDTPRE = PR32            // Watchdog Timer Prescaler bit (1:32)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (Watchdog timer always enabled)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF             //per PWM dinamico metto off! Peripheral pin select configuration bit (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL))
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)


#else
_FBS( BWRP_OFF & BSS_OFF ) 
_FGS( GWRP_OFF & GSS0_OFF ) 
_FOSCSEL(FNOSC_FRCPLL & IESO_OFF & LPRCSEL_LP & SOSCSRC_DIG)			//FNOSC_FPRILL 
_FOSC( POSCMOD_NONE & OSCIOFNC_OFF & FCKSM_CSDCMD & SOSCSEL_SOSCLP & POSCFREQ_HS)
_FWDT( WDTPS_PS8192 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_ON )		// 8sec WDT
_FPOR( PWRTEN_ON & BOREN_BOR3 & BORV_V20 & I2C1SEL_PRI & LVRCFG_ON & MCLRE_ON )  // mclr off non gli piace..; BORV=20, a 27 si bloccava già sotto i 3...
_FICD( ICS_PGx2 )
#endif


/** V A R I A B L E S ********************************************************/


BYTE InitOK;			// idem...
char disconnected=1;


BYTE kbScanCode;
BYTE keyToSend[2];
BYTE FLAGK;
WORD FLAGS;
volatile BYTE kbKeys[MAX_TASTI_CONTEMPORANEI];
BYTE kbKeysOld[MAX_TASTI_CONTEMPORANEI];

BYTE keyboardTypematic1=1,keyboardTypematic2=15,ledStatusLocal=FALSE;
extern unsigned char PS2Errors;
extern unsigned char SlavePresent;

volatile BYTE minute_1,second_1;



struct SAVED_PARAMETERS configParms;


BYTE temp2;
BYTE temp3;



//__attribute__((section("__FUID0.sec,code"))) int UID0 = 0x44; // hmmm, su PIC24F non ci sono, su H sì!
/*_FUID0(0x1111);
_FUID1(0x2222);
_FUID2(0x3333);
_FUID3(0x4444);*/
//	__IDLOCS 0x4744


static const char I2C_PS2_C[]="$Id:I2C_PS2.C Ver 1.0.0 - 27/11/22";
extern const char *CopyrString;

// ---------------------------------------------------------------------



/** P R I V A T E  P R O T O T Y P E S ***************************************/
BYTE recvMsg(void);
void doMain(void);

void MandaTastoPremuto(BYTE);
void MandaTastoPremuto2(void);
void MandaTastoRilasciato(BYTE);

short int GetByte232(void);
void PutByte232(BYTE);
void sendQueryStatus232(void);




/******************************************************************************
 * Function:        void (void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/



#define DEVICE_ADDRESS 0x48     //I2C device address (device_address+1 = read) BOH LASCIO QUESTO ANCHE QUA!
																// in teoria il dispositivo dovrebbe anche rispondere a "0+W" (GENERAL_CALL), preparandosi a ricevere un secondo BYTE di istruzioni... (v.Philips)

// In lettura:   subAddress 0 restituisce 8 char ID+versione
//							  subAddress 1 restituisce 4 BYTE CFG (equip,slavepresent,ecc?)
//							  subAddress 2 restituisce 4 BYTE buffer tastiera (stato tasti) 
// In scrittura: subCommand 0 = enable/disable
//								subCommand 1 = reset
//								subCommand 2 = Set led
//								subCommand 3 = Set typematic
//								subCommand 160 = bootloader






union COMM_STATUS CommStatus;
BYTE Byte2Send;
BYTE ByteRec;
BYTE Buf232Ptr;					//contatore per scrittura buffer 232
BYTE Buf232PtrI;				//contatore per lettura buffer 232
BYTE Buf232[16];				// buffer 16 BYTE per ricezione I2C e 232 



WORD I2CAddr;
BYTE I2CBuffer[16];					// buffer 16 BYTE per ric./trasm. I2C-slave 

extern volatile BYTE I2CFLG;              //I2C flag reg
//------------------------------------------------------------
volatile BYTE I2CSUBA;		              //Subaddress
extern volatile BYTE I2CCNT;						//contatore per lettura/scrittura buffer I2C
extern volatile BYTE I2CR0[16];					// buffer 16 BYTE per ricezione I2C e 232
extern volatile BYTE PacketReceived;


#define CR          0xd
#define LF          0xa
#define BEL					0x7
#define BS					0x8
#define TAB					0x9
#define DC1					0x11
#define DC2					(DC1+1)
#define DC3					(DC1+2)
#define DC4					(DC1+3)
#define FF					0xc
#define CAN					0x18				// stesso di FF ossia CLS
#define ESC					0x1b
// VTAB, curs giu', curs dx??

#define FLASH_TIME  7




//
//***** VARIABLE DEFINITIONS

BYTE myPortC;						// specchio portB
//
WORD Counter;
BYTE CounterL2;


BYTE TimerBuzz;
WORD BeepFreq;


BYTE Clock_10;		// (decimi di secondo per dividere interrupt); ANCHE MARKER_GOOD_EEPROM (solo in Eprom, v.)








/** D E C L A R A T I O N S **************************************************/
int main(void) {
	static short int ch;
	static BYTE i;



/*========================================================================
;       RESET ENTRY
;========================================================================*/
start:
cold_reset:

//	InitOK=RCON 	/*STATUS*/;
	RCON &= ~0b0000000010000001;			// pulisco cmq hard reset (ev. per bootloader...)

	ClrWdt();					// DOPO!

//	RCONbits.NOT_BOR=1;
//	RCONbits.NOT_POR=1;

#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__)
	CLKDIVbits.FRCDIV = 0b000;		// Set 32MHz FRC postscalar pll 
  
//	_PLLDIV=302;						// M = _PLLDIV+2, 2..513
//	_PLLPRE=6;						// N1 = _PLLPRE+2, 2..33
//	_PLLPOST=0;						// N2 = _PLLPOST, 2 (0), 4 (1), 8 (3)
	OSCTUN=0;			//(da USB)
// 
// (280*7.37)/(8*2) = 129~  (osc interno)
// (280*8)/(8*2) = 140
// (304*7.37)/(8*2) = 140

	// forse meglio così?
  CLKDIVbits.PLLPRE = 0; // N1 = 2		// in quest'ordine, dice http://www.microchip.com/forums/FindPost/1011737 (ma non è vero...))
  CLKDIVbits.PLLPOST = 0; // N2 = 2  
  PLLFBD = 74; // M = PLLFBD + 2 = 70
//  __builtin_write_OSCCONH(0x01);
//  __builtin_write_OSCCONL(OSCCON | 0x01);
#ifndef __DEBUG
//  while(OSCCONbits.COSC != 1)		// OCCHIO a che OSC si usa!
//		ClrWdt();
#endif


/*	
  Fosc= Fin*M/(N1*N2)
	0.8MHz<(Fin/N1)<8MHz
	100MHz<(Fin*M/N1)<340MHz
	M=2,3,4,...513
	N1=2...33
	N2=2,4,8
	
	PLLFBD    = M -2;
	CLKDIVbits.PLLPRE = N1 -2;
	CLKDIVbits.PLLPOST = 0b00; //N2={2,4,R,8} */

//OSCCONbits.CLKLOCK=1;OSCCONbits.NOSC=1;OSCCONbits.OSWEN=1;
//  while (OSCCONbits.COSC != 0x7)LATB ^= 1;; 
//	while(OSCCONbits.LOCK!=1)
//		ClrWdt();


	//Wait for the Primary PLL to lock and then
       // configure the auxilliary PLL to provide 48MHz needed for USB
       // Operation.
#ifndef __DEBUG
//#if !defined(__dsPIC33EP128GS702__)   // v. sopra PLLKEN flag...
  while(OSCCONbits.LOCK != 1) ClrWdt();			// boh?
//#endif

#endif

#else
	CLKDIVbits.RCDIV = 0b000;		// Set 32MHz FRC postscalar pll 
	// imposto oscillatore a 32MHz
  while(OSCCONbits.LOCK != 1)
		ClrWdt();			// boh?
#endif
  

	__delay_ms(100);		// sembra sia meglio aspettare un pizzico prima di leggere la EEPROM.. (v. forum 2006)
	di();			// disabilito tutti interrupt



	TRISA=0b0000000000011001;				// puls1-2; CTS; led
	TRISB=0b1111001100000010;				// RX; PS2; I2C; buzzer; led
	LATA= 0b0000000000000000;				// 
	LATB= 0b0000000000000000;

	ANSELA=0b0000000000000000;			// 
	ANSELB=0b0000000000000000;

#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)
  AD1CON1 = 0b0000000000000000;			// 
  AD1CON2 = 0b0000000000000000;			// 
  AD1CON3 = 0b0000000000000000;			// 
  AD1CON4 = 0b0000000000000000;			// 
  AD1CHS123 = 0b0000000000000000;		// 
  AD1CHS0 = 0b0000000000000000;			// 
  AD1CSSH = 0b0000000000000000;			// 
  AD1CSSL = 0b0000000000000000;			// 
#else
	AD1CON1=0b0000000000000000;
	AD1CON2=0b0000000000000000;
	AD1CON3=0b0000000000000000;
	AD1CON5=0b0000000000000000;
	AD1CHS =0b0000000000000000;		// 
	AD1CHITL=AD1CHITH=0;
	AD1CSSL=AD1CSSH=0;
	AD1CTMUENH=AD1CTMUENL=0;
	AD1CON1bits.ADON=1;
#endif
  

#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)
	CNPUAbits.CNPUA0 = 1;   		// pulsante1
//	CNPUBbits.CNPUB8 = CNPUBbits.CNPUB9 = 1;			// i2c??
	CNPUAbits.CNPUA3 = 1;				// pulsante2
#else
	CNPU1bits.CN2PUE = 1;				// pulsante2
	CNPU2bits.CN29PUE = 1;				// pulsante1
//	CNPU2bits.CN21PUE = CNPU2bits.CN22PUE = 1;			// i2c??
#endif

#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
  __builtin_write_OSCCONL(OSCCON & 0xbf);
  // PPS - Configure buzzer OC1 - put on pin 16 (RB7) 
  RPOR2bits.RP39R = 0b010000;
  // SPI(2) diofa manca il MISO sul pcb... serve per MFR
  RPOR1bits.RP37R = 0b001001;   // SCK
//  RPINR22bits.SCK2INR = 0b0100101  /*SERVE??*/;    // SCK in
  RPOR2bits.RP38R = 0b001000;   // SDO
  RPINR22bits.SDI2R = 0b0101010 /*RB10 pin 21 RP42*/ /*0b0100000*/ /*RPI32 pin 4 RB0*/;    // SDI 
  __builtin_write_OSCCONL(OSCCON | 0x40);
#endif


//	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_32);
	PR1=TMR1BASE;					// inizializzo TMR1
#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
	T1CON=0b1000000000110000;			// 1:256
#else
	T1CON=0b1000000000110000;			// 1:256
#endif
	IPC0bits.T1IP=3;
	IEC0bits.T1IE=1;

  
  BeepFreq=BEEP_STD_FREQ;

  T3CON=0b1000000000010000;
  PR3=10000;
    
  OC1CON2 = 0x0000;
  OC1RS   = 2187  /* se uso Timer come Src SEMBRA NON FARE NULLA... qua boh! */;  //PWM_PERIOD - 1;  /* set the period */
  OC1R    = 1093  /* basato su Timer3 @140/8=17.5MHz /* set the duty cycle tp 50% */;
  OC1CON2 = 0x001f;   /* 0x001F = Sync with This OC module                               */
  OC1CON1 = 0x0400 /* 0x0400 => src=Timer3 */;  /* 0x1C08 = Clock source Fcyc, trigger mode 1, Mode 0 (disable OC1) */
//  OC1CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ 




	ClrWdt();

#ifdef USA_232

	OpenUSART2(USART_RX_INT_ON & USART_TX_INT_OFF & USART_ASYNCH_MODE & 
		USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH,
		23);
	INTCONbits.PEIE = 1;			// attiva interrupt perif (SERVE x USART!
		:D
// v. skyusb18.c, e/oppure usare irq low/high
//	baudUSART(BAUD_16_BIT_RATE & BAUD_WAKEUP_OFF & BAUD_AUTO_OFF);
	// x= (Fosc / (Brate * 16 (BRGH=1) ) ) ) -1
	// Fosc=44236800 (11.0592 XTAL)
	
#endif

	


	ClrWdt();

init_i2c:

//	OpenI2C()
#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)
	I2C1CON= 0b0000001010000000;		// disable, 7bit, no stretch, No SlewRate, General Call enabled
  I2C1ADD = DEVICE_ADDRESS >> 1; // slave address
	I2C1MSK=0x0000		/*0x00ff  per general call , forse, ma non va cmq */;
	I2C1BRG=157; 		// 100KHz @32MHz (ma come slave non fa nulla)
	I2C1STAT=0;
  I2C1CONbits.I2CEN = 1; // enable I2C
  IFS1bits.SI2C1IF = 0;
  IPC4bits.SI2C1IP = 4;
	IEC1bits.SI2C1IE = 1; // enable SI2C1IF interrupt 
#else
	I2C1CON= 0b0000001010000000;		// disable, 7bit, no stretch, No SlewRate, General Call enabled
  I2C1ADD = DEVICE_ADDRESS >> 1; // slave address
	I2C1MSK=0x0000		/*0x00ff  per general call , forse, ma non va cmq */;
	I2C1BRG=157; 		// 100KHz @32MHz (ma come slave non fa nulla)
	PADCFG1=0;
	I2C1STAT=0;
  I2C1CONbits.I2CEN = 1; // enable I2C
  IFS1bits.SI2C1IF = 0;
  IPC4bits.SI2C1IP = 4;
	IEC1bits.SI2C1IE = 1; // enable SI2C1IF interrupt 
#endif




	if(InitOK & (1 << 3 /*NOT_TO*/))	 	// se era watchdog...
		{


		}

	__delay_ms(100);


  {
	BYTE *p=(BYTE *)&configParms;
	for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) 
		*p++=EEleggi(i);
	}


//	memcpy(RAMIcons,ROMIcons,sizeof(RAMIcons));



	__delay_ms(500);
				m_Led0Bit ^= 1;

	
  /*
  char buf[32];
  LCDCls();
  sprintf(buf,"RCON=%04x, test=%lu",RCON,testCrash);
  LCDWrite(buf);
  display();
  while(m_Puls1Bit)
    ClrWdt();
  RCON=0;
    */
	
  


	__delay_ms(200);
				m_Led0Bit ^= 1;
	__delay_ms(200);
				m_Led0Bit ^= 1;
	__delay_ms(200);


	ClrWdt();
				m_Led0Bit = 1;




warm_reset:

	FLAGS=0;						// salvare in EEPROM?
  FLAGK=0;

	ei();			// attiva interrupt globali


	Counter=0;
#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
	CounterL2=6;			// 140MHz int osc 2021
#else
	CounterL2=2;			// 32MHz int osc 2021
#endif
	CommStatus.w=0;			// 


#ifdef USA_232
	RCSTAbits.CREN=1;			// abilito ricevitore 232
#endif


	ClrWdt();							// rientro caldo (da watchdog):
	KBInit();



//	FLAGS;						// salvare in EEPROM?



	__delay_ms(250);
				m_Led0Bit ^= 1;
	__delay_ms(250);
				m_Led0Bit ^= 1;



	ClrWdt();				// DOPO!
//	LCDHome();
//		LCDPutChar('*');



	if(!m_Puls2Bit) {		// cold

debug_init:

;
		}



#ifdef USA_232
	IEC1bits.U2RXIE=1;				// ripristino IRQ 
#else
	IEC1bits.U2RXIE=0;				// ripristino IRQ 
#endif
	IEC1bits.SI2C1IE=1;


#ifdef USA_232
	Buf232Ptr=0;
	Buf232PtrI=0;
	RCSTAbits.CREN=1;				// abilito ricevitore 232...
#endif

#ifdef USA_232
	sendQueryStatus232();
#endif


	myPortC=PORTB;								// salvo situazione input 

/*{
static char buf[10];
LCDCls();
//sprintf(buf,"%u",PORTC);
//LCDWrite(buf);
LCDWriteROM("  ::");
sprintf(buf,"aa%u",LATC);
LCDWriteN(buf,4);
}*/

main:


/*while(1) {
	__delay_ms(100);
				m_Led0Bit ^= 1;
}*/



/************************************************************
; Main wait loop while idle.
;************************************************************/

  I2CCNT=0;
	PacketReceived=0;


	I2CFLG=0;						//Init state flags
//	m_Led0Bit = 0;
	// I2CSUBA viene pulito solo in seguito a uno stop, in modo che la BLOCK_READ (che usa uno start-ripetuto) puo' "riciclarlo" piu' volte...
	ei();								// riabilito tutti gli interrupt
//	bcf FLAGS,NOSCANK							; oppure basta questo...?
  
  
	while(1) {
    ClrWdt();                 //Clear watchdog timer (senza prescaler => tWDT=18mSec)

    if(PacketReceived) {
//			di();			// per evitare casini... BOH qua??
//LCDPrintHex(I2CFLG);
//LCDPrintHex(I2CSUBA);
			if(I2CFLG & (1 << B_RID)) {		// 
				switch(I2CSUBA & 0xfe) {
					case 6:		// https://www.i2c-bus.org/addressing/general-call-address/ https://www.nxp.com/docs/en/user-guide/UM10204.pdf
						asm("Reset");
						break;
					}
				}
			else if(!(I2CFLG & (1 << B_RD)))		// sarebbe sottinteso :)
				recvMsg();

//  		m_Led1Bit ^= 1;

/*{
char buf[16];
sprintf(buf,"I2C: %x %x %x ",I2CR0[0],I2CR0[1],I2CR0[2]);
			LCDWrite(buf);
display();
}*/

      PacketReceived = 0;
      I2CR0[0]=I2CR0[1]=0;			//ClearRXBuffer();
			I2CFLG=0;
			I2CSUBA=0;
			I2CCNT=0;
      }


		if(!--Counter)						// quando il contatore e' zero...
			doMain();								//Call user code while in idle state




#ifndef USA_232

#else

	doMain(); v. SOPRA ovviamente va rivista 2021 :)

	ch=GetByte232();

	if(ch != -1) {


		}

#endif

		}


#ifdef USA_232
	CloseUSART();
#endif
    
	} //end main



void doMain(void) {
	BYTE *p,*p2;
	BYTE i;
	BYTE tempC,tempC2;			// temporanea, usata da KB ecc.
	static BYTE clockDivider,animDivider;


//	Counter=0x7fff;

#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
	CounterL2--;        // quando il sub-contatore e' zero...
	if(!CounterL2) {
		CounterL2=6;			// 140MHz int osc 2021 => ~75mS
#ifdef USA_232
		Counter--;         // quando il contatore e' zero...
		if(!Counter)
#endif
			goto ResCnt;
		}
#else
	CounterL2--;        // quando il sub-contatore e' zero...
	if(!CounterL2) {
		CounterL2=2;			// 32MHz int osc 2021 => ~90mS

#ifdef USA_232
		Counter--;         // quando il contatore e' zero...
		if(!Counter)
#endif
			goto ResCnt;
		} 
#endif


		if(!m_RXDataBit)	{		//Check for slave request-to-send
      BYTE TEMP0;
//			__builtin_disableinterrupts();					//Request-to-send detected
			TEMP0=PS2_slavein();
			if(!(PS2Errors & 0x3)) {
				// o fare RESEND_PS2_slaveout
        if(SlavePresent & 0b00000001) { // se tastiera..
				if(!slaveHandler(TEMP0)) {    // se tasto...
//			__builtin_enableinterrupts();
          switch(TEMP0) {
            case 0xE0:   // beh finire!
              TEMP0=PS2_slavein();
              if(!slaveHandler(TEMP0)) {    // 
                }
              switch(TEMP0) {
                case 0x11:    // RALT (Gr)
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_ALT);
                  break;
                case 0x14:    // RCTRL
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_CONTROL);
                  break;
                case 0x1f:    // L-WIN
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_GUI);
                  break;
                case 0x27:    // R-WIN
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_GUI);
                  break;
                case 0x75:    // frecce
                  keyToSend[0]=0x94;    // up
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x74:
                  keyToSend[0]=0x91;    //right
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x72:
                  keyToSend[0]=0x93;    //down
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6b:
                  keyToSend[0]=0x92;    //left
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7a:
                  keyToSend[0]=0x95;    //pgdn
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7d:
                  keyToSend[0]=0x96;    //pgup
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6c:
                  keyToSend[0]=0x97;    //home
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x69:
                  keyToSend[0]=0x98;    //end
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x70:
                  keyToSend[0]=0x99;    //ins
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x71:
                  keyToSend[0]=0x9a;    //del
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x4a:
                  keyToSend[0]='/';    //keypad /
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x37:    //Power
                  keyToSend[0]=0xb9;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x3f:    //Sleep
                  keyToSend[0]=0xba;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x5e:    //Wake
                  keyToSend[0]=0xbb;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x12:
  // ARRIVANO DEI CAZZO DI 12 DOPO OGNI tasto-freccia ecc se NUMLOCK è ON!! boh?? tast. packard bell 24/6/22
//                PrtSc arriva come E0 12 E0 7C... SISTEMARE!
//                  keyToSend[0]=0xb8;    //PrtSc
//                  keyToSend[1]=getModifierKeys();
/*                  //quindi dovremmo aspettare il 7c e farlo secco!
                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();
                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();*/
                  break;
                case 0x7c:    // quindi prendo questo
                  keyToSend[0]=0xb8;    //PrtSc
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7e:    // Ctrl-Break(Pause)
                  keyToSend[0]=0x9f;
                  keyToSend[1]=getModifierKeys(); // forzare CTRL?? o non serve?
                  break;
                  // seguono multimedia.. http://users.utcluj.ro/~baruch/sie/labor/PS2/Scan_Codes_Set_2.htm
                case 0x20:    //www refresh
                  break;
                case 0x3a:    //www home
                  break;
                case 0x21:    //volume down
                  break;
                case 0x32:    //volume up
                  break;
                case 0x34:    //play/pause
                  break;
                case 0x2b:    //calculator
                  break;
                case 0x2f:    //apps
                  break;
                case 0x40:    //my computer
                  break;
                case 0x48:    //email
                  break;
                case 0xF0:    // 
                  goto F0_code1;    // bah diciamo
                  break;
                default:
                  keyToSend[0]=App_PS22ASCII(TEMP0);
                  keyToSend[1]=getModifierKeys();
                  break;
                }
              break;
            case 0xF0:   // break
F0_code1:
              TEMP0=PS2_slavein();
              if(!slaveHandler(TEMP0)) {    // 
                }
              switch(TEMP0) {
                case 0x11:    // LALT
                  PS2_BufferModifierKeys &= ~((1 << MODIFIER_LEFT_ALT) | (1 << MODIFIER_RIGHT_ALT));
                  // per ora li pulisco cmq entrambi!
                  break;
                case 0x12:    // LSHIFT
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_LEFT_SHIFT);
                  break;
                case 0x14:    // LCTRL
                  PS2_BufferModifierKeys &= ~((1 << MODIFIER_LEFT_CONTROL) | (1 << MODIFIER_RIGHT_CONTROL));
                  break;
                case 0x1f:    // L-WIN
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_LEFT_GUI);
                  break;
                case 0x27:    // R-WIN
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_RIGHT_GUI);
                  break;
                case 0x59:    // RSHIFT
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_RIGHT_SHIFT);
                  break;
                case 0x7c:  // PrtSc 
                  //quindi dovremmo aspettare il 7c O IL 12 e farlo secco!
//                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();
//                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();
                  TEMP0=PS2_slavein();
                  break;
                }
              break;
            case 0xE1:   // qua arriva Pause
              TEMP0=PS2_slavein();
              if(!slaveHandler(TEMP0)) {    // 
                }
              switch(TEMP0) {
								case 0x14:		// 0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xF0, 0x77
                  TEMP0=PS2_slavein();  // mangio il resto!
                  TEMP0=PS2_slavein();
                  TEMP0=PS2_slavein();
                  TEMP0=PS2_slavein();
                  TEMP0=PS2_slavein();
                  TEMP0=PS2_slavein();
                  keyToSend[0]=0x9f;
                  keyToSend[1]=getModifierKeys();
									break;
								case 0xF0:
									break;
								}
              break;
            default:
              switch(TEMP0) {
                case 0x11:    // ALT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_ALT);
                  break;
                case 0x12:    // LSHIFT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_SHIFT);
                  break;
                case 0x14:    // CTRL
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_CONTROL);
                  break;
                case 0x59:    // RSHIFT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_SHIFT);
                  break;
                case 0x75:    // frecce
                  keyToSend[0]=NUM_Lock_Pressed() ? '8' : 0x94;    // up
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x74:
                  keyToSend[0]=NUM_Lock_Pressed() ? '6' : 0x91;    //right
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x72:
                  keyToSend[0]=NUM_Lock_Pressed() ? '2' : 0x93;    //down
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6b:
                  keyToSend[0]=NUM_Lock_Pressed() ? '4' : 0x92;    //left
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7a:
                  keyToSend[0]=NUM_Lock_Pressed() ? '3' : 0x95;    //pgdn
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7d:
                  keyToSend[0]=NUM_Lock_Pressed() ? '9' : 0x96;    //pgup
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6c:
                  keyToSend[0]=NUM_Lock_Pressed() ? '7' : 0x97;    //home
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x69:
                  keyToSend[0]=NUM_Lock_Pressed() ? '1' : 0x98;    //end
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x70:
                  keyToSend[0]=NUM_Lock_Pressed() ? '0' : 0x99;    //ins
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x71:
                  keyToSend[0]=NUM_Lock_Pressed() ? '.' : 0x9a;    //del
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x77:
                  Appl_led_report_buffer.NUM_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb1;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x58:
                  Appl_led_report_buffer.CAPS_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb2;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7e:
                  Appl_led_report_buffer.SCROLL_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb3;
                  keyToSend[1]=getModifierKeys();
                  break;
                default:
                  keyToSend[0]=App_PS22ASCII(TEMP0);
                  keyToSend[1]=getModifierKeys();
                  break;
                }
              break;
            }
          }
        else {    // potrebbe essere hot-plug...
          }
          }
        }
			}

		if(!m_RXDataBit2)	{		//Check for slave request-to-send
      BYTE TEMP0;
//			__builtin_disableinterrupts();					//Request-to-send detected
			TEMP0=PS2_slavein2();
			if(!(PS2Errors & 0xc)) {
				// o fare RESEND_PS2_slaveout
        if(SlavePresent & 0b00000100) {   // se tastiera..
				if(!slaveHandler2(TEMP0)) {    // se tasto...
//			__builtin_enableinterrupts();
          switch(TEMP0) {
            case 0xE0:   // beh finire!
              TEMP0=PS2_slavein2();
              if(!slaveHandler2(TEMP0)) {    // 
                }
              switch(TEMP0) {
                case 0x11:    // RALT (Gr)
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_ALT);
                  break;
                case 0x14:    // RCTRL
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_CONTROL);
                  break;
                case 0x1f:    // L-WIN
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_GUI);
                  break;
                case 0x27:    // R-WIN
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_GUI);
                  break;
                case 0x75:    // frecce
                  keyToSend[0]=0x94;    // up
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x74:
                  keyToSend[0]=0x91;    //right
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x72:
                  keyToSend[0]=0x93;    //down
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6b:
                  keyToSend[0]=0x92;    //left
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7a:
                  keyToSend[0]=0x95;    //pgdn
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7d:
                  keyToSend[0]=0x96;    //pgup
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6c:
                  keyToSend[0]=0x97;    //home
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x69:
                  keyToSend[0]=0x98;    //end
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x70:
                  keyToSend[0]=0x99;    //ins
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x71:
                  keyToSend[0]=0x9a;    //del
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x4a:
                  keyToSend[0]='/';    //keypad /
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x37:    //Power
                  keyToSend[0]=0xb9;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x3f:    //Sleep
                  keyToSend[0]=0xba;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x5e:    //Wake
                  keyToSend[0]=0xbb;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x12:
  // ARRIVANO DEI CAZZO DI 12 DOPO OGNI tasto-freccia ecc se NUMLOCK è ON!! boh?? tast. packard bell 24/6/22
//                PrtSc arriva come E0 12 E0 7C... SISTEMARE!
//                  keyToSend[0]=0xb8;    //PrtSc
//                  keyToSend[1]=getModifierKeys();
/*                  //quindi dovremmo aspettare il 7c e farlo secco!
                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();
                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein();*/
                  break;
                case 0x7c:    // quindi prendo questo
                  keyToSend[0]=0xb8;    //PrtSc
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7e:    // Ctrl-Break(Pause)
                  keyToSend[0]=0x9f;
                  keyToSend[1]=getModifierKeys(); // forzare CTRL?? o non serve?
                  break;
                  // seguono multimedia.. http://users.utcluj.ro/~baruch/sie/labor/PS2/Scan_Codes_Set_2.htm
                case 0x20:    //www refresh
                  break;
                case 0x3a:    //www home
                  break;
                case 0x21:    //volume down
                  break;
                case 0x32:    //volume up
                  break;
                case 0x34:    //play/pause
                  break;
                case 0x2b:    //calculator
                  break;
                case 0x2f:    //apps
                  break;
                case 0x40:    //my computer
                  break;
                case 0x48:    //email
                  break;
                case 0xF0:    // 
                  goto F0_code2;    // bah diciamo
                  break;
                default:
                  keyToSend[0]=App_PS22ASCII(TEMP0);
                  keyToSend[1]=getModifierKeys();
                  break;
                }
              break;
            case 0xF0:   // break
F0_code2:
              TEMP0=PS2_slavein2();
              if(!slaveHandler(TEMP0)) {    // 
                }
              switch(TEMP0) {
                case 0x11:    // LALT
                  PS2_BufferModifierKeys &= ~((1 << MODIFIER_LEFT_ALT) | (1 << MODIFIER_RIGHT_ALT));
                  // per ora li pulisco cmq entrambi!
                  break;
                case 0x12:    // LSHIFT
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_LEFT_SHIFT);
                  break;
                case 0x14:    // LCTRL
                  PS2_BufferModifierKeys &= ~((1 << MODIFIER_LEFT_CONTROL) | (1 << MODIFIER_RIGHT_CONTROL));
                  break;
                case 0x1f:    // L-WIN
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_LEFT_GUI);
                  break;
                case 0x27:    // R-WIN
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_RIGHT_GUI);
                  break;
                case 0x59:    // RSHIFT
                  PS2_BufferModifierKeys &= ~(1 << MODIFIER_RIGHT_SHIFT);
                  break;
                case 0x7c:  // PrtSc 
                  //quindi dovremmo aspettare il 7c O IL 12 e farlo secco!
//                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein2();
//                  while(m_RXDataBit) ClrWdt();
                  TEMP0=PS2_slavein2();
                  TEMP0=PS2_slavein2();
                  break;
                }
              break;
            case 0xE1:   // qua arriva Pause
              TEMP0=PS2_slavein2();
              if(!slaveHandler(TEMP0)) {    // 
                }
              switch(TEMP0) {
								case 0x14:		// 0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xF0, 0x77
                  TEMP0=PS2_slavein2();  // mangio il resto!
                  TEMP0=PS2_slavein2();
                  TEMP0=PS2_slavein2();
                  TEMP0=PS2_slavein2();
                  TEMP0=PS2_slavein2();
                  TEMP0=PS2_slavein2();
                  keyToSend[0]=0x9f;
                  keyToSend[1]=getModifierKeys();
									break;
								case 0xF0:
									break;
								}
              break;
            default:
              switch(TEMP0) {
                case 0x11:    // ALT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_ALT);
                  break;
                case 0x12:    // LSHIFT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_SHIFT);
                  break;
                case 0x14:    // CTRL
                  PS2_BufferModifierKeys |= (1 << MODIFIER_LEFT_CONTROL);
                  break;
                case 0x59:    // RSHIFT
                  PS2_BufferModifierKeys |= (1 << MODIFIER_RIGHT_SHIFT);
                  break;
                case 0x75:    // frecce
                  keyToSend[0]=NUM_Lock_Pressed() ? '8' : 0x94;    // up
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x74:
                  keyToSend[0]=NUM_Lock_Pressed() ? '6' : 0x91;    //right
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x72:
                  keyToSend[0]=NUM_Lock_Pressed() ? '2' : 0x93;    //down
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6b:
                  keyToSend[0]=NUM_Lock_Pressed() ? '4' : 0x92;    //left
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7a:
                  keyToSend[0]=NUM_Lock_Pressed() ? '3' : 0x95;    //pgdn
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7d:
                  keyToSend[0]=NUM_Lock_Pressed() ? '9' : 0x96;    //pgup
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x6c:
                  keyToSend[0]=NUM_Lock_Pressed() ? '7' : 0x97;    //home
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x69:
                  keyToSend[0]=NUM_Lock_Pressed() ? '1' : 0x98;    //end
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x70:
                  keyToSend[0]=NUM_Lock_Pressed() ? '0' : 0x99;    //ins
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x71:
                  keyToSend[0]=NUM_Lock_Pressed() ? '.' : 0x9a;    //del
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x77:
                  Appl_led_report_buffer.NUM_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb1;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x58:
                  Appl_led_report_buffer.CAPS_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb2;
                  keyToSend[1]=getModifierKeys();
                  break;
                case 0x7e:
                  Appl_led_report_buffer.SCROLL_LOCK ^= 1;
                  if(ledStatusLocal) {
                    syncPS2Leds();
                    }
                  keyToSend[0]=0xb3;
                  keyToSend[1]=getModifierKeys();
                  break;
                default:
                  keyToSend[0]=App_PS22ASCII(TEMP0);
                  keyToSend[1]=getModifierKeys();
                  break;
                }
              break;
            }
          }
        else {    // potrebbe essere hot-plug...
          }
          }
  			}
			}
    
    if(keyToSend[0]) {    // in teoria potrei volere anche solo i modifier... (e i break code ovviamente!)
//      notifyToCPU(EVENT_KEYBOARD,keyToSend,2);
      
      keyToSend[0]=keyToSend[1]=0;
      }


#ifdef USA_TASTIERA

	if(FLAGK & (1 << TRIGK)) {		// se c'e' stata una scansione di tastiera...

		FLAGS |= (1 << NOSCANK); 		//Disable button/sensor sampling
		FLAGK &= ~(1 << TRIGK);

		p=kbKeys;
		for(tempC=0; tempC<MAX_TASTI_CONTEMPORANEI; tempC++) {
			ClrWdt();
			if(*p) {							// se c'e' un tasto premuto...

				p2=kbKeysOld;
				for(tempC2=0; tempC2<MAX_TASTI_CONTEMPORANEI; tempC2++) {
					if(*p2 == *p) {
						goto cercaKeys2;			//...e questo tasto prima non era premuto...
						}
					p2++;
					}
				i=GetKBchar(*p);				// lo trascodifico e...
				if(i) {								// (se e' un tasto "vero")
					MandaTastoPremuto(i);			//...lo mando...
//m_Led0Bit ^= 1;
//LCDPutChar(i);
					}

#ifdef USA_KB_BUZZER
// suonare bene!
				if(!(FLAGS & (1 << NOKCLICK))) {			// se voglio key-click...
    		  OC1CON1 |= 0x0006;
					FLAGS |= (1 << BUZZON);				// verra' spento da interrupt
          TimerBuzz=2;
					}
#endif

				}		// if(*p)

cercaKeys2:
			p++;
			}


		p=kbKeysOld;
		for(tempC=0; tempC<MAX_TASTI_CONTEMPORANEI; tempC++) {
			ClrWdt();
			if(*p) {							// se c'e' un tasto che prima era premuto...

				p2=kbKeys;
				for(tempC2=0; tempC2<MAX_TASTI_CONTEMPORANEI; tempC2++) {
					if(*p2 == *p) {
						goto cercaKeysRil2;			//...e ora non lo è più...
						}
					p2++;
					} 

				i=GetKBchar(*p);				// lo trascodifico e...
				if(i) {								// (se e' un tasto "vero")
					MandaTastoRilasciato(i);			//...lo mando...
					}

				}		// if(*p)

cercaKeysRil2:
			p++;
			}

// copio la situazione tasti attuale nello storico
//		memcpy(&kbKeysOld,&kbKeys,MAX_TASTI_CONTEMPORANEI);
		kbKeysOld[0]=kbKeys[0];		// mi sa che è più veloce così...
		kbKeysOld[1]=kbKeys[1];
#if MAX_TASTI_CONTEMPORANEI>2
		kbKeysOld[2]=kbKeys[2];
		kbKeysOld[3]=kbKeys[3];
#endif

		FLAGS &= ~(1 << NOSCANK);		//Enable button/sensor sampling
		}

#endif				// USA_TASTIERA




	return;


ResCnt:			// circa 150mS su FJ, 27/1/21; 100mS su EP 12/4/21

  

	m_Led0Bit ^= 1;



no_clock:


//	if(m_Led0Bit)			// così son 300mS! display impiega circa 1.5mS @ 8MHz SPI NO! v. sopra


	// salvo situazione input 
	myPortC=PORTB;


	if(TimerBuzz) {
		if(!--TimerBuzz) {
			// spengo PWM1 (faccio in modo che non si attivi mai... v. anche PR2)
		  OC1CON1 &= ~0x0006;
#ifdef USA_KB_BUZZER
    	if(FLAGS & (1 << BUZZON)) {				// OCCHIO alla contemporaneita' col BEEP (chr$7)
    //OVVIAMENTE no per Bolymin!!
        FLAGS &= ~(1 << BUZZON);
  			}
#endif
			}
		}

	if(minute_1) {
		if(disconnected<=5)
			disconnected++;
		minute_1=0;
		}

	}		// doMain



// ---------------------------------------------------------------------------------

BYTE recvMsg(void) {
	BYTE *p;
	BYTE i;
	WORD n;

	disconnected=0;
	I2CCNT--;			// perché qua conta anche il subaddress..


// ANDREBBE FATTO SU TUTTI, direi...			if(I2CCNT >= 4)				// 

#ifdef I2C_SMBUS
	p=I2CR0+1;
#else
	p=I2CR0;
#endif
	switch(I2CSUBA) {
		case 0:
#ifdef I2C_SMBUS
			if(I2CCNT != 4)				// qui devono essere proprio 4 
				return 0;						// ...altrimenti non eseguo!
#else
			if(I2CCNT >= 3) {				// qui devono essere almeno 3
#endif
			return 1;
			}
			break;

		case 1:
			break;

		case 2:
      if(SlavePresent & 0b00000001) {
        PS2_slaveout(0xF3);
        i=PS2_slavein();
        PS2_slaveout(((keyboardTypematic1 & 0b11) << 5) | (keyboardTypematic2 & 0b00011111));
        i=PS2_slavein();
        slaveHandler(i);
        }
      if(SlavePresent & 0b00000010) {
        PS2_slaveout2(0xF3);
        i=PS2_slavein2();
        PS2_slaveout2(((keyboardTypematic1 & 0b11) << 5) | (keyboardTypematic2 & 0b00011111));
        i=PS2_slavein2();
        slaveHandler2(i);
        }
			break;

		case 3:						// cambiare ?
			break;

		case 4:
			break;

		case 5:
recvMsg_setLight_Clock_Buzz:
#ifdef I2C_SMBUS
			if(I2CCNT != 5)			// qui devono essere proprio 5 
				return 0;
#else
			if(I2CCNT >= 4) {			// qui devono essere almeno 4
#endif

//	movfw		I2CR0			; qui ci sarebbe il cnt.BYTE (in modalita' BLOCK_WRITE SmBus)

			if(*p & 0x80)				//1=no! (per lasciare default=sì...)
				FLAGS |= (1 << NOKCLICK);
			else
				FLAGS &= ~(1 << NOKCLICK);

			p++;
			if(*p) {
				BeepFreq = *p * 20;		// ~2100 * 4KHz su PIC24EP
				}


// questo sia SERIALE che I2C
			return 1;
			}

			break;

		case 6:
recvMsg_setLed:
#ifdef I2C_SMBUS
//	movfw		I2CR0			; qui ci sarebbe il cnt.BYTE (in modalita' BLOCK_WRITE SmBus)
			p++;
#else
#endif
//			LATB=(*p & 1) << 3;		//  tanto per
			return 1;
			break;

		case 7:
			return 0;
			break;

		case 8:
			break;

		case 9:
			break;

		case 10:					// 
			return 1;
			break;

		case 11:			// 
			break;

		case 12:			// write flash / RAM icons, o imposta ptr per read (v.)
			break;

		case 13:			// "pulisci" 2017, scrivo riga ma prima la sbianchetto (al cursore)
			break;

		case 14:
			return 0;
			break;

		case 15:
			return 0;
			break;

		case 16:
			break;

		case 32:
//			StdBeep();
			Beep(*p);
			break;

		case 128:			// save EEprom
			p=(BYTE *)&configParms;
			for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) 
				EEscrivi_(i,*p++);
			return 1;
			break;

		case 160:			// bootloader... finire
		case 161:			// 
		case 162:
		case 163:
		case 164:
		case 165:
		case 166:
		case 167:
			switch(I2CSUBA & 7) {
        case 1:   // leggi versione v. bootloader MiWi!!
          break;
        case 2:   // leggi device
          break;
        case 3:   // erase
          break;
        case 4:   // program
          break;
        case 5:   // verify
          break;
        case 7:   // entra bootloader CONTROLLARE CHIAVE!
          break;
        }
			return 1;
			break;
      
		default:				// usato come HeartBeat :)
			return 0;
			break;

		}

	return 0;
	}




// ---------------------------------------------------------------------

BYTE KBInit(void) {

	TRISAbits.TRISA0=1;
	TRISAbits.TRISA1=1;
  ANSELAbits.ANSA0=0;
  ANSELAbits.ANSA1=0;

	InitPS2(1);

	return 1;
	}


void kbEmptyBuf(void) {

	kbKeys[0]=0;
	kbKeys[1]=0;
#if MAX_TASTI_CONTEMPORANEI>2
	kbKeys[2]=0;
	kbKeys[3]=0;
#endif
	}

// ----------------------------------------------------------------------------

BYTE checkKey(void) {
	BYTE *p;
	BYTE n;

	p=kbKeys;
	for(n=0; n<MAX_TASTI_CONTEMPORANEI; n++) {
		ClrWdt();
		if(!*p) {							//	cerco un posto vuoto x metterci lo scan code
			*p=kbScanCode;
			goto checkKey_fine;
			}
		p++;
		}

#ifdef USA_KB_BUZZER
// suonare male!
	OC1RS=BEEP_LOW_FREQ;
	OC1R=(BEEP_LOW_FREQ)/2;

  OC1CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ 
	__delay_ms(25);
  OC1CON1 &= ~0x0006;   
#endif

	return 0;
	// non ho più posto (max 4 tasti contemporanei!) dare errore!! (tutti 0x1)

checkKey_fine:

	return 1;									// 
	}


BYTE GetKBchar(BYTE n) {

	switch(n) {				// gli scan-code partono da 1...
		case 1:		return '0';		break;
		case 2:		return '1';		break;
		case 3:		return '2';		break;
		case 4:		return '3';		break;
		case 5:		return '4';		break;
		case 6:		return '5';		break;
		case 7:		return '6';		break;
		case 8:		return '7';		break;
		case 9:		return '8';		break;
		case 10:		return '9';		break;
		case 11:		return 'A';		break;
		case 12:		return 'B';		break;
		case 13:		return 'C';		break;
		case 14:		return 'D';		break;
		case 15:		return 'E';		break;
		case 16:		return 'F';		break;
		default:		return 0;		break;
	
		}
	}



void MandaTastoPremuto(BYTE n) {				// entra W=temp=char

#ifdef USA_232
	PutByte232(n);
#else
	// avvisa I2C di leggere un pacchetto
	IEC1bits.SI2C1IE = 0; // 
  I2C1CONbits.I2CEN = 0; // 
	__delay_us(10);
  m_I2CslvDBit=0;
//	LATB  &= ~0b0000001000000000;					// output 0
  I2CslvDTRIS=0;
//	TRISB &= ~0b0000001000000000;					// output 0
	__delay_us(10);
  m_I2CslvDBit=1;
//	LATB  |=  0b0000001100000000;					// output 0
  I2CslvDTRIS=1;
//	TRISB |=  0b0000001000000000;					// output 1
	__delay_us(10);
  I2C1CONbits.I2CEN = 1; // 
	IEC1bits.SI2C1IE = 1; // 
	
#endif

//	if(!m_Puls2Bit)
//		LCDPutChar(n);

	//goto  MandaTastoPremuto2;

	}

void MandaTastoPremuto2(void) {
	}

void MandaTastoRilasciato(BYTE n) {
	}


void syncPS2Leds(void) {
  PS2_LED_REPORT_BUFFER psl;
  int i;
  
  psl.b=0;
  psl.CAPS_LOCK= Appl_led_report_buffer.CAPS_LOCK;// dio cheffroci!
  psl.NUM_LOCK = Appl_led_report_buffer.NUM_LOCK;
  psl.SCROLL_LOCK = Appl_led_report_buffer.SCROLL_LOCK;
  if(SlavePresent & 0b00000001) {
    PS2_slaveout(0xED);   // SetLed
    i=PS2_slavein();
    PS2_slaveout(psl.b);
    i=PS2_slavein();
    slaveHandler(i);
    }
  if(SlavePresent & 0b00000100) {
    PS2_slaveout2(0xED);   // SetLed
    i=PS2_slavein2();
    PS2_slaveout2(psl.b);
    i=PS2_slavein2();
    slaveHandler2(i);
    }
  }



// ---------------------------------------------------------------------

void StdBeep(void) {

  Beep(10);
	}

void Beep(BYTE t) {

  t &= 31;      // per sicurezza :)
  
	OC1RS=BEEP_STD_FREQ;
	OC1R=BEEP_STD_FREQ/2;

  OC1CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ 
	__delay_ms(t*100);
  OC1CON1 &= ~0x0006;   
	}

void ErrorBeep(void) {

	OC1RS=BEEP_LOW_FREQ;
	OC1R=(BEEP_LOW_FREQ)/2;

  OC1CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ 
	Delay_S();
  OC1CON1 &= ~0x0006;   
	}



// ---------------------------------------------------------------------

#ifdef USA_232

short int GetByte232(void) {                // torna 0 e NC se niente oppure C e il BYTE in A
	BYTE ch;

  ClrWdt();
	if(Buf232Ptr == Buf232PtrI) {
		return -1;
		}

	ch=Buf232[Buf232PtrI++];
	Buf232PtrI &= 0xf;				// max 16
	return ch;
	}

void PutByte232(BYTE Byte2Send) {                // manda un BYTE

	do {
		ClrWdt();
		} while(!PIR1bits.TXIF);			// aspetto che il buffer di TX sia libero (v. anche MandaHdr)

	WriteUSART(Byte2Send);

	do {
		ClrWdt();
		} while(BusyUSART());

	}

#endif

// ---------------------------------------------------------------------



// ---------------------------------------------------------------------

#define Delay01() Delay_uS(1		/* TIME_GRANULARITY*/)			// 1 bit-time




void SetBeep(void) {									// asincrono, viene poi resettato da interrupt

	OC1RS=BeepFreq;
	OC1R=BeepFreq >> 1;		// /2 (duty cycle 50%)

// [was: il problema è che cambiando freq. qui, cambia anche quella del var. luce LCD...
// quindi occorre aggiornare quel valore CCPR2L
// Cio' viene fatto in aggLuceContrasto, che viene chiamata ANCHE quando si setta freq. buzz. con il medesimo cmd. I2C ]

 
  OC1CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ 

	TimerBuzz=5;
	}






#ifdef USA_232

// ---------------------------------------------------------------------
// calcola la parita: entra W, esce 0 se E e 1 se O
BYTE GetParita(BYTE n) {
	BYTE p=0;

#ifdef DARIO
	p=n >> 1;			// Microchip AN774
	p ^= n;
	n= p >> 2;
	p ^= n;
	n= p >> 4;
	p ^= n;
	return (p & 1);
#endif

	if(n & 1)
		p++;
	if(n & 2)
		p++;
	if(n & 4)
		p++;
	if(n & 8)
		p++;
	if(n & 16)
		p++;
	if(n & 32)
		p++;
	if(n & 64)
		p++;
	if(n & 128)
		p++;
  return p;
	}


#endif


#if defined(__PIC24FV32KA302__) || defined(__PIC24FV32KA301__)
//__HAS_EEDATA__ forse c'è solo su XC...
struct SAVED_PARAMETERS __attribute__ ((space(eedata))) eeData =
#else
#warning NON C'E' EEPROM!
const struct SAVED_PARAMETERS eeData =
#endif
	{
	1,		//typematic
	15,		//
	1,		//Flags

	};

// -------------------------------------------------------------------------------------

void EEscrivi_(BYTE addr,BYTE n) {	// 
/*---------------------------------------------------------------------------------------------
The variable eeData must be a Global variable declared outside of any method
the code following this comment can be written inside the method that will execute the write
-----------------------------------------------------------------------------------------------
*/
	unsigned int offset,temp;

	addr = (((unsigned int)addr) & 0x1ff);	// 512 byte EEprom

// si potrebbe fare un'altra funzione che scriva delle Word...

	// Set up NVMCON to erase one word of data EEPROM
	NVMCON = 0x4004;		// STRANO, secondo il datasheet di questo PIC dovrebbe essere 0x4080
	// Set up a pointer to the EEPROM location to be erased
	TBLPAG = __builtin_tblpage(&eeData); 				// Initialize EE Data page pointer
	offset = /*0xfe00;*/ __builtin_tbloffset(&eeData); 			// Initialize lower word of address
	offset+=addr;
	temp=__builtin_tblrdl(offset);
	if(((unsigned int)addr) & 1)
		temp = MAKEWORD(LOBYTE(temp), n);
	else
		temp = MAKEWORD(n, HIBYTE(temp));
	__builtin_tblwtl(offset, temp); 								// Write EEPROM data to write latch
	__builtin_disi(5); 												// Disable Interrupts For 5 Instructions
	__builtin_write_NVM(); 											// Issue Unlock Sequence & Start Write Cycle
	while(NVMCONbits.WR==1); 									// Optional: Poll WR bit to wait for
																							// write sequence to complete

//  _write_eedata_word(0x7FFe00+addr, n);
//	_wait_eedata();
// provare!    
//http://www.microchip.com/forums/m416332.aspx

  }

BYTE EEleggi(BYTE addr) {
/*--------------------------------------------------------------------------------------------
The variable eeData must be a Global variable declared outside of any method
the code following this comment can be written inside the method that will execute the read
----------------------------------------------------------------------------------------------
*/
	unsigned int offset;

	addr = (((unsigned int)addr) & 0x1ff);	// 512 byte EEprom

	// Set up a pointer to the EEPROM location to be erased
	TBLPAG = __builtin_tblpage(&eeData); 		// Initialize EE Data page pointer
	offset = __builtin_tbloffset(&eeData); 	// Initialize lower word of address
	offset += addr;
	return (((unsigned int)addr) & 1) ? __builtin_tblrdl(offset) >> 8 : __builtin_tblrdl(offset); // read EEPROM data from write latch

//  data=_read_eedata_word(addr);
	}



/**********************************************************************
;* I2C Device routines
;*
;* Enable must be HIGH, else state goes to 0
;* write is to me, read is from me.
;*
;*            <============== first BYTE / subsequant writes =====> <end>
;*
;* SDA   --|   X-----X-----X-----X-----X-----X-----X-----X---X------|  |--
;*         |---X-----X-----X-----X-----X-----X-----X-----X---X******|--|
;* (bit)   s    7     6     5     4     3     2     1     0   ackout
;* SCL  -----|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |---
;*           |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|
;*
;* STATE: 0 1 2  3  2  3  2  3  2  3  2  3  2  3  2  3  2  4  5  6  2  3 0
;*
;*            <============== subsequant reads ===================>
;*
;* SDA     X-----X-----X-----X-----X-----X-----X-----X----X------X-----
;*       --X-----X-----X-----X-----X-----X-----X-----X----X******X-----
;* (bit)ack   7     6     5     4     3     2     1     0   ackin
;* SCL  --|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--
;*        |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|
;*
;* STATE:  7  8  7  8  7  8  7  8  7  8  7  8  7  8  7  8  9  A  7  8
;*
;*            <============== Final READ =========================>
;*
;* SDA     X-----X-----X-----X-----X-----X-----X-----X----X-|**|-------
;*       --X-----X-----X-----X-----X-----X-----X-----X----X*|--|
;* (bit)ack   7     6     5     4     3     2     1     0   ackin
;* SCL  --|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |---------
;*        |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|
;*
;* STATE:  7  8  7  8  7  8  7  8  7  8  7  8  7  8  7  8  9  A  0  0
;*
;* STATE B is an ignore bit state for non-addressed bits
;* STATE C indicates last sample had ENA low
;* on rising edge of ENA, DATA LOW = low voltage, DATA&CLOCK low = RESET
;**********************************************************************

; tempi/frequenze tipiche di I2C/SMBus:
; 100KHz è OK per MicroChip 24LC16 (memorie) e TC74 (temp); Sensirion SHT11 va fino a oltre 1MHz; l'SMBus del chipset VIA andava a 16KHz; il clock-generator per PC ICS9248-95 va al max a 100KHz

; indirizzi usati:
; 1001xxxR ... Microchip TC74 temp-A/D (typ. 1001101R)
; 1001000R ... Microchip TC1320 D/A
; 1010xxxR ... Microchip Flash 24LC..
; 1001xxxR ... Microchip A/D MCP3221..	(typ. 1001101R)
; 1010xxxR ... ST EEPROM M34C02 ..
; 0110xxxR ...   ST EEPROM M34C02 (protect register)..
; .. il potenziometro digitale MCP41xxx/42xxx è SPI!
; 1001111R ... Dallas DS1629 termometro
; .. il DS1722 è SPI e il DS1820-1822 sono one-wire
; 00000xxR ... Sensirion SHT11 (non è proprio standard...)
; 011100AR ... MAX5821 (dual D/A) (modello 1; VADD sceglie indirizzo)
; 101100AR ...	 MAX5821 (dual D/A) (modello 2; VADD sceglie indirizzo)
; 1100100R ... MAX1036/7 (quad A/D)
; 1100101R ...	 MAX1038/9 (12 A/D)
; .. MAX1062 (A/D 14bit) è SPI/microwire
; 10000xxR ... LM4832 (amplificatore audio stereo National)
; 1111000R ... Sensore di Pressione I2C Sensortechnics
; 0100011R ... WatchDog ADPM (46h-47h!)
; 1101001R ... Clock-generator ICS9248-95 (D2h)
; 0100100R ... Display LCD ADPM (48h)
; 0100101R ... RFid TAG reader ADPM (4Ah-4Bh)

; Roba Intel (da internet)
;I2CRAM     EQU     0AEh           ;Slave address for PCF8570 RAM chip.
;I2CIO      EQU     4Eh            ;Slave address for PCF8574 I/O expandor.
;I2CLED     EQU     76h            ;Slave address for SAA1064 LED driver.


*************************************************************************/





/** EOF picbell24.c *********************************************************/

