#ifdef __XC16
#include <xc.h>
#else
#include <p24fxxxx.h>
#endif
//#include <portb.h>
//#include "timer.h"
#include "i2c_ps2.h"
#include "io_cfg.h"
#include <libpic30.h>

/** D E C L A R A T I O N S **************************************************/



extern volatile BYTE Clock_10;
extern volatile BYTE second_1,minute_1,hour_1,day_1;





#define TRUE            1
#define FALSE           0

volatile BYTE I2CR0[16 /*RXBUFFER_SIZE*/];
volatile BYTE PacketReceived;
volatile BYTE I2CFLG;              //I2C flag reg
BYTE I2CREG;							//I2C I/O register
volatile BYTE I2CCNT;									//contatore per lettura/scrittura buffer I2C
extern volatile BYTE I2CSUBA;		              //Subaddress
volatile BYTE I2CTimeout;									//timeout/watchdog




void __attribute__ (( interrupt, /*shadow, solo UNO! */ no_auto_psv )) _CNInterrupt(void) {
  
	IFS1bits.CNIF = 0;
  }

void __attribute__ (( interrupt, shadow, no_auto_psv )) _SI2C1Interrupt(void) {
	WORD n;

// https://www.microchip.com/forums/m920858.aspx
// https://www.microchip.com/forums/m1085150.aspx
// https://engineering.purdue.edu/ece477/Archive/2009/Spring/S09-Grp06/Code/PIC/pic24_code_examples/docs/i2c__slave__reverse__string_8c-source.html
/*typedef enum  {
STATE_WAIT_FOR_ADDR,
STATE_WAIT_FOR_WRITE_DATA,
STATE_SEND_READ_DATA,
STATE_SEND_READ_LAST
} STATE;

volatile STATE e_mystate = STATE_WAIT_FOR_ADDR;
#define BUFSIZE 64
volatile char  sz_1[BUFSIZE+1];
volatile char  sz_2[BUFSIZE+1];
volatile uint16 u16_index;


void _ISRFAST _SI2C1Interrupt(void) {
uint8 u8_c;
_SI2C1IF = 0;
switch (e_mystate) {
case STATE_WAIT_FOR_ADDR:
u8_c = I2C1RCV; //clear RBF bit for address
u16_index = 0;
//check the R/W bit and see if read or write transaction
if (I2C1STATbits.R_W) {
I2C1TRN = sz_2[u16_index++];  //get first data byte
I2C1CONbits.SCLREL = 1;     //release clock line so MASTER can drive it
e_mystate = STATE_SEND_READ_DATA; //read transaction
} else e_mystate = STATE_WAIT_FOR_WRITE_DATA;
break;
case STATE_WAIT_FOR_WRITE_DATA:
//character arrived, place in buffer
sz_1[u16_index++] = I2C1RCV;  //read the byte
if (sz_1[u16_index-1] == 0) {
//have a complete string, reverse it.
reverseString(sz_1,sz_2);
e_mystate = STATE_WAIT_FOR_ADDR; //wait for next transaction
}
break;
case STATE_SEND_READ_DATA:
//just keep placing reversed characters in buffer as MASTER reads our I2C port
I2C1TRN = sz_2[u16_index++];
I2C1CONbits.SCLREL = 1;     //release clock line so MASTER can drive it
if (sz_2[u16_index-1] == 0) e_mystate = STATE_SEND_READ_LAST;
//this is the last character, after byte is shifted out, release the clock line again
break;
case STATE_SEND_READ_LAST:  //this is interrupt for last character finished shifting out
e_mystate = STATE_WAIT_FOR_ADDR;
break;
default:
e_mystate = STATE_WAIT_FOR_ADDR;
}
*/

//n=I2C1STAT;
// http://www.ccsinfo.com/forum/viewtopic.php?t=59033&start=0

	if(I2C1STATbits.GCSTAT) {		
		I2CFLG |= (1 << B_RID);			// ma sembra non arrivare cmq a meno di non azzerare la MASK...
		}

/*	if(I2C1STATbits.P) {		// questo NON avviene MAI in interrupt... v. link sopra. Serve polling
    PacketReceived = TRUE;
		}*/

	if(I2C1STATbits.S) {		// questo avviene SEMPRE durante il messaggio!
		I2CFLG |= (1 << B_ST);		// inutile... tanto per
//    I2CCNT=0;
		}

receive_i2c_loop:

  if(I2C1STATbits.RBF == 1)
    I2CREG = I2C1RCV; // RCV buffer cleared
    
//  if(I2C1STATbits.ACKSTAT == 1) // NACK received
//    goto fine;
    
  if(I2C1STATbits.R_W == 0) { // write operation
    if(I2C1STATbits.D_A == 1) { // valid data
			I2CFLG &= ~(1 << B_RD);
			if(!I2CCNT) {
				I2CFLG |= (1 << B_SA);
				I2CSUBA=I2CREG;
				}
			else
      	I2CR0[I2CCNT-1] = I2CREG;
			I2CCNT++;
			}
		else {		// address received
			I2CFLG |= (1 << B_UA);
			T4CON=0x8030;			// 1:256, 4uS
			TMR4=0;
			PR4=1200;			//4mSec timeout
			IFS1bits.T4IF=0;
			I2CTimeout=1;
#ifdef PCB2021
			m_Led1Bit ^= 1;
#endif
			}

do_receive_i2c_loop:
		if(I2CCNT >= sizeof(I2CR0))
			I2CCNT--;
		IFS1bits.SI2C1IF=0;
		while(!I2C1STATbits.P && !IFS1bits.T4IF) {
			ClrWdt();
			if(IFS1bits.SI2C1IF)
				goto receive_i2c_loop;
//			if(I2C1STATbits.ACKSTAT)
//				goto fine;
			}
		}		// write
	else {			// if(I2C1STATbits.R_W ==1) // read operation

//    if(I2C1STATbits.D_A == 1) { // valid data
		if(!(I2CFLG & (1 << B_UA))) {		// questa è una lettura pura (no SubAddress)
			I2CFLG |= (1 << B_UA);
			I2CSUBA=0;		//...cosa che qua non ha molto senso! ma in altri device magari
			I2CFLG |= (1 << B_SA);
			I2CCNT=0;
			T4CON=0x8030;			// 1:256, 4uS
			TMR4=0;
			PR4=1200;			//4mSec timeout
			IFS1bits.T4IF=0;
			I2CTimeout=1;
// qua non dovrebbe servire			while(I2C1STATbits.TBF);
//			I2C1TRN = 0xff; // send dummy
	    I2C1CONbits.SCLREL = 1; // release clock 
		  goto do_receive_i2c_loop;
			}
		else {													// è stato prima scritto un SubAddress e ora leggiamo da là
			if(!(I2CFLG & (1 << B_RD))) {		// quindi se ci son state Write prima, I2CCNT è != 0, per cui ora lo azzero
				I2CFLG |= (1 << B_RD);
				I2CCNT=0;
				}
			else
				I2CCNT++;
//			if(I2CCNT>0) {
//				I2CSUBA=I2CREG;
//				I2CFLG |= (1 << B_SA);
//		    I2C1CONbits.SCLREL = 1; // release clock 
//				while(I2C1STATbits.TBF);
//				I2C1TRN = 'x'; // send dummy

//				I2CSUBA=0;


//LCDPrintHex(I2CFLG);
//LCDPrintHex(I2CSUBA);
//LCDPrintHex(I2CCNT);
			switch(I2CSUBA) {
				case 0:

/**********************************************************************
;* Device ID text: read starting at sub-address 0
;**********************************************************************/
					if(I2CFLG & (1 << B_RID)) {		// (pd)
						switch(I2CSUBA & 0xfe) {
							case DEVICE_ADDRESS:		// non va ancora... 28/1/21 forse dipende dal modo "sola lettura", v sopra, che però pare ok...
								n=0x4710 | VERNUMH;			// c'ho solo 2 BYTE e faccio così!! 1 è picbell
								if(I2CCNT & 1) 
									I2CREG = LOBYTE(n);
								else
									I2CREG = HIBYTE(n);		// standard i2c...!
			// https://www.microchip.com/forums/m1013076.aspx ; NXP user-guide/UM10204
			// mandare 3 BYTE:
			//  12bit manufacturer ID
			//  9 bit part ID
			//  3 bit revision ID
								break;
// arriva in Write, giustamente! (v. di là)			case 6:		// https://www.i2c-bus.org/addressing/general-call-address/ https://www.nxp.com/docs/en/user-guide/UM10204.pdf
//								asm("reset");
//								break;
							default:
								I2CREG = -1;			//boh sì
								break;
							}
						}
					else {
			  		switch((I2CCNT) & 7) {
					  	case 0:
					    case 1:
					    case 2:
			  	    case 3:
 					    case 4:
					    case 5:
					    case 6:
								I2CREG="I2C_PS2 "[(I2CCNT) & 7];
								break;
					    case 7:
								I2CREG = (VERNUMH << 4) | VERNUML;		// occhio...
								break;
							}
						}
					break;
		
				case 1:
sendMsg_getCfg:						// v. anche QUERY_STATUS per 232!
					break;
		
				case 2:
#ifdef USA_TASTIERA
#ifdef USA_232
// inviare su seriale? ci pensa MandaTastoPremuto!
#else
sendMsg_getKB:
#if MAX_TASTI_CONTEMPORANEI>2
					I2CREG=kbKeys[I2CCNT & 3];		// 0..3 (buff tastiera)
#else
					I2CREG=kbKeys[I2CCNT & 1];		// 0..1
#endif
					// bisognerebbe azzerare kbKeys dopo la lettura? o no? in teoria non serve...
#endif
#endif

#ifdef USA_ANALOG
#ifdef USA_232
				// inviare su seriale? ci pensa MandaTastoPremuto!
#else
sendMsg_getMouse:
					I2CREG=mouKeys[I2CCNT & 3];		//0..3 (buff mouse)
#endif
#endif

#ifdef USA_RFID
sendMsg_getRFID:
#if MAX_TASTI_CONTEMPORANEI>2
					I2CREG=kbKeys[I2CCNT & 3];
#else
					I2CREG=kbKeys[I2CCNT & 1];		// 0..1 bastano direi
#endif
#endif
//        	if(I2CCNT>=3)
					break;
		
				case 3:
sendMsg_getCfg2:					// v. anche QUERY_STATUS per 232!
					switch(I2CCNT & 3) {
						case 0:
#ifdef USA_TASTIERA	
							I2CREG= 1;
#else
#ifdef USA_ANALOG
							I2CREG= 2;
#else
							I2CREG= 0;
#endif
#endif
							break;
						case 1:
							I2CREG= 0;								// TEXT
							break;
						case 2:
							I2CREG= 0;
							break;
						case 3:
							I2CREG= 0;						// tipo=ROM
							break;
						}
					// Limit to 4 locations (GET_EQUIPMENT)
		
					break;
		
				case 4:							// 
					break;
		
		 		case 12:					// 
sendMsg_readFlash:
					break;
			
				case 5:
				case 6:
				case 7:
					break;
				case 8:			// leggo AN?, per sfizio 
					break;
				case 9:

				case 10:
				case 11:
				case 13:
				case 14:
				default:
					I2CREG =-1;
					break;
				}	
//LCDPrintHex(I2CREG);
//			if(I2C1STATbits.ACKSTAT)	// potrebbe aver senso controllare se l'ultimo byte ha avuto ACK dal master... c'è un doppione sulle letture singole 9/29/21
			I2C1TRN = I2CREG; // send data
      I2C1CONbits.SCLREL = 1; // release clock 
    	goto do_receive_i2c_loop;
			
			}			// 

    I2C1CONbits.SCLREL = 1; // release clock 
	  goto do_receive_i2c_loop;
		}				// read

	if(I2C1STATbits.P) {
		if(!(I2CFLG & (1 << B_RD))) {		// distinguo tra i 2 casi... :)
   		PacketReceived = TRUE;
			I2CTimeout=0;
			I2C1STATbits.IWCOL=0;
			I2C1STATbits.I2COV=0;
			}
		else 
			goto pulisci;
		}
	else {
pulisci:
    I2CR0[0]=I2CR0[1]=0;			//ClearRXBuffer();
		I2CFLG=0;
		I2CSUBA=0;
		I2CCNT=0;
		I2C1STATbits.IWCOL=0;
		I2C1STATbits.I2COV=0;
    I2C1CONbits.SCLREL=1; 
		I2CTimeout=0;
		}
	T4CON=0;
    

fine:
	IFS1bits.SI2C1IF=0;

	}


void __attribute__ (( interrupt/*, shadow*/,  no_auto_psv )) _T1Interrupt(void) {     //100mS
	BYTE i;
	static const BYTE dayOfMonth[12]={31,28,31,30,31,30,31,31,30,31,30,31};

	static BYTE kbScanY;
	static BYTE kbtemp;



//					m_Led0Bit ^= 1;		//test timing: 100mSec 29/1/21


	if(!(FLAGS & (1 << NOSCANK))) {
//		ClrWdt();
#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)
      if(m_Puls1Bit && m_Puls2Bit) {
        kbKeys[0]=kbKeys[1]=0;
        }
      else if(!m_Puls1Bit && (!ANSELAbits.ANSA1 && !m_Puls2Bit)) {    //v. sopra leggianalog
        kbKeys[0]=1;
        kbKeys[1]=2;
        }
      else {
        if(!m_Puls1Bit) 
          kbKeys[0]=1;
        if(!ANSELAbits.ANSA1 && !m_Puls2Bit) 
          kbKeys[0]=2;
        }	
#else
      if(m_Puls1Bit && m_Puls2Bit) {
        kbKeys[0]=kbKeys[1]=0;
        }
      else if(!m_Puls1Bit && !m_Puls2Bit) {
        kbKeys[0]=1;
        kbKeys[1]=2;
        }
      else {
        if(!m_Puls1Bit) 
          kbKeys[0]=1;
        if(!m_Puls2Bit) 
          kbKeys[0]=2;
        }	

      FLAGK |= (1 << TRIGK);
		}			// NOSCANK


#endif
	

		if(I2CTimeout) {
			I2CTimeout++;
			if(I2CTimeout>5) {			//1/2 secondo...
				I2CTimeout=0;
				I2C1STATbits.IWCOL=0;
				I2C1STATbits.I2COV=0;
		    I2C1CONbits.SCLREL=1; 
//inutile	T4CON=0;
	    	PacketReceived = FALSE;
				}
			}


		Clock_10++;
		if(Clock_10 >= 10) {
			Clock_10=0;

		}
  
//	T1_Clear_Intr_Status_Bit; 	
	IFS0bits.T1IF = 0; 			//Clear the Timer1 interrupt status flag 
	}


void __attribute__ ((interrupt,/*shadow,*/no_auto_psv)) _U2RXInterrupt(void) {
	BYTE ByteRec;

//		ClrWdt();
#ifdef USA_232
 	
  if(U2STAbits.FERR)		// ignorare carattere?
	  /*CommStatus.COMM_FRERR=1*/;
  if(U2STAbits.PERR)
	  /*CommStatus.COMM_PERR=1*/;
	if(U2STAbits.OERR) {			// non mi interessano i caratteri ev. in attesa...
		U2STAbits.OERR=0;

//		CommStatus.COMM_OVRERR=1;
		}

	while(!DataRdyUART1());
  ByteRec = ReadUART1();

//		m_Led2Bit ^= 1;			// ***test
//		CommStatus.FRAME_REC=1;
//goto fine;

	Buffer232[Buf232Ptr++]=ByteRec;
	Buf232Ptr &= BUF_232_SIZE-1;



//putcUART1(ByteRec );			// test
#endif

	IFS1bits.U2RXIF = 0; 			//Clear the Timer1 interrupt status flag 
//  U2RX_Clear_Intr_Status_Bit;
	}

// ---------------------------------------------------------------------------------------

#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)    // se no non c'è spazio!
#include <stdio.h>
void _ISR __attribute__((__no_auto_psv__)) _AddressError(void) {
	Nop();
	Nop();
  char buf[32];
  LCDCls();
  sprintf(buf,"address error");
  LCDWrite(buf);
  display();
  while(1)
    ClrWdt();
	}

void _ISR __attribute__((__no_auto_psv__)) _StackError(void) {
	Nop();
	Nop();
  char buf[32];
  LCDCls();
  sprintf(buf,"stack error");
  LCDWrite(buf);
  display();
  while(1)
    ClrWdt();
	}

void _ISR __attribute__((__no_auto_psv__)) _MathError(void) {
	Nop();
	Nop();
  char buf[32];
  LCDCls();
  sprintf(buf,"math error");
  LCDWrite(buf);
  display();
  while(1)
    ClrWdt();
	}


void _ISR __attribute__((__no_auto_psv__)) _OscillatorFail(void) {
	Nop();
	Nop();
  char buf[32];
  LCDCls();
  sprintf(buf,"oscillator error");
  LCDWrite(buf);
  display();
  while(1)
    ClrWdt();
	}

void _ISR __attribute__((__no_auto_psv__)) _DefaultInterrupt(void) {
	Nop();
	Nop();
  char buf[32];
  LCDCls();
  sprintf(buf,"error, INTCON=%04X",INTCON1);
  LCDWrite(buf);
  display();
  while(1)
    ClrWdt();
	}

#else
void _ISR __attribute__((__no_auto_psv__)) _AddressError(void) {
	Nop();
	Nop();
  }
void _ISR __attribute__((__no_auto_psv__)) _StackError(void) {
	Nop();
	Nop();
  }
#endif

