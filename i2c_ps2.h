/*********************************************************************
 *
 *                Microchip C30 Firmware - PICBell24
 *
 *********************************************************************
 * FileName:        picbell.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC24
 * Compiler:        C30/XC16
 * Company:         Microchip Technology, Inc. (quei froci)
 *
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/

#ifndef I2C_PS2_H
#define I2C_PS2_H

#include "generictypedefs.h"


//#define USA_232	1							// I/F 232 (altrimenti I2C)
//#define I2C_SMBUS 1							// per  modalita' SmBus-compatibile (v. RecvMsg e SendMsg)
//#define DEBUG_PULS  1					// reset ai default di alcuni valori se premuto il puls2 all'accensione
#define USA_KB_BUZZER 1					// bip di conferma sui tasti


#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
#define CLOCK_FREQ 140000000ULL
#else
#define CLOCK_FREQ 32000000ULL
#endif
#define GetSystemClock()		(CLOCK_FREQ)      // Hz
#define GetInstructionClock()	(GetSystemClock()/2)
#define GetPeripheralClock()	GetInstructionClock()
#define FCY (GetSystemClock()/2)		// per LibPic30.h e delay

#if defined(__PIC24EP512GP202__) || defined(__PIC24EP256GP202__) 
#define TMR1BASE  (27343+1)		// IRQ ogni 100mS circa (prescaler=256: 14nS*256=3657)
#else
#define TMR1BASE  (6250+1 /*3% veloce 28/1/21*/)		// IRQ ogni 100mS circa (prescaler=256: 64nS*256=16000)
#endif

extern WORD FLAGS;        //GENERAL PURPOSE FLAG, METTERE IN CONFIGPARMS??
extern BYTE FLAGK;

#define TIME_GRANULARITY 104   // il tempo per 1 bit (104.2 @ 9600baud)

#define MAX_TASTI_CONTEMPORANEI 2			// 2 o 4 o.. !
extern volatile BYTE kbKeys[MAX_TASTI_CONTEMPORANEI];
extern BYTE kbKeysOld[MAX_TASTI_CONTEMPORANEI];


#define DEVICE_ADDRESS 0x48     //I2C device address (device_address+1 = read)
#define DEVICE_ID_ADDRESS 0xf8
#define DEVICE_GENERAL_CALL_ADDRESS 0x00

//--i2c flags-------------------------------------------------
#define B_ST  0		//Flag: 1=in start
#define B_RD  1		//Flag: 1=read
#define B_SA	2		//Flag: 1=reading subabbress
#define B_UA	3		//Flag: 1=reading unit address
#define B_RID	4		//Flag: 1=reading DeviceID
#define B_ACK	7		//Flag: 1=reading ACK
//------------------------------------------------------------

#define SERNUM 1000
#define VERNUMH 0
#define VERNUML 1


//void di(void);
#define di() { asm volatile ("disi #0x3FFF"); }	// Only disabled for 32768 cycles
//void ei(void);
#define ei() { asm volatile ("disi #0"); }



struct __attribute__((__packed__)) SAVED_PARAMETERS {
	BYTE keyboardTypematic1,keyboardTypematic2;		// usare questi??
	BYTE FLAGS;
	};

extern struct SAVED_PARAMETERS configParms;
#define PARMS_FIELD(field) configParms.field



union COMM_STATUS {
	unsigned int w;
	struct {
// errori/flag in CommStatus Seriale, (LSB prima - right-aligned)
		unsigned int FRAME_2SEND:1;         // c'e' da mandare un frame
		unsigned int WAIT_4ACK  :1;
		unsigned int FRAME_REC  :1;         // c'e' un frame ricevuto
		unsigned int COMM_OVRERR:1;					// overrun seriale 
		unsigned int COMM_OVL   :1;         // errore di overflow buffer in ricezione 
		unsigned int COMM_TOUT  :1;         // errore di timeout in ricezione frame
		unsigned int COMM_FRERR :1;         // errore di framing (START/STOP) in ricezione 
		unsigned int COMM_PERR  :1;         // errore di parita' in ricezione  (0x80)
		unsigned int BUSY  :1;							// occupato
		unsigned int LINE_BUSY  :1;         // linea intasata, impossibile inviare
		unsigned int COMM_CRCERR  :1;         // errore di checksum in ricezione 
		};
	};


// Flag bits (FLAGS)
#define NOSCANK	0
#define NOKCLICK 1
#define BUZZON	2

//FLAGK
#define TRIGK		0



enum {
	BEEP_STD_FREQ	=2187,  	// 2187=4KHz ca. @ 140MHz; i buzzer danno più dB verso i 4KHz...
	BEEP_LOW_FREQ	=9000,
	BEEP_HIGH_FREQ	=1500
	};


/** P U B L I C  P R O T O T Y P E S *****************************************/

#define Delay_S() __delay_ms(1000)

#define EEscrivi0(p) EEscrivi(p,0)               // scrive 0...
#define EEscrivi(a,b) { *(((BYTE *)configParms)+a)=b; EEscrivi_(a,b); }  // scrive W in RAM e EPROM...
void EEscrivi_(BYTE,BYTE);
BYTE EEleggi(BYTE);
#define EEcopiaARAM(o) { *(((BYTE *)configParms)+o)=EEleggi(o); }
#define EEcopiaAEEPROM(n) { EEscrivi_(offsetof(struct SAVED_PARAMETERS,n),PARMS_FIELD(n)); }




#define kbKeyPressed()  ( kbKeys[0] )
	// ritorna NOZERO se c'e' un tasto premuto
BYTE GetKBchar(BYTE);
void syncPS2Leds(void);

BYTE KBInit(void);

void StdBeep(void);
void Beep(BYTE);


#endif // I2C_PS2_H
