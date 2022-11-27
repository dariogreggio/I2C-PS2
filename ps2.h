#ifndef _PS2_INCLUDED
#define _PS2_INCLUDED

// PS/2 routines comuni - 04/5/03 - 13/7/04 - 22/2/2012
// versione C 1/3/2005
// versione PIC24 (PC_PIC) 2021-2022
// Dario's automation (Cyberdyne (ADPM Synthesis sas)) 2001-2022 - G.Dar
//		(portions Copyright 2001, Adam Chapweske chap0179@tc.umn.edu)

// PARAMETRIZZARE i BIT e le PORTE usate!!

enum {
	DEVICE_MOUSE	= 0,		// 0 per standard PS/2 mouse, 3=Intellimouse, 4=Intellimouse con 4-5 pulsanti
	DEVICE_INTELLIMOUSE	= 3,
	DEVICE_INTELLIMOUSE4	= 4,
    
	DEVICE_KEYBOARD = 0xAB,     // non apparterrebbe qua ma ok :)
    
	DEVICE_UNKNOWN = 255				
	};
enum {
	RESET_MODE  = 0,
	STREAM_MODE, 
	REMOTE_MODE,
	WRAP_MODE
	};


extern unsigned char PS2Errors;

signed char InitPS2(unsigned char q);
unsigned char Input(void);
unsigned char InputSlave(void);
unsigned char PS2in(void);
unsigned char PS2inBit(void);
unsigned char PS2InError(void);

void Delay_uS(unsigned char);

unsigned char PS2out(unsigned char );
void ACK_PS2out(void);
void PS2outBit(unsigned char );
unsigned char PS2outEnd(void);
unsigned char PS2_slavein(void);
unsigned char PS2_slaveinBit(void);
unsigned char PS2_slavein2(void);
unsigned char PS2_slaveinBit2(void);

#define RESEND_PS2_slaveout() PS2_slaveout(0xFE)

#define ACK_PS2_slaveout() PS2_slaveout(0xFA)
		
unsigned char PS2_slaveout(unsigned char );
unsigned char PS2_slaveoutBit(unsigned char);

unsigned char PS2_slaveout2(unsigned char );
unsigned char PS2_slaveoutBit2(unsigned char);

#define Delay(n) Delay_uS(n)

unsigned char slaveHandler(unsigned char);
unsigned char slaveHandler2(unsigned char);
void slave_KB_ED(void);

#endif
