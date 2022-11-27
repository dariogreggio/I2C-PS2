// PS/2 routines comuni - 04/5/03 - 13/7/04 - 19/12/07 - 19/1/08
// versione C 1/3/2005
// (C) ADPM Synthesis sas 2001-2008 - G.Dar, Cyberdyne 2009-2012
//		(portions Copyright 2001, Adam Chapweske chap0179@tc.umn.edu)
// 2021 GD/K per PC_PIC motherboard

// PARAMETRIZZARE i BIT e le PORTE usate!!


#ifdef __XC16
#include <xc.h>
#else
#include <p24fxxxx.h>
#endif
#include "io_cfg.h"
#include "ps2.h"
#include "i2c_ps2.h"

#include <libpic30.h>


BYTE PS2ID[2];
BYTE PS2Errors;			// b0..1=errori slave1, b2..3=errori slave2 -
BYTE SlavePresent;  // b0=tast 1, b1=mouse 1, b2=tast 2, b3=mouse1; b6=tast usb, b7=mouse USB

extern BYTE keyboardTypematic1,keyboardTypematic2;

// ----------------------------------------------------------------------
signed char InitPS2(BYTE q) {
  BYTE i;

  LATB  &= ~0b0000000000001111;     // 
  TRISB &= ~0b0000000000001111;     // tengo in reset
  ODCB  = 	0b0000000000001111;
  
  __delay_ms(100);

#warning in teoria si potrebbero mettere i pullup cmq, che non disturbano e specie se il cs è incompleto aiutano..
#if 0  
#if defined(__PIC24FJ256GB606__) || defined(__PIC24FJ512GB606__)
  CNPUBbits.CNPUB0=CNPUBbits.CNPUB1=CNPUBbits.CNPUB2=CNPUBbits.CNPUB3=1;
#else
  CNPU2bits.CN69PUE=CNPU2bits.CN71PUE=CNPU2bits.CN17PUE=CNPU2bits.CN18PUE=1;
#endif
#endif

  TRISB |= 0b0000000000001111;
  ClrWdt();
  __delay_ms(300);    // se no la tastiera non è pronta, (considerando tutto)

  SlavePresent &= ~0b00001111;
  
  PS2_slaveout(0xFF);   // la risposta qua arriva subito! v. main loop, dev'essere pronto!??
  i=PS2_slavein();    // non è chiaro se c'è risposta al comando reset, ma pare andare sia tast che mouse, 25/6/22
  
  i=PS2_slavein();    // 0xAA...
  if(slaveHandler(i) != 0xff) {

    PS2_slaveout(0xF2);   // Read ID?
    PS2_slavein();
    PS2ID[0]=PS2_slavein();
    
    PS2_slaveout(0xF6);   // SetDefault
    i=PS2_slavein();
    slaveHandler(i);

    if(PS2ID[0] == DEVICE_KEYBOARD) {    // keyboard
      PS2_slaveout(0xED);   // SetLed
      i=PS2_slavein();
      PS2_slaveout(0b00000111);   // come pc
      i=PS2_slavein();
      slaveHandler(i);
      __delay_ms(50);
      PS2_slaveout(0xED);   // SetLed
      i=PS2_slavein();
      PS2_slaveout(0b00000000);
      i=PS2_slavein();
      slaveHandler(i);
      PS2_slaveout(0xF3);   // Typematic
      i=PS2_slavein();
      PS2_slaveout(((keyboardTypematic1 & 0b11) << 5) | (keyboardTypematic2 & 0b00011111));   // b6:5 delay before 1st; b4:0 30Hz-2Hz repeat
      i=PS2_slavein();
      slaveHandler(i);
      }

    PS2_slaveout(0xF4);   // Enable
    i=PS2_slavein();
    slaveHandler(i);
    }
  
  PS2_slaveout2(0xFF);   // 
  i=PS2_slavein2();    // 
  
  i=PS2_slavein2();    // 0xAA...
  if(slaveHandler2(i) != 0xff) {

    PS2_slavein2();
    PS2ID[1]=PS2_slavein2();

    PS2_slaveout2(0xF6);   // SetDefault
    i=PS2_slavein2();
    slaveHandler2(i);
      
    if(PS2ID[1] == DEVICE_KEYBOARD) {    // keyboard
      PS2_slaveout2(0xED);   // SetLed
      i=PS2_slavein2();
      PS2_slaveout2(0b00000111);   // come pc
      i=PS2_slavein2();
      slaveHandler2(i);
      __delay_ms(50);
      PS2_slaveout2(0xED);   // SetLed
      i=PS2_slavein2();
      PS2_slaveout2(0b00000000);
      i=PS2_slavein2();
      slaveHandler2(i);
      PS2_slaveout2(0xF3);   // Typematic
      i=PS2_slavein2();
      PS2_slaveout2(((keyboardTypematic1 & 0b11) << 5) | (keyboardTypematic2 & 0b00011111));   // b6:5 delay before 1st; b4:0 30Hz-2Hz repeat
      i=PS2_slavein2();
      slaveHandler2(i);
      }

    PS2_slaveout2(0xF4);   // Enable
    i=PS2_slavein2();
    slaveHandler2(i);
    }
  }

//---------------------------------------------------------------------------------------------
BYTE PS2Error(void) {
  
  return PS2Errors;
  }


BYTE PS2_slaveinBit(void) {
	BYTE i,slaveInTimer;

	slaveInTimer=250;					// ca. 750uS max (x2=6KHz min. acc.)

	do {
		ClrWdt();
		if(!m_RXClkBit)				// aspetto che CLK scenda...
			goto  PS2_slaveinBit2;

		__delay_us(3);
		} while(--slaveInTimer);
	return 0xff;

PS2_slaveinBit2:
	__delay_us(2);
	if(m_RXClkBit)					// CLK DEVE essere ancora basso!!
		return 0xff;

#ifdef DEBUG3
	LATCbits.LATC3 ^= 1;		// ***** debug
#endif

	i= m_RXDataBit ? 0x80 : 0x00;

	slaveInTimer=250;						// v.sopra

	do {
		ClrWdt();
		if(m_RXClkBit)				// aspetto che CLK risalga...
			goto  PS2_slaveinBit4;

		__delay_us(3);
		} while(--slaveInTimer);
	return 0xff;

PS2_slaveinBit4:
	return i;
	}


BYTE PS2_slavein(void) {
	BYTE i,PARITY,TEMP0,COUNTER;

	i=PS2_slaveinBit();
	if(i == 0xff) {									// se errore...
//		clrf SlavePresent
//			LATCbits.LATC0 = 0;		// ***** debug
		PS2Errors |= 1;
		return 0xff;
		}

		
	COUNTER=8;		//Setup a counter
	PARITY=0;
	TEMP0=0;		// forse inutile...
	PS2Errors &= ~1;

	do {
		i=PS2_slaveinBit();
		if(i == 0xff)	{								// se errore...
			SlavePresent &= ~(PS2ID[0] == DEVICE_KEYBOARD ? 1 : 2);
//			LATCbits.LATC0 = 0;		// ***** debug
			PS2Errors |= 1;
			return 0xff;
			}

		TEMP0 >>= 1;
		TEMP0 |= i;
		PARITY ^= i;
		} while(--COUNTER);


	i=PS2_slaveinBit();			//Parity Bit
	if(i == 0xff) {									// se errore...
//	clrf SlavePresent
//			LATCbits.LATC0 = 0;		// ***** debug
		PS2Errors |= 1;
		return 0xff;
		}

	PARITY ^= i;

	i=PS2_slaveinBit();			//Stop Bit
	if(i == 0xff)	{								// se errore...
//		clrf SlavePresent
//			LATCbits.LATC0 = 0;		// ***** debug
		PS2Errors |= 1;
		return 0xff;
		}

	if(i == 0x80) {														// dev'essere "1"!!
		if(PARITY & 0x80) {

#ifdef DEBUG1
	PutByte232(TEMP0)	;			//***** debug
#endif

#ifdef DEBUG_232
	//	movfw	TEMP0					FARE!!! v. interlin.asm
	//	call PutByte232_HW
#endif

			return TEMP0;
			}
		}


//		call	PS2InError
// NO!! COSA DEVO FARE??
	PS2Errors |= 1;


#ifdef DEBUG2
		PutByte232(TEMP0);		// ***** debug
#endif

#ifdef DEBUG_232
//	movfw	TEMP0					FARE!!! v. interlin.asm
//	call PutByte232_HW
#endif

	return 0;

	}


BYTE PS2_slaveout(BYTE n) {
	BYTE i,slaveInTimer,PARITY,COUNTER,TEMP0;


	TXClkTris=0;				// blocco eventuali trasmissioni...
	__delay_us(100);
	TXDataTris=0;				// inizio la trasmissione...
	__delay_us(2);
	TXClkTris=1;
//		Delay 2


// il loop di ritardo che segue e' importante perche' (come da documentazione) se si cerca di scrivere alla slave mentre era inibita, questa puo' metterci fino a 10mS per iniziare a generare il clock...
	slaveInTimer=255;

	do {
		ClrWdt();
		if(!m_RXClkBit)				// aspetto che CLK scenda...
			goto PS2_slaveoutStart2;

		__delay_us(25);
		} while(--slaveInTimer);
	TXDataTris=1;				// inizio la trasmissione...
	TXClkTris=1;

	return 0xff;

PS2_slaveoutStart2:

//		clrw
//		call  PS2_slaveoutBit
		
	PARITY=0x01;
	COUNTER=0x08;
	TEMP0 =n;

	do {
		PARITY ^= TEMP0;
		i=PS2_slaveoutBit(TEMP0);
		if(i == 0xff) { 									// se errore...
			TXDataTris=1;				// fine trasmissione...
//		clrf SlavePresent
//			LATCbits.LATC0 = 0;		// ***** debug
			return 0xff;
			}

		TEMP0 >>= 1;
		} while(--COUNTER);

	PS2_slaveoutBit(PARITY);
	PS2_slaveoutBit(1);

	TXDataTris=1;				// fine trasmissione (forse inutile xche' lo fa il stop-bit)

// aspetto ACK dalla tastiera/mouse!!
	PS2_slaveinBit();
// controllare???
	return	0x00;
	}

BYTE PS2_slaveoutBit(BYTE n) {
	BYTE slaveInTimer;
		
	slaveInTimer=255;

	do {
		ClrWdt();
		if(!m_RXClkBit)				// aspetto che CLK scenda... 
			goto  PS2_slaveoutBit2;

		__delay_us(1);
		} while(--slaveInTimer);

	return 0xff;


PS2_slaveoutBit2:
	TXDataTris = n & 1;

	slaveInTimer=255;

	do {
		ClrWdt();
		if(m_RXClkBit)				// aspetto che CLK salga...
			goto PS2_slaveoutBit4;

		__delay_us(1);
		} while(--slaveInTimer);

	return 0xff;

PS2_slaveoutBit4:
	return 0;
	}


BYTE slaveHandler(BYTE cmd) { // 0 se tasto, 1 se comando ok, 0xff se errore (fare...)
  
	switch(cmd) {
		case 0xAA:

//0xAA - Power on self test passed - BAT complete
slaveKb_AA:
			PS2_slaveout(0xF2);   // Read ID?
			PS2_slavein();
			PS2ID[0]=PS2_slavein();

			SlavePresent |= PS2ID[0] == DEVICE_KEYBOARD ? 1 : 2;

//			LATCbits.LATC0 = 1;		// ***** debug

//mandare lo stato dei led??
//			slave_KB_ED();

			return 1;
			break;

		case 0xFA:
//0xFA - Acknowledge
slaveKb_FA:
			SlavePresent |= PS2ID[0] == DEVICE_KEYBOARD ? 1 : 2;
			return 1;
			break;

		case 0xEE:
//0xEE - Set Echo
slaveKb_EE:
      //PS2_slaveout(0xee);fare?
			return 1;
			break;

		case 0xFE:
//0xFE - Resend
slaveKb_FE:
//		call  ACK_PS2_slaveout			;Unbuffered output 0xFA (acknowledge)
//		goto	PS2_slaveOut					; ammesso che serva, forse bisogna rispedire l'intero ultimo blocchetto (che potrebbe in effetti essere un solo byte, sempre, per lo slave)
			return 1;
			break;

		case 0xFC:
		case 0xFD:
//0xFC - Power on self test Failed !
slaveKb_FC:

// che fare??
			SlavePresent &= ~(PS2ID[0] == DEVICE_KEYBOARD ? 1 : 2);

			return 0xff;
			break;

		case 0x0:
		case 0xff:
//0x00 o 0xFF - Error or buffer overflow
slaveKb_00_FF:
			return 0;
			break;

		default:


			SlavePresent |= (PS2ID[0] == DEVICE_KEYBOARD ? 1 : 2);			// se sono qua, la slave c'e'!
//			LATCbits.LATC0 = 1;		// ***** debug

			return 0;
//		goto slaveKbError		boh?? no, mai...
			break;



//Invalid Command
slaveKbError:
//		movlw	0xFC
//		call	PacketOut1Byte
			return 0xFF;

		}			// switch
	}			// slaveKBHandler


#if 0
void slave_KB_ED(void) {
	BYTE i,TEMP0;

	i=PS2_slaveout(0xED);
	if(i == 0xff)							// controllo ev. errore (piu' che altro x debug...)
		goto Kb_ED_2;

	TEMP0=PS2_slavein();				// aspettare ACK?
//		call PS2_slavein

		// gli inoltra lo stato dei led
	i=PS2_slaveout(LedStatus);
	if(i == 0xff)							// controllo ev. errore (piu' che altro x debug...)
		goto Kb_ED_2;

	TEMP0=PS2_slavein();				// aspettare ACK?

Kb_ED_2:
				;
	}
#endif


BYTE PS2_slaveinBit2(void) {
	BYTE i,slaveInTimer;

	slaveInTimer=250;					// ca. 750uS max (x2=6KHz min. acc.)

	do {
		ClrWdt();
		if(!m_RXClkBit2)				// aspetto che CLK scenda...
			goto  PS2_slaveinBit2;

		__delay_us(3);
		} while(--slaveInTimer);
	return 0xff;

PS2_slaveinBit2:
	__delay_us(2);
	if(m_RXClkBit2)					// CLK DEVE essere ancora basso!!
		return 0xff;

	i= m_RXDataBit2 ? 0x80 : 0x00;

	slaveInTimer=250;						// v. sopra

	do {
		ClrWdt();
		if(m_RXClkBit2)				// aspetto che CLK risalga...
			goto  PS2_slaveinBit4;

		__delay_us(3);
		} while(--slaveInTimer);
	return 0xff;

PS2_slaveinBit4:
	return i;
	}


BYTE PS2_slavein2(void) {
	BYTE i,PARITY,TEMP0,COUNTER;

	i=PS2_slaveinBit2();
	if(i == 0xff) {									// se errore...
//			SlavePresent &= ~1;
//			LATCbits.LATC0 = 0;		// ***** debug
		PS2Errors |= 4;
		return 0xff;
		}

		
	COUNTER=0x08;		//Setup a counter
	PARITY=0;
	TEMP0=0;		// forse inutile...
	PS2Errors &= ~4;

	do {
		i=PS2_slaveinBit2();
		if(i == 0xff)	{								// se errore...
			SlavePresent &= ~(PS2ID[1] == DEVICE_KEYBOARD ? 4 : 8);
//			LATCbits.LATC0 = 0;		// ***** debug
			PS2Errors |= 4;
			return 0xff;
			}

		TEMP0 >>= 1;
		TEMP0 |= i;
		PARITY ^= i;
		} while(--COUNTER);


	i=PS2_slaveinBit2();			//Parity Bit
	if(i == 0xff) {									// se errore...
		PS2Errors |= 4;
		return 0xff;
		}

	PARITY ^= i;

	i=PS2_slaveinBit2();			//Stop Bit
	if(i == 0xff)	{								// se errore...
		PS2Errors |= 4;
		return 0xff;
		}

	if(i == 0x80) {														// dev'essere "1"!!
		if(PARITY & 0x80) {

			return TEMP0;
			}
		}


//		call	PS2InError
// NO!! COSA DEVO FARE??
	PS2Errors |= 4;

	return 0;
	}


BYTE PS2_slaveout2(BYTE n) {
	BYTE i,slaveInTimer,PARITY,COUNTER,TEMP0;


	TXClkTris2=0;				// blocco eventuali trasmissioni...
	__delay_us(100);
	TXDataTris2=0;				// inizio la trasmissione...
	__delay_us(2);
	TXClkTris2=1;
//		Delay 2


// il loop di ritardo che segue e' importante perche' (come da documentazione) se si cerca di scrivere alla slave mentre era inibita, questa puo' metterci fino a 10mS per iniziare a generare il clock...
	slaveInTimer=255;

	do {
		ClrWdt();
		if(!m_RXClkBit2)				// aspetto che CLK scenda...
			goto PS2_slaveoutStart2;

		__delay_us(25);
		} while(--slaveInTimer);
	TXDataTris2=1;				// inizio la trasmissione...
	TXClkTris2=1;

	return 0xff;

PS2_slaveoutStart2:
	PARITY=0x01;
	COUNTER=0x08;
	TEMP0 =n;

	do {
		PARITY ^= TEMP0;
		i=PS2_slaveoutBit2(TEMP0);
		if(i == 0xff) { 									// se errore...
			TXDataTris2=1;				// fine trasmissione...
			return 0xff;
			}

		TEMP0 >>= 1;
		} while(--COUNTER);

	PS2_slaveoutBit2(PARITY);
	PS2_slaveoutBit2(0x01);

	TXDataTris2=1;				// fine trasmissione (forse inutile xche' lo fa il stop-bit)

// aspetto ACK dalla tastiera/mouse!!
	PS2_slaveinBit2();
// controllare???
	return 0x00;
	}

BYTE PS2_slaveoutBit2(BYTE n) {
	BYTE slaveInTimer;
		
	slaveInTimer=255;

	do {
		ClrWdt();
		if(!m_RXClkBit2)				// aspetto che CLK scenda... NON DOVREBBE ESSERE il "2" ?? ! (2012)
			goto  PS2_slaveoutBit2;

		__delay_us(1);
		} while(--slaveInTimer);

	return 0xff;

PS2_slaveoutBit2:
	TXDataTris2 = n & 1;

	slaveInTimer=255;

	do {
		ClrWdt();
		if(m_RXClkBit2)				// aspetto che CLK salga...
			goto  PS2_slaveoutBit4;

		__delay_us(1);
		} while(--slaveInTimer);

	return 0xff;

PS2_slaveoutBit4:
	return 0;
	}


BYTE slaveHandler2(BYTE cmd) { // 0 se tasto, 1 se comando ok, 0xff se errore (fare...)

	switch(cmd) {
		case 0xAA:
			PS2_slaveout2(0xF2);   // Read ID?
			PS2_slavein2();
			PS2ID[1]=PS2_slavein2();
			SlavePresent |= PS2ID[1] == DEVICE_KEYBOARD ? 4 : 8;
			return 1;
			break;

		case 0xFA:
			SlavePresent |= PS2ID[1] == DEVICE_KEYBOARD ? 4 : 8;
			return 1;
			break;

		case 0xEE:
      //PS2_slaveout(0xee);fare?
			return 1;
			break;

		case 0xFE:
			return 1;
			break;

		case 0xFC:
		case 0xFD:
			SlavePresent &= ~(PS2ID[1] == DEVICE_KEYBOARD ? 4 : 8);
			return 0xff;
			break;

		case 0x0:
		case 0xff:
			return 1;
			break;

		default:
      // se sono qua, CMQ direi lo slave c'è!
			SlavePresent |= PS2ID[1] == DEVICE_KEYBOARD ? 4 : 8;
			return 0;
			break;

//Invalid Command
slaveKbError:
			return 0xFF;

		}			// switch
	}			// slaveKBHandler2

