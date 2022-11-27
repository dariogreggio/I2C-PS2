/*********************************************************************
 *
 *                Microchip C30 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        io_cfg.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC24
 * Compiler:        C30/XC16
 * Company:         Microchip Technology, Inc.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       21/07/04    Original.
 * Cinzia Greggio       11/22		  	I2C_PS2
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0


/** I N P U T / O U T P U T *****************************************************/

// port A
#define BeepBit			7
#define m_BeepBit		LATBbits.LATB7
#define Puls1Bit			3
#define m_Puls1Bit	PORTAbits.RA3
#define Puls2Bit			0
#define m_Puls2Bit	PORTAbits.RA0		// condiviso con PGEC


// port B
#define I2CCIBit	8		// Clock I2C ev., I2C2 2021
#define m_I2CCIBit	PORTBbits.RB8
#define m_I2CCOBit	LATBbits.LATB8
#define I2CDBit		9		// Dati I2C
#define m_I2CDIBit	PORTBbits.RB9
#define m_I2CDOBit	LATBbits.LATB9
#define I2CCTRIS	TRISBbits.TRISB8
#define I2CDTRIS	TRISBbits.TRISB9
#define I2CCIVal	(1 << I2CCIBit)
#define I2CDVal		(1 << I2CDBit)

#define TXBit			0		// TX-232 su circuitino 2021, UART2, non usata cmq
#define RXBit			1		// RX-232
#define TXVal			(1 << TXBit)
#define RXVal			(1 << RXBit)

#define I2CslvCBit	8		// Clock I2C slave (qua usiamo I2C1 hw)
#define m_I2CslvCBit	LATBbits.LATB8
#define I2CslvDBit	9		// Dati I2C slave
#define m_I2CslvDBit	LATBbits.LATB9
#define I2CslvDTRIS 	TRISBbits.TRISB9
#define m_I2CslvDIBit	PORTBbits.RB9		// occhio a scrivere qua... causa RMW

// PWM1 buzzer
#define BuzzBit		7		// PWM1 OC3 RB1

#define Puls1Val  (1 << Puls1Bit)
#define Puls2Val	(1 << Puls2Bit)
#define BuzzVal		(1 << BuzzBit)
#define I2CslvCVal	(1 << I2CslvCBit)
#define I2CslvDVal	(1 << I2CslvDBit)


#define Led0Bit		2
#define m_Led0Bit		LATBbits.LATB2
#define Led1Bit		2
#define m_Led1Bit		LATAbits.LATA2			// 

#define Led0Val		(1 << Led0Bit)
#define Led1Val		(1 << Led1Bit)



// Hardware I/O pin mappings

#define m_RXDataBit PORTBbits.RB15
#define m_RXClkBit PORTBbits.RB14
#define TXDataTris TRISBbits.TRISB15         //
#define TXClkTris TRISBbits.TRISB14         // 
#define m_RXDataBit2 PORTBbits.RB13
#define m_RXClkBit2 PORTBbits.RB12
#define TXDataTris2 TRISBbits.TRISB13         // 
#define TXClkTris2 TRISBbits.TRISB12


#define UART1TX_TRIS		(TRISBbits.TRISB0)
#define UART1TX_IO			(LATBbits.LATB0)
#define UART1RX_TRIS		(TRISBbits.TRISB1)
#define UART1RX_IO			(PORTBbits.RB1)


#endif //IO_CFG_H
