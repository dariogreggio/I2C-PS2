/******************************************************************************

    USB Keyboard Host Application Demo

Description:
    This file contains the basic USB keyboard application. Purpose of the demo
    is to demonstrate the capability of HID host . Any Low speed/Full Speed
    USB keyboard can be connected to the PICtail USB adapter along with 
    Explorer 16 demo board. This file schedules the HID transfers, and interprets
    the report received from the keyboard. Key strokes are decoded to ascii 
    values and the same can be displayed either on hyperterminal or on the LCD
    display mounted on the Explorer 16 board. Since the purpose is to 
    demonstrate HID host all the keys have not been decoded. However demo gives
    a fair idea and user should be able to incorporate necessary changes for
    the required application. All the alphabets, numeric characters, special
    characters, ESC , Shift, CapsLK and space bar keys have been implemented.

Summary:
 This file contains the basic USB keyboard application.

Remarks:
    This demo requires Explorer 16 board and the USB PICtail plus connector.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the “Company”) for its PICmicro® Microcontroller is intended and
supplied to you, the Company’s customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.


********************************************************************************

 Change History:
  Rev    Description
  ----   -----------
  2.6a   fixed bug in LCDDisplayString() that could cause the string print to
         terminate early.

*******************************************************************************/
//DOM-IGNORE-END
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "GenericTypeDefs.h"
#include "io_cfg.h"
#include "i2c_ps2.h"
#include "keyboard.h"
#include "ps2.h"

//#define DEBUG_MODE

// *****************************************************************************
// *****************************************************************************
// Constants
// *****************************************************************************
// *****************************************************************************

// We are taking Timer 3  to schedule input report transfers

// da 0x91 le frecce, ecc.; pgdn/pgup; 9f=Pause
// da 0xa1 a 0xac i tasti Fn
// da 0xb1 a 0xb3 i lock; 0xb8=PrtSc, b9-bb=power/sleep/wake
//  su PS/2 CTRL BREAK arriva come B3??? e non ctrl-9f? su usb va..


#define MILLISECONDS_PER_TICK       10
#define TIMER_PERIOD                20000                // 10ms=20000, 1ms=2000



extern BYTE keyToSend[2];
extern BYTE SlavePresent;
extern WORD CodePage;
BYTE codePageArr=0;
extern BYTE keyboardTypematic1,keyboardTypematic2;
HID_LED_REPORT_BUFFER Appl_led_report_buffer;

// *****************************************************************************
// *****************************************************************************
// Internal Function Prototypes
// *****************************************************************************
// *****************************************************************************
void AppInitialize(void);
void App_Detect_Device(void);
void App_ProcessInputReport(void);
void App_PrepareOutputReport(void);
void InitializeTimer(void);
void App_Clear_Data_Buffer(void);
BOOL App_CompareKeyPressedPrevBuf(BYTE);
void App_CopyToShadowBuffer(void);


// *****************************************************************************
// *****************************************************************************
// Macros
// *****************************************************************************
// *****************************************************************************


//******************************************************************************
//  macros to identify special charaters(other than Digits and Alphabets) for HID/USB
//******************************************************************************
#define Symbol_Exclamation              (0x1E)
#define Symbol_AT                       (0x1F)
#define Symbol_Pound                    (0x20)
#define Symbol_Dollar                   (0x21)
#define Symbol_Percentage               (0x22)
#define Symbol_Cap                      (0x23)
#define Symbol_AND                      (0x24)
#define Symbol_Star                     (0x25)
#define Symbol_NormalBracketOpen        (0x26)
#define Symbol_NormalBracketClose       (0x27)

#define Symbol_Return                   (0x28)
#define Symbol_Escape                   (0x29)
#define Symbol_Backspace                (0x2A)
#define Symbol_Tab                      (0x2B)
#define Symbol_Space                    (0x2C)
#define Symbol_HyphenUnderscore         (0x2D)
#define Symbol_EqualAdd                 (0x2E)
#define Symbol_BracketOpen              (0x2F)
#define Symbol_BracketClose             (0x30)
#define Symbol_BackslashOR              (0x31)
#define Symbol_SemiColon                (0x33)
#define Symbol_InvertedComma            (0x34)
#define Symbol_Tilde                    (0x35)
#define Symbol_CommaLessThan            (0x36)
#define Symbol_PeriodGreaterThan        (0x37)
#define Symbol_FrontSlashQuestion       (0x38)

// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************


BYTE PS2_BufferModifierKeys=0;
//#warning servono MODIFIER PRIVATI PER PS2... l usb li sovrascrive a ogni giro (se c'è)
  

BYTE getModifierKeys(void) {
  BYTE i,j,n=0;
  
  n |= PS2_BufferModifierKeys;
  return n;
  }


const BYTE ps2_2_ascii[4][160] = {
  {			// 1°
//  https://wiki.osdev.org/PS/2_Keyboard
	0,
  0xa9,    //F9
  0,
  0xa5,		 //F5
  0xa3,    //F3
  0xa1,    //F1  = 0x05
  0xa2,    //F2
  0xac,    //F12
  0,
  0xaa,    //F10
  0xa8,    //F8
  0xa6,    //F6
  0xa4,    //F4
  9,      //TAB=0xD
  '\'',     // | 
  0,
  
  0,    //16
  0,    //LALT=0x11
  0,    //LSHIFT=0x12 
  0,
  0,    //LCTRL=0x14 
	'q',
	'1',
	0,
	0,
	0,
	'z',
	's',
	'a',
	'w',
	'2',
	0,      // left WIN (con E0)
  
  
	0,    //32
	'c',
	'x',
	'd',
	'e',
	'4',
	'3',
	0,      // right WIN (con E0)
	0,
	' ',    //0x29
	'v',
	'f',
	't',
	'r',
	'5',
	0,
  
	0,    //48
	'n',
	'b',
	'h',
	'g',
	'y',
	'6',
	0,   // Power (con E0)
	0,
	0,
	'm',
	'j',
	'u',
	'7',
	'8',
	0,   // Sleep (con E0)
  
	0,    //64
	',',
	'k',
	'i',
	'o',
	'0',
	'9',
	0,
	0,
	'.',
	'/',	
	'l',
	';',		// sarebbe ò ossia @
	'p',
	'-',
	'\'',		// sarebbe à ossia #
  
	0,    //80
	0,
	0,
	0,
	'[',
	'=',
	0,
	0,
	0xb1,    //caps lock
	0,    //RSHIFT
	'\n',   // CR (0x5A)
	']',
	0,   // 
	'\\',   // (0x5D)
  0,      // Wake (con E0)
  0,
  
  0,    // 0x60
  0,
  0,
  0,
  0,
  0,    
  0x8,    // 0x66 = BKSP
  0,
  0,
  '1',		// end= 0x69
  0,
  '4',		// e cursor left = 0x6b
  '7',		// home= 0x6c
  0,
  0,
  0,
  
  '0',    //0x70, INS se E0
  '.',    // keypad ovvero DEL se E0 71
  '2',		// e cursor down = 0x72
  '5',
  '6',		// e cursor right = 0x74
  '8',		// e cursor up = 0x75
  0x1b,   //0x76=ESC   0xe0 prefix! ...=RCTRL
  0xb2,    // NumLock
  0xab,    // F11
  '+',
  '3',
  '-',
  '*',
  '9',
  0xb3,  //0x7e scroll lock
  0,
  
	0,
	0,
	0,
	0xA7,   //0x83 è F7 !!

	},
  
  
  {			// 2°
	0,
  0xa9,    //F9
  0,
  0xa5,		 //F5
  0xa3,    //F3
  0xa1,    //F1  = 0x05
  0xa2,    //F2
  0xac,    //F12
  0,
  0xaa,    //F10
  0xa8,    //F8
  0xa6,    //F6
  0xa4,    //F4
  9,      //TAB=0xD
  '\\',     // | 
  0,
  
  0,    //16
  0,    //LALT=0x11
  0,    //LSHIFT=0x12 
  0,
  0,    //LCTRL=0x14 
	'q',
	'1',
	0,
	0,
	0,
	'z',
	's',
	'a',
	'w',
	'2',
	0,      // left WIN (con E0)
	0,    //32
	'c',
	'x',
	'd',
	'e',
	'4',
	'3',
	0,      // right WIN (con E0)
	0,
	' ',    //0x29
	'v',
	'f',
	't',
	'r',
	'5',
	0,
  
	0,    //48
	'n',
	'b',
	'h',
	'g',
	'y',
	'6',
	0,   // Power (con E0)
	0,
	0,
	'm',
	'j',
	'u',
	'7',
	'8',
	0,   // Sleep (con E0)
  
	0,    //64
	',',
	'k',
	'i',
	'o',
	'0',
	'9',
	0,
	0,
	'.',
	'/',	
	'l',
	'@',		// sarebbe ò ossia @
	'p',
	'-',
	'#',		// sarebbe à ossia #
  
	0,    //80
	0,
	0,
	0,
	'è',
	'=',
	0,
	0,
	0xb1,    //caps lock
	0,    //RSHIFT
	'\n',   // CR (0x5A)
	'+',
	0,   // 
	'\\',   // (0x5D)
  0,      // Wake (con E0)
  0,
  
  0,    // 0x60
  0,
  0,
  0,
  0,
  0,    
  0x8,    // 0x66 = BKSP
  0,
  0,
  '1',		// end= 0x69
  0,
  '4',		// e cursor left = 0x6b
  '7',		// home= 0x6c
  0,
  0,
  0,
  
  '0',    //0x70, INS se E0
  '.',    // keypad ovvero DEL se E0 71
  '2',		// e cursor down = 0x72
  '5',
  '6',		// e cursor right = 0x74
  '8',		// e cursor up = 0x75
  0x1b,   //0x76=ESC   0xe0 prefix! ...=RCTRL
  0xb2,    // NumLock
  0xab,    // F11
  '+',
  '3',
  '-',
  '*',
  '9',
  0xb3,  //0x7e scroll lock
  0,
  
	0,
	0,
	0,
	0xA7,   //0x83 è F7 !!

	},
  
  {			// 3°
	0,
  0xa9,    //F9
  0,
  0xa5,		 //F5
  0xa3,    //F3
  0xa1,    //F1  = 0x05
  0xa2,    //F2
  0xac,    //F12
  0,
  0xaa,    //F10
  0xa8,    //F8
  0xa6,    //F6
  0xa4,    //F4
  9,      //TAB=0xD
  '\'',     // | 
  0,
  
  0,    //16
  0,    //LALT=0x11
  0,    //LSHIFT=0x12 
  0,
  0,    //LCTRL=0x14 
	'a',    // prove :) AZERTY
	'1',
	0,
	0,
	0,
	'w',
	's',
	'q',
	'z',
	'2',
	0,      // left WIN (con E0)
	0,    //32
	'c',
	'x',
	'd',
	'e',
	'4',
	'3',
	0,      // right WIN (con E0)
	0,
	' ',    //0x29
	'v',
	'f',
	't',
	'r',
	'5',
	0,
  
	0,    //48
	'n',
	'b',
	'h',
	'g',
	'y',
	'6',
	0,   // Power (con E0)
	0,
	0,
	'm',
	'j',
	'u',
	'7',
	'8',
	0,   // Sleep (con E0)
  
	0,    //64
	',',
	'k',
	'i',
	'o',
	'0',
	'9',
	0,
	0,
	'.',
	'/',	
	'l',
	'@',		// sarebbe ò ossia @
	'p',
	'-',
	'#',		// sarebbe à ossia #
  
	0,    //80
	0,
	0,
	0,
	'[',
	'=',
	0,
	0,
	0xb1,    //caps lock
	0,    //RSHIFT
	'\n',   // CR (0x5A)
	']',
	0,   // 
	'\\',   // (0x5D)
  0,      // Wake (con E0)
  0,
  
  0,    // 0x60
  0,
  0,
  0,
  0,
  0,    
  0x8,    // 0x66 = BKSP
  0,
  0,
  '1',		// end= 0x69
  0,
  '4',		// e cursor left = 0x6b
  '7',		// home= 0x6c
  0,
  0,
  0,
  
  '0',    //0x70, INS se E0
  '.',    // keypad ovvero DEL se E0 71
  '2',		// e cursor down = 0x72
  '5',
  '6',		// e cursor right = 0x74
  '8',		// e cursor up = 0x75
  0x1b,   //0x76=ESC   0xe0 prefix! ...=RCTRL
  0xb2,    // NumLock
  0xab,    // F11
  '+',
  '3',
  '-',
  '*',
  '9',
  0xb3,  //0x7e scroll lock
  0,
  
	0,
	0,
	0,
	0xA7,   //0x83 è F7 !!

	},
  
  {			// 4°
	0,
  0xa9,    //F9
  0,
  0xa5,		 //F5
  0xa3,    //F3
  0xa1,    //F1  = 0x05
  0xa2,    //F2
  0xac,    //F12
  0,
  0xaa,    //F10
  0xa8,    //F8
  0xa6,    //F6
  0xa4,    //F4
  9,      //TAB=0xD
  '\'',     // | 
  0,
  
  0,    //16
  0,    //LALT=0x11
  0,    //LSHIFT=0x12 
  0,
  0,    //LCTRL=0x14 
	'q',
	'1',
	0,
	0,
	0,
	'z',
	's',
	'a',
	'w',
	'2',
	0,      // left WIN (con E0)
	0,    //32
	'c',
	'x',
	'd',
	'e',
	'4',
	'3',
	0,      // right WIN (con E0)
	0,
	' ',    //0x29
	'v',
	'f',
	't',
	'r',
	'5',
	0,
  
	0,    //48
	'n',
	'b',
	'h',
	'g',
	'y',
	'6',
	0,   // Power (con E0)
	0,
	0,
	'm',
	'j',
	'u',
	'7',
	'8',
	0,   // Sleep (con E0)
  
	0,    //64
	',',
	'k',
	'i',
	'o',
	'0',
	'9',
	0,
	0,
	'.',
	'/',	
	'l',
	'@',		// sarebbe ò ossia @
	'p',
	'-',
	'#',		// sarebbe à ossia #
  
	0,    //80
	0,
	0,
	0,
	'[',
	'=',
	0,
	0,
	0xb1,    //caps lock
	0,    //RSHIFT
	'\n',   // CR (0x5A)
	']',
	0,   // 
	'\\',   // (0x5D)
  0,      // Wake (con E0)
  0,
  
  0,    // 0x60
  0,
  0,
  0,
  0,
  0,    
  0x8,    // 0x66 = BKSP
  0,
  0,
  '1',		// end= 0x69
  0,
  '4',		// e cursor left = 0x6b
  '7',		// home= 0x6c
  0,
  0,
  0,
  
  '0',    //0x70, INS se E0
  '.',    // keypad ovvero DEL se E0 71
  '2',		// e cursor down = 0x72
  '5',
  '6',		// e cursor right = 0x74
  '8',		// e cursor up = 0x75
  0x1b,   //0x76=ESC   0xe0 prefix! ...=RCTRL
  0xb2,    // NumLock
  0xab,    // F11
  '+',
  '3',
  '-',
  '*',
  '9',
  0xb3,  //0x7e scroll lock
  0,
  
	0,
	0,
	0,
	0xA7,   //0x83 è F7 !!

	}
  };


BYTE App_PS22ASCII(BYTE a) {	//convert PS/2 code to ASCII code
  BYTE AsciiVal;
  BYTE ShiftkeyStatus,AltkeyStatus;

  //ovvio layout italiano :) KBIT se 850 ossia arr=1 altrimenti KBUS=437
  ShiftkeyStatus = getModifierKeys() & 0b00100010;
  AltkeyStatus = getModifierKeys() & 0b01000100;
  AsciiVal=ps2_2_ascii[codePageArr][a];
  
#warning finire di gestire codePageArr
  
  if(AsciiVal>='a' && AsciiVal<='z') {
    if(((CAPS_Lock_Pressed()) && !(getModifierKeys() & 0b00100010))
      || ((!CAPS_Lock_Pressed()) && (getModifierKeys() & 0b00100010)))
      AsciiVal &= ~0x20;   // capital
    else
      AsciiVal |= 0x20;   // lower case
    }
  else {
  	switch(a) {
      case 0x4a:
        if(!ShiftkeyStatus)
          AsciiVal =  codePageArr ? '-' : '-';
        else
          AsciiVal =  codePageArr ? '_' : '_';
        break;                      
      case 0x46:
        if(!ShiftkeyStatus)
          AsciiVal = '9';
        else
          AsciiVal =  codePageArr ? ')' : ')';
        break;
      case 0x45:
        if(!ShiftkeyStatus)
          AsciiVal = '0';
        else
          AsciiVal =  codePageArr ? '=' : '=';
        break;
      case 0x16:
        if(!ShiftkeyStatus)
          AsciiVal = '1';
        else
          AsciiVal = '!';
        break;
      case 0x1e:
        if(!ShiftkeyStatus)
          AsciiVal = '2';
        else
          AsciiVal = '"';
        break;
      case 0x26:
        if(!ShiftkeyStatus)
          AsciiVal = '3';
        else
          AsciiVal = codePageArr ? '£' : '#';
        break;
      case 0x25:
        if(!ShiftkeyStatus)
          AsciiVal = '4';
        else
          AsciiVal = '$';
        break;
      case 0x2e:
        if(!ShiftkeyStatus)
          AsciiVal = '5';
        else
          AsciiVal = '%';
        break;
      case 0x36:
        if(!ShiftkeyStatus)
          AsciiVal = '6';
        else
          AsciiVal = codePageArr ? '&' :  '&';
        break;
      case 0x3d:
        if(!ShiftkeyStatus)
          AsciiVal = '7';
        else
          AsciiVal = codePageArr ? '/' : '/';
        break;
      case 0x3e:
        if(!ShiftkeyStatus)
          AsciiVal = '8';
        else
          AsciiVal = codePageArr ? '(' : '(';
        break;
        
      case 0x61:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? '<' : '<';
        else
          AsciiVal = codePageArr ? '>' : '>';
        break;
      case 0xe:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? '\\' : '\\';
        else
          AsciiVal = codePageArr ? '|' : '|'; 
        break;
      case 0x41:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? ',' : ',';
        else
          AsciiVal = codePageArr ? ';' : ';';
        break;
      case 0x49:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? '.' : '.';
        else
          AsciiVal = codePageArr ? ':' : ':';
        break;
/*      case 0x8b: cosa sono??
        if(!ShiftkeyStatus)
          AsciiVal = '+';
        else
          AsciiVal = '*';            
        break;*/
      case 0x8d:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? 'ù' : '#';
        else
          AsciiVal = codePageArr ? '§' : '~';
        break;
      case 0x55:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? 'ì' : 'ì';
        else
          AsciiVal = codePageArr ? '^' : '^'; 
        break;
      case 0x4e:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? '\'' : '\'';
        else
          AsciiVal = codePageArr ? '?' : '?';
        break;
      case 0x4c:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? 'ò' : ';';
        else
          AsciiVal = codePageArr ? (AltkeyStatus ? '@' : 'ç') : ':';
        break;
      case 0x52:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? 'à' : '\'';
        else
          AsciiVal = codePageArr ? (AltkeyStatus ? '#' : '°') : '@';
        break;
      case 0x54:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? 'è' : '[';
        else
          AsciiVal = codePageArr ? '[' : '{';
        break;
      case 0x5b:
        if(!ShiftkeyStatus)
          AsciiVal = codePageArr ? '+' : ']';
        else
          AsciiVal = codePageArr ? '*' : '}';
        break;
//      case 0xb3:			// CTRL-BREAK arriva direttamente così...
//        AsciiVal = 0x9f;
//        break;
      default:
        break;
      }
    }

  return AsciiVal;
	}




//******************************************************************************


unsigned char setCodePageArray(WORD n) {
  
  switch(n) {
    case 437:
      codePageArr=0;
      return 1;
      break;
    case 850:
      codePageArr=1;
      return 1;
      break;
    case 855:   // Cyrillic; https://www.aivosto.com/articles/charsets-codepages-dos.html
    case 866:
      codePageArr=2;
      codePageArr=0;
      return 0;
      break;
    case 936:   // chinese
      codePageArr=3;
      codePageArr=0;
      return 0;
      break;
    case 710:   // arabic
      codePageArr=4;
      codePageArr=0;
      return 0;
      break;
    default:
      codePageArr=0;
      return 0;
      break;
      
    }
  }
  
