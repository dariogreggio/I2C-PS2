/* 
 * File:   keyboard.h
 * Author: cinthia
 *
 * Created on 27 novembre 2022, 17.15
 */

#ifndef KEYBOARD_H
#define	KEYBOARD_H

#ifdef	__cplusplus
extern "C" {
#endif


extern BYTE PS2_BufferModifierKeys;

BYTE App_PS22ASCII(BYTE); //convert PS/2 code to ASCII code

typedef struct _HID_LED_REPORT_BUFFER {
  BYTE  NUM_LOCK      : 1;
  BYTE  CAPS_LOCK     : 1;
  BYTE  SCROLL_LOCK   : 1;
  BYTE  UNUSED        : 5;
	} HID_LED_REPORT_BUFFER;
extern HID_LED_REPORT_BUFFER Appl_led_report_buffer;
#define CAPS_Lock_Pressed() Appl_led_report_buffer.CAPS_LOCK
#define NUM_Lock_Pressed() Appl_led_report_buffer.NUM_LOCK

typedef union __attribute((packed)) _PS2_LED_REPORT_BUFFER {
    BYTE b;
    struct {
        BYTE  SCROLL_LOCK   : 1;
        BYTE  NUM_LOCK      : 1;
        BYTE  CAPS_LOCK     : 1;
        BYTE  UNUSED        : 5;
        };
	} PS2_LED_REPORT_BUFFER;
extern WORD CodePage;

BYTE getModifierKeys(void);

/* Array index for modifier keys */
#define MODIFIER_LEFT_CONTROL           (0)
#define MODIFIER_LEFT_SHIFT             (1)
#define MODIFIER_LEFT_ALT               (2)
#define MODIFIER_LEFT_GUI               (3)
#define MODIFIER_RIGHT_CONTROL          (4)
#define MODIFIER_RIGHT_SHIFT            (5)
#define MODIFIER_RIGHT_ALT              (6)
#define MODIFIER_RIGHT_GUI              (7)



#ifdef	__cplusplus
}
#endif

#endif	/* KEYBOARD_H */

