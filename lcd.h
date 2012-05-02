#ifndef LCD_H
#define LCD_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino,h"
#else
  #include "WProgram.h"
#endif

//#include <avr/io.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

// Major parts of this code is based on the code from
//   http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung

////////////////////////////////////////////////////////////////////////////////
// Hier die verwendete Taktfrequenz in Hz eintragen, wichtig!

#ifndef F_CPU
#define F_CPU 16000000
#endif


#define LCD_E    13    // low-high-low to latch data
#define LCD_RS   6    // Low=command, High=data 
#define LCD_D7   5
#define LCD_D6   4
#define LCD_D5   3
#define LCD_D4   2

// Required delays for LCD commands
#define LCD_BOOTUP_MS           15
#define LCD_ENABLE_US           20
#define LCD_WRITEDATA_US        46
#define LCD_COMMAND_US          42
#define LCD_SOFT_RESET_MS1      5
#define LCD_SOFT_RESET_MS2      1
#define LCD_SOFT_RESET_MS3      1
#define LCD_SET_4BITMODE_MS     5
#define LCD_CLEAR_DISPLAY_MS    2
#define LCD_CURSOR_HOME_MS      2


// Offset for first position in LCD rows
#define LCD_DDADR_LINE1         0x00
#define LCD_DDADR_LINE2         0x40
#define LCD_DDADR_LINE3         0x10
#define LCD_DDADR_LINE4         0x50


void LcdOut(uint8_t data);
void LcdPortMode(void);
void LcdInit(void);    // Initialize/rewset LCD display
void LcdClear(void);   // Clear LCD screen
void LcdHome(void);    // Position cursor top left corner
void LcdSetCursor(uint8_t x, uint8_t y); //Set cursor position
void LcdData(uint8_t data); // Display character 
void LcdString(const char *data); // Display string from RAM
void LcdCommand(uint8_t data);  // Send command byte to LCD
void LcdProgString(uint8_t *p); // Display string from PROGMEM 
void LcdGenerateChar(uint8_t code, const uint8_t *data);



// Clear Display -------------- 0b00000001
#define LCD_CLEAR_DISPLAY       0x01

// Cursor Home ---------------- 0b0000001x
#define LCD_CURSOR_HOME         0x02

// Set Entry Mode ------------- 0b000001xx
#define LCD_SET_ENTRY           0x04

#define LCD_ENTRY_DECREASE      0x00
#define LCD_ENTRY_INCREASE      0x02
#define LCD_ENTRY_NOSHIFT       0x00
#define LCD_ENTRY_SHIFT         0x01

// Set Display ---------------- 0b00001xxx
#define LCD_SET_DISPLAY         0x08

#define LCD_DISPLAY_OFF         0x00
#define LCD_DISPLAY_ON          0x04
#define LCD_CURSOR_OFF          0x00
#define LCD_CURSOR_ON           0x02
#define LCD_BLINKING_OFF        0x00
#define LCD_BLINKING_ON         0x01

// Set Shift ------------------ 0b0001xxxx
#define LCD_SET_SHIFT           0x10

#define LCD_CURSOR_MOVE         0x00
#define LCD_DISPLAY_SHIFT       0x08
#define LCD_SHIFT_LEFT          0x00
#define LCD_SHIFT_RIGHT         0x04

// Set Function --------------- 0b001xxxxx
#define LCD_SET_FUNCTION        0x20

#define LCD_FUNCTION_4BIT       0x00
#define LCD_FUNCTION_8BIT       0x10
#define LCD_FUNCTION_1LINE      0x00
#define LCD_FUNCTION_2LINE      0x08
#define LCD_FUNCTION_5X7        0x00
#define LCD_FUNCTION_5X10       0x04

#define LCD_SOFT_RESET          0x30

// Set CG RAM Address --------- 0b01xxxxxx  (Character Generator RAM)
#define LCD_SET_CGADR           0x40

#define LCD_GC_CHAR0            0
#define LCD_GC_CHAR1            1
#define LCD_GC_CHAR2            2
#define LCD_GC_CHAR3            3
#define LCD_GC_CHAR4            4
#define LCD_GC_CHAR5            5
#define LCD_GC_CHAR6            6
#define LCD_GC_CHAR7            7

// Set DD RAM Address --------- 0b1xxxxxxx  (Display Data RAM)
#define LCD_SET_DDADR           0x80


// Makros für LCD
#define Line1() SetCursor(1,0)	//An den Anfang der 1. Zeile springen
#define Line2() SetCursor(2,0)	//An den Anfang der 2. Zeile springen

#define SetCursor(y, x) LcdSetcCursor(x, y) //An eine bestimmte Position springen

#define LcdLoadCustomChar(addr) LcdCommand(LCD_SET_CGADR | (addr<<3))	//Custom-Zeichen laden

// Eigene Zeichen
#define LCD_CHAR_DIODE  0	//Dioden-Icon; wird als Custom-Character erstellt
#define LCD_CHAR_OMEGA  244	//Omega-Zeichen
#define LCD_CHAR_U  228		//µ-Zeichen

#endif

