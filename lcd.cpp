#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino,h"
#else
#include "WProgram.h"
#endif

#include "lcd.h"

// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
//

////////////////////////////////////////////////////////////////////////////////
// Hier die verwendete Taktfrequenz in Hz eintragen, wichtig!

#ifndef F_CPU
#define F_CPU 16000000
#endif


////////////////////////////////////////////////////////////////////////////////
//  Toggle LCD E-pin
void LcdEnable(void) {
  digitalWrite(LCD_E, HIGH);   
  _delay_us( LCD_ENABLE_US );  // kurze Pause
  digitalWrite(LCD_E, LOW);   
}



////////////////////////////////////////////////////////////////////////////////
// Sendet eine 4-bit Ausgabeoperation an das LCD
void LcdOut(uint8_t data) {
  digitalWrite(LCD_D7, (data&0x80));
  digitalWrite(LCD_D6, (data&0x40));
  digitalWrite(LCD_D5, (data&0x20));
  digitalWrite(LCD_D4, (data&0x10));
  LcdEnable();
}




////////////////////////////////////////////////////////////////////////////////
// Sendet einen Befehl an das LCD
void LcdCommand(uint8_t data) {
  digitalWrite(LCD_RS, LOW);    //RS auf 0 setzen
  LcdOut( data );             // zuerst die oberen, 
  LcdOut( data<<4 );           // dann die unteren 4 Bit senden
  _delay_us(LCD_COMMAND_US);
}



////////////////////////////////////////////////////////////////////////////////
// Sendet ein Datenbyte an das LCD
void LcdData(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);    //RS auf 1 setzen
  LcdOut( data );            // zuerst die oberen, 
  LcdOut( data<<4 );         // dann die unteren 4 Bit senden
  _delay_us(LCD_WRITEDATA_US);
}


////////////////////////////////////////////////////////////////////////////////
// 
void LcdPortMode(void) {
  pinMode(LCD_E, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  digitalWrite(LCD_E, LOW);   
  digitalWrite(LCD_RS, LOW);   
  digitalWrite(LCD_D7, LOW);   
  digitalWrite(LCD_D6, LOW);   
  digitalWrite(LCD_D5, LOW);   
  digitalWrite(LCD_D4, LOW);   
}

////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void LcdInit(void) {
  LcdPortMode();

  // warten auf die Bereitschaft des LCD
  _delay_ms(LCD_BOOTUP_MS);

  // Soft-Reset muss 3mal hintereinander gesendet werden zur Initialisierung
  LcdOut(LCD_SOFT_RESET);
  _delay_ms(LCD_SOFT_RESET_MS1);

  LcdEnable();
  _delay_ms(LCD_SOFT_RESET_MS2);

  LcdEnable();
  _delay_ms(LCD_SOFT_RESET_MS3);

  // 4-bit Modus aktivieren 
  LcdOut(LCD_SET_FUNCTION | LCD_FUNCTION_4BIT);
  _delay_ms(LCD_SET_4BITMODE_MS);

  // 4-bit Modus / 2 Zeilen / 5x7
  LcdCommand(LCD_SET_FUNCTION | LCD_FUNCTION_4BIT | LCD_FUNCTION_2LINE | LCD_FUNCTION_5X7);

  // Display ein / Cursor aus / Blinken aus
  LcdCommand(LCD_SET_DISPLAY | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINKING_OFF); 

  // Cursor inkrement / kein Scrollen
  LcdCommand(LCD_SET_ENTRY | LCD_ENTRY_INCREASE | LCD_ENTRY_NOSHIFT);

  LcdClear();
}



////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl zur Löschung des Displays
void LcdClear(void) {
  LcdCommand(LCD_CLEAR_DISPLAY);
  _delay_ms(LCD_CLEAR_DISPLAY_MS);
}

////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl: Cursor Home
void LcdHome(void) {
  LcdCommand( LCD_CURSOR_HOME );
  _delay_ms( LCD_CURSOR_HOME_MS );
}

////////////////////////////////////////////////////////////////////////////////
// Setzt den Cursor in Spalte x (0..15) Zeile y (1..4) 

void LcdSetCursor(uint8_t x, uint8_t y) {
  uint8_t data;

  switch (y) {
  case 1:    // 1. Zeile
    data=LCD_SET_DDADR + LCD_DDADR_LINE1 + x;
    break;

  case 2:    // 2. Zeile
    data=LCD_SET_DDADR + LCD_DDADR_LINE2 + x;
    break;

  case 3:    // 3. Zeile
    data=LCD_SET_DDADR + LCD_DDADR_LINE3 + x;
    break;

  case 4:    // 4. Zeile
    data=LCD_SET_DDADR + LCD_DDADR_LINE4 + x;
    break;

  default:
    return;                                   // für den Fall einer falschen Zeile
  }

  LcdCommand(data);
}

////////////////////////////////////////////////////////////////////////////////
// Schreibt einen String auf das LCD

void LcdString(const char *data) {
  while(*data != '\0') {
    LcdData(*data++);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Schreibt ein Zeichen in den Character Generator RAM
void LcdGenerateChar(uint8_t code, const uint8_t *data) {
  uint8_t i;
  // Startposition des Zeichens einstellen
  LcdCommand(LCD_SET_CGADR | (code<<3));

  // Bitmuster übertragen
  for (i=0; i<8; i++) {
    LcdData( data[i] );
  }
}


////////////////////////////////////////////////////////////////////////////////
//
void LcdProgString(uint8_t *p) {
  uint8_t c;
  do {
    c=pgm_read_byte_near(p++);
    if (c) LcdData(c);
  } while (c>0);
}




////////////////////////////////////////////////////////////////////////////////
// Zeichen aus EEPROM laden und schreiben
//void LcdEepCustomChar(const uint8_t *chardata) {	
//  uint8_t i;
//  for(i=0; i<8; i++) {
//    LcdData(eeprom_read_byte(chardata));
//    chardata++;
//  }
//}



