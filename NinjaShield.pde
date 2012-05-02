#include "avr/io.h"
#include "lcd.h"
#include "wavetable.h"



uint8_t AppName[] PROGMEM = "NinjaShield 0.01";

uint8_t DiodeIcon[] PROGMEM = {4,31,31,14,14,4,31,4};	//Dioden-Icon


uint8_t TestRunning[] PROGMEM = "Testing ...€€€€€€";
uint8_t Bat[] PROGMEM = "Battery €";
uint8_t BatWeak[] PROGMEM = "weak€€€";
uint8_t BatEmpty[] PROGMEM = "empty!€€";
uint8_t TestFailed1[] PROGMEM = "No, unknown, or€";
uint8_t TestFailed2[] PROGMEM = "damaged €€€€";
uint8_t Bauteil[] PROGMEM = "part€€€€€€";
uint8_t Unknown[] PROGMEM = " unknown€";
uint8_t Diode[] PROGMEM = "Diode: ";
uint8_t DualDiode[] PROGMEM = "Double diode €";
uint8_t TwoDiodes[] PROGMEM = "2 diodes";
uint8_t Antiparallel[] PROGMEM = "anti-parallel";
uint8_t InSeries[] PROGMEM = "serial A=€€";
uint8_t K1[] PROGMEM = ";C1=";
uint8_t K2[] PROGMEM = ";C2=";
uint8_t GAK[] PROGMEM = "GAC=";
uint8_t NextK[] PROGMEM = ";C=";
uint8_t K[] PROGMEM = "C=";
uint8_t Triac[] PROGMEM = "Triac";
uint8_t Thyristor[] PROGMEM = "Thyristor";
uint8_t OrBroken[] PROGMEM = "or damaged €€";
uint8_t Resistor[] PROGMEM = "Resistor: €€";
uint8_t Capacitor[] PROGMEM = "Capacitor: €€";
uint8_t mosfet[] PROGMEM = "-MOS";
uint8_t emode[] PROGMEM = "-E";
uint8_t dmode[] PROGMEM = "-D";
uint8_t jfet[] PROGMEM = "-JFET";
uint8_t AA1[] PROGMEM = ";A1=";
uint8_t AA2[] PROGMEM = ";A2=";
uint8_t NullDot[] PROGMEM = "0,";
uint8_t GateCap[] PROGMEM = " C=";
uint8_t hfestr[] PROGMEM ="hFE=";
uint8_t NPN[] PROGMEM = "NPN";
uint8_t PNP[] PROGMEM = "PNP";
uint8_t bstr[] PROGMEM = " B=";
uint8_t cstr[] PROGMEM = ";C=";
uint8_t estr[] PROGMEM = ";E=";
uint8_t gds[] PROGMEM = "GDS=";
uint8_t Uf[] PROGMEM = "Uf=";
uint8_t vt[] PROGMEM = "Vt=";
uint8_t mV[] PROGMEM = "mV";
uint8_t Anode[] PROGMEM = "A=";
uint8_t Gate[] PROGMEM = "G=";
uint8_t CA[] PROGMEM = "CA";
uint8_t CC[] PROGMEM = "CC";
uint8_t TestTimedOut[] PROGMEM = "Timeout!";



#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7

#define PART_MODE_N_E_MOS 1  // Special for FET transistors
#define PART_MODE_P_E_MOS 2
#define PART_MODE_N_D_MOS 3
#define PART_MODE_P_D_MOS 4
#define PART_MODE_N_JFET 5
#define PART_MODE_P_JFET 6

#define PART_MODE_NPN 1    // Special for BIPOLAR transistors
#define PART_MODE_PNP 2



uint8_t PartType, PartMode, RepeatDetect;
uint16_t hfe[2], vBE[2];
uint8_t c,b,e;








unsigned int ReadADC(uint8_t mux) {
  //ADC-Wert des angegebenen Kanals auslesen und als unsigned int zurückgegen
  unsigned int adcx = 0;
  //  ADMUX = mux | (1<<REFS0);
  for(uint8_t j=0;j<20;j++) {	//20 Messungen; für bessere Genauigkeit
    //    ADCSRA |= (1<<ADSC);
    //    while (ADCSRA&(1<<ADSC));
    //    adcx += ADCW;
    adcx+=analogRead(mux);
  }
  adcx /= 20;
  return adcx;
}



#define TP1 0    // Test pins connected diretly to DUT
#define TP2 1    
#define TP3 2    

#define T1_680    8   // Digital pin 8
#define T2_680    9   // Digital pin 9
#define T3_680   10   // Digital pin 10
#define T1_470K  11   // Digital pin 11
#define T2_470K  12   // Digital pin 12
#define T3_470K  17   // Analog pin 3 as digital
#define T1_0     14   // Analog pin 0 as digital
#define T2_0     15   // Analog pin 1 as digital
#define T3_0     16   // Analog pin 2 as digital

#define T1_ana   0    // Analog pin 0 for ADC
#define T2_ana   1    // Analog pin 1 for ADC
#define T3_ana   2    // Analog pin 2 for ADC


void R_0(uint8_t pin, uint8_t level) {
  switch(pin) {
    case TP1:
      pinMode(T1_0, OUTPUT);
      digitalWrite(T1_0, level);
      pinMode(T1_680, INPUT);
      pinMode(T1_470K, INPUT);
      break;
    case TP2:
      pinMode(T2_0, OUTPUT);
      digitalWrite(T2_0, level);
      pinMode(T2_680, INPUT);
      pinMode(T2_470K, INPUT);
      break;
    case TP3:
      pinMode(T3_0, OUTPUT);
      digitalWrite(T3_0, level);
      pinMode(T3_680, INPUT);
      pinMode(T3_470K, INPUT);
      break;
  }      
}

void R_680(uint8_t pin, uint8_t level) {
  switch(pin) {
    case TP1:
      pinMode(T1_680, OUTPUT);
      digitalWrite(T1_680, level);
      pinMode(T1_0, INPUT);
      pinMode(T1_470K, INPUT);
      break;
    case TP2:
      pinMode(T2_680, OUTPUT);
      digitalWrite(T2_680, level);
      pinMode(T2_0, INPUT);
      pinMode(T2_470K, INPUT);
      break;
    case TP3:
      pinMode(T3_680, OUTPUT);
      digitalWrite(T3_680, level);
      pinMode(T3_0, INPUT);
      pinMode(T3_470K, INPUT);
      break;
  }      
}


void R_470K(uint8_t pin, uint8_t level) {
  switch(pin) {
    case TP1:
      pinMode(T1_470K, OUTPUT);
      digitalWrite(T1_470K, level);
      pinMode(T1_0, INPUT);
      pinMode(T1_680, INPUT);
      break;
    case TP2:
      pinMode(T2_470K, OUTPUT);
      digitalWrite(T2_470K, level);
      pinMode(T2_0, INPUT);
      pinMode(T2_680, INPUT);
      break;
    case TP3:
      pinMode(T3_470K, OUTPUT);
      digitalWrite(T3_470K, level);
      digitalWrite(13, level);
      pinMode(T3_0, INPUT);
      pinMode(T3_680, INPUT);
      break;
  }      
}


void HiZ(uint8_t pin) {
  switch(pin) {
    case TP1:
      pinMode(T1_0, INPUT);
      pinMode(T1_680, INPUT);
      pinMode(T1_470K, INPUT);
      break;
    case TP2:
      pinMode(T2_0, INPUT);
      pinMode(T2_680, INPUT);
      pinMode(T2_470K, INPUT);
      break;
    case TP3:
      pinMode(T3_0, INPUT);
      pinMode(T3_680, INPUT);
      pinMode(T3_470K, INPUT);
      break;
  }      
  
}

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin){
	uint16_t InitialADC, HighPinADC,TristatePinADC, LowPinADC;

	//LowPin 680ohm R to ground
	R_680(LowPin, LOW);
	//HighPin to output/high
	R_0(HighPin, HIGH);
        HiZ(TristatePin);
        
	_delay_ms(5);

	//Read ADC on LowPin
	InitialADC=ReadADC(LowPin);


	//reset pins
	//LowPin 680ohm R to ground
	R_680(LowPin, LOW);
	//HighPin to output/high
	R_0(HighPin, HIGH);
        HiZ(TristatePin);

	if(InitialADC<200){
		/****************************************/
		//
		//	PNP tests
		//	If this PNP:
		// 	then collector should be high when base ground and emitter at +5
		//
		/*****************************************/
		//C, B low through 680R, E high
		R_680(TristatePin, LOW); //base to ground through 680R
		_delay_ms(10);
		
		//read LowPin ADC
		//PNP allows current from Emitter to Collector and should be high
		LowPinADC=ReadADC(LowPin);

     Serial.print("PNP test LowPinADC="); Serial.println(LowPinADC);
		if(LowPinADC>700){//PHP active, current flowing to Emitter
//			HiZ(TristatePin);//Base HiZ
			R_470K(TristatePin, LOW); //Base to 470K low
			
			_delay_ms(10);

			//read LowPin ADC for hfe
			LowPinADC=ReadADC(LowPin);
			
			//read TristatePin ADC, see if base is transistor (>2volts) or FET (<2volts)
			//save value for uBE too
			TristatePinADC=ReadADC(TristatePin);

  Serial.print("PNP LowPinADC="); Serial.print(LowPinADC);
  Serial.print(" TristatePinADC="); Serial.print(TristatePinADC);
  Serial.println();
			if((PartType==PART_TRANSISTOR) || (PartType==PART_FET))RepeatDetect=1;
			hfe[RepeatDetect]=LowPinADC;
			vBE[RepeatDetect]=TristatePinADC;

			//if(PartFound != PART_THYRISTOR) {
			if(TristatePinADC>200){ //high base voltage is transistor
				//PNP transistor found
				PartType=PART_TRANSISTOR;
				PartMode=PART_MODE_PNP;
			}else{ //low base voltage is MOSFET
				//MOSFET and tests
				PartType=PART_FET;
				PartMode=PART_MODE_PNP;
			}
			c=LowPin;
			e=HighPin;
			b=TristatePin;
Serial.print("PNP");
Serial.print(HighPin,DEC);
Serial.print(LowPin,DEC);
Serial.print(TristatePin,DEC);
Serial.println();		
                }

		/****************************************/
		//
		//	NPN tests
		//	If this NPN:
		// 	then collector should hold pullup low when base high and emitter to ground
		//
		/*****************************************/
		//B,C high through pullup, E low
		//LowPin ground
		R_0(LowPin, LOW);
		//HighPin and TristatePin 680R high
		R_680(HighPin, HIGH);
		R_680(TristatePin, HIGH);
		
		//read HighPin ADC to see if NPN holds the 680R pullup low...
		HighPinADC=ReadADC(HighPin);
  Serial.print("NPN test HighPinADC="); Serial.println(HighPinADC);

		if(HighPinADC<500){//NPN grounds the weak pullup
			//THYRISTOR test

			/***************************************/
			//	Transistor or MOSFET test
			//	Transistor is current controlled, the base voltage after the resistor will be ~0.6-1volts
			//	MOSFET is voltage controlled, the base voltage after the resistor will be close to the supply (5volts)
			/***************************************/
			//TristatePin 470K high
			R_470K(TristatePin, HIGH);				
			//read HighPin ADC for hfe
			HighPinADC=ReadADC(HighPin);
			
			//read TristatePin ADC, see if base is transistor (<2volts) or FET (>2volts)
			//save value for uBE too
			TristatePinADC=ReadADC(TristatePin);

  Serial.print("NPN HighPinADC="); Serial.print(HighPinADC);
  Serial.print(" TristatePinADC="); Serial.print(TristatePinADC);
  Serial.println();


			if((PartType==PART_TRANSISTOR) || (PartType==PART_FET))RepeatDetect=1;
			hfe[RepeatDetect]=1023-HighPinADC;
			vBE[RepeatDetect]=1023-TristatePinADC;

			if(TristatePinADC<500){ //low base voltage is transistor
				//NPN transistor found
				PartType=PART_TRANSISTOR;
				PartMode=PART_MODE_NPN;
			}else{ //high base voltage is MOSFET
				//MOSFET and tests
				PartType=PART_FET;
				PartMode=PART_MODE_NPN;
			}
			c=HighPin;
			e=LowPin;
			b=TristatePin;
Serial.print("NPN");
Serial.print(HighPin,DEC);
Serial.print(LowPin,DEC);
Serial.print(TristatePin,DEC);
Serial.println();
		}//adc<500
	}//adc < 200	


testend:	
	HiZ(HighPin);
	HiZ(LowPin);
	HiZ(TristatePin);

}






void setup() {
  MCUCR |=_BV(PUD);   // Globally turn off all PullUp's
  LcdInit();
//  LcdLoadCustomChar(0);
//  LcdEepCustomChar(DiodeIcon);
  LcdProgString(AppName);
  delay(1000);
  Serial.begin(9600);  
  Serial.println();  
  Serial.println();  
  Serial.println();  
  Serial.println();  
  Serial.println();  


  uint8_t offset=256-(((long)sinewave)&0xFF);
  if (offset!=0) {
    LcdClear();
    if (sizeof(filler)>0) {
      LcdString("Tbl align error");
    } else {
      LcdString("Enable ");
      for (uint8_t i=0; i<8; i++) 
      if ((offset&((uint8_t)1<<i))) LcdData(49+i);
    }
    for (;;);
  }
}


void loop() {
  uint8_t  i,cmd, param[9], tmp; 
  unsigned long lhfe=0;

  LcdClear();

  PartType=0;
  PartMode=0;
  RepeatDetect=0;
  CheckPins(TP1, TP2, TP3); //CBE npn --
  CheckPins(TP1, TP3, TP2); //CBE npn
  CheckPins(TP2, TP1, TP3); //CBE npn --
  CheckPins(TP2, TP3, TP1); //CBE npn
  CheckPins(TP3, TP1, TP2); //CBE npn
  CheckPins(TP3, TP2, TP1); //CBE npn

  LcdPortMode();
  LcdClear();
  if (PartType==PART_TRANSISTOR) {
 
   if (RepeatDetect>0) {
    Serial.print("hfe[0]="); Serial.print(hfe[0],DEC);
    Serial.print(" hfe[1]="); Serial.print(hfe[1],DEC);
    Serial.print(" vBE[0]="); Serial.print(vBE[0],DEC);
    Serial.print(" vBe[1]="); Serial.print(vBE[1],DEC);
    Serial.println();
   } else {
    Serial.print("hfe[0]="); Serial.print(hfe[0],DEC);
    Serial.print(" vBE[0]="); Serial.print(vBE[0],DEC);
    Serial.println();
   }
    
			if(RepeatDetect==0){
				hfe[1] = hfe[0];
				vBE[1] = vBE[0];
			}

			if(hfe[0]>hfe[1]){
				hfe[1] = hfe[0];
				vBE[1] = vBE[0];
				tmp = c;
				c = e;
				e = tmp;
			}

			lhfe = hfe[1];

			lhfe *= (((unsigned long)4700 * 100) / (unsigned long)680);	//Verhältnis von High- zu Low-Widerstand

			if(vBE[1]<11) vBE[1] = 11;
			lhfe /= vBE[1];
			hfe[1] = (unsigned int) lhfe;



    if (PartMode==PART_MODE_NPN) {
      LcdString("NPN Transistor");
    }
    if (PartMode==PART_MODE_PNP) {
      LcdString("PNP Transistor");
    }

    LcdSetCursor(0,2);
    char tmps[20];
    sprintf(tmps,"    hFE=%d",hfe[0]);
    LcdString(tmps);    

    LcdSetCursor(0,2);

    if (c==0) LcdData('C');
    if (b==0) LcdData('B');
    if (e==0) LcdData('E');
    if (c==1) LcdData('C');
    if (b==1) LcdData('B');
    if (e==1) LcdData('E');
    if (c==2) LcdData('C');
    if (b==2) LcdData('B');
    if (e==2) LcdData('E');

  }
  for(;;);
}


#ifdef XYZZY

#include <avr/io.h>
#include "lcd-routines.h"
#include "config.h"
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <math.h>

#define MCU_STATUS_REG MCUCR
#define UseM8


/*########################################################################################
 	Konfigurations-Einstellungen
 */

/* Port für die Test-Pins
 	Dieser Port muss über einen ADC verfügen (beim Mega8 also PORTC).
 	Für die Test-Pins müssen die unteren 3 Pins dieses Ports benutzt werden.
 	Bitte die Definitionen für TP1, TP2 und TP3 nicht ändern!
 */

#define ADC_PORT PORTC
#define ADC_DDR DDRC
#define ADC_PIN PINC
#define TP1 PC0
#define TP2 PC1
#define TP3 PC2

/*Einstellungen für Kapazitätsmessung (nur für ATMega8 interessant)
 	Der Test, ob ein Kondensator vorhanden ist, dauert relativ lange, mit über 50ms je Testvorgang ist zu rechnen
 	Bei allen 6 möglichen Testvorgängen ergibt das eine Verlängerung der Testdauer um ca 0,3s bis 0,5s.
 	Mit CAP_TEST_MODE lassen sich die durchgeführten Tests festlegen.
 
 	Bedeutungen der Bits (7 = MSB):
 	7:6 Nicht verwendet
 
 	5:4 Test-Modus
 	00: Kondensator-Messung deaktiviert
 	01: Kondensator-Messung für eine einstellbare Pin-Kombination (in beide Richtungen); verlängert Testdauer um ca. 120...200ms
 	10: Kondensator-Messung für alle 6 Pin-Kombinationen; verlängert Testdauer um ca. 300...500ms
 	
 	3:2 Erster Pin der gewählten Pin-Kombination (0...2), nur entscheidend wenn Bits 5:4 = 01
 
 	1:0 Zweiter Pin der gewählten Pin-Kombination (0...2), nur entscheidend wenn Bits 5:4 = 01
 	*/
uint8_t CapTestMode PROGMEM = 0b00100010;	//Messung für alle 6 Pin-Kombinationen

//3 EEPROM-Bytes für zukünftige Verwendung reserviert
uint8_t RFU1 PROGMEM = 0;
uint8_t RFU2 PROGMEM = 0;
uint8_t RFU3 PROGMEM = 0;


/*
	Genaue Werte der verwendeten Widerstände in Ohm.
 	Der Nennwert für R_L ist 680 Ohm, für R_H 470kOhm
 	Um das Programm auf Abweichungen von diesen Werten (z.B. durch Bauteiltoleranzen)
 	zu kalibrieren, die Widerstandswerte in Ohm in die folgenden Defines eintragen:
 */
unsigned int R_L_VAL PROGMEM = 680;			//R_L; Normwert 680 Ohm
unsigned int R_H_VAL PROGMEM = 4700;			//R_H; Normwert 470000 Ohm, durch 100 dividiert angeben



/*	Faktoren für die Kapatitätsmessung bei Kondensatoren
 		Diese Faktoren hängen von Fertigungstoleranzen des AVR ab und müssen somit ggf. angepasst werden
 		H_CAPACITY_FACTOR ist für die Messung mit 470k-Widerstand (geringe Kapazität)
 		L_CAPACITY_FACTOR ist für die Messung mit 680-Ohm-Widerstand (hohe Kapazität)
 		Der gesamte Messbereich ist ca. 0,2nF bis 7300µF.
 	*/
unsigned int H_CAPACITY_FACTOR PROGMEM = 394;
unsigned int L_CAPACITY_FACTOR PROGMEM = 283;


/*########################################################################################
 Ende der Konfigurations-Einstellungen
 */


//Ende der EEPROM-Strings

//Watchdog
#define WDT_enabled
/* Wird das Define "WDT_enabled" entfernt, wird der Watchdog beim Programmstart
 nicht mehr aktiviert. Das ist für Test- und Debuggingzwecke sinnvoll.
 Für den normalen Einsatz des Testers sollte der Watchdog aber unbedingt aktiviert werden!
 */


struct Diode {
  uint8_t Anode;
  uint8_t Cathode;
  int Voltage;
};

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin);
void DischargePin(uint8_t PinToDischarge, uint8_t DischargeDirection);
unsigned int ReadADC(uint8_t mux);
void lcd_show_format_cap(char outval[], uint8_t strlength, uint8_t CommaPos);

void ReadCapacity(uint8_t HighPin, uint8_t LowPin);		//Kapazitätsmessung nur auf Mega8 verfügbar

#define ON_DDR DDRD
#define ON_PORT PORTD
#define ON_PIN_REG PIND
#define ON_PIN PD6	//Pin, der auf low gezogen werden muss, um Schaltung in Betrieb zu halten
#define RST_PIN PD7	//Pin, der auf low gezogen wird, wenn der Einschalt-Taster gedrückt wird

/* Port für die Testwiderstände
 	Die Widerstände müssen an die unteren 6 Pins des Ports angeschlossen werden,
 	und zwar in folgender Reihenfolge:
 	RLx = 680R-Widerstand für Test-Pin x
 	RHx = 470k-Widerstand für Test-Pin x
 
 	RL1 an Pin 0
 	RH1 an Pin 1
 	RL2 an Pin 2
 	RH2 an Pin 3
 	RL3 an Pin 4
 	RH3 an Pin 5
 
 */

#define R_DDR DDRD
#define R_PORT PORTD
#define R_DDR_DEF (1<<ON_PIN)
#define R_PORT_DEF (1<<ON_PIN) | (1<<RST_PIN)

//Bauteile
#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7

//Ende (Bauteile)
//Spezielle Definitionen für Bauteile
//FETs
#define PART_MODE_N_E_MOS 1
#define PART_MODE_P_E_MOS 2
#define PART_MODE_N_D_MOS 3
#define PART_MODE_P_D_MOS 4
#define PART_MODE_N_JFET 5
#define PART_MODE_P_JFET 6

//Bipolar
#define PART_MODE_NPN 1
#define PART_MODE_PNP 2


struct Diode diodes[6];
uint8_t NumOfDiodes;

uint8_t b,c,e;			//Anschlüsse des Transistors
unsigned long lhfe;		//Verstärkungsfaktor
uint8_t PartReady;		//Bauteil fertig erkannt
unsigned int hfe[2];		//Verstärkungsfaktoren
unsigned int uBE[2];	//B-E-Spannung für Transistoren
uint8_t PartMode;
uint8_t tmpval, tmpval2;
uint8_t ra, rb;				//Widerstands-Pins
unsigned int rv[2];			//Spannungsabfall am Widerstand
unsigned int radcmax[2];	//Maximal erreichbarer ADC-Wert (geringer als 1023, weil Spannung am Low-Pin bei Widerstandsmessung über Null liegt)
uint8_t ca, cb;				//Kondensator-Pins
uint8_t cp1, cp2;			//Zu testende Kondensator-Pins, wenn Messung für einzelne Pins gewählt
uint8_t ctmode;				//Kondensator-Test-Modus (siehe ab Zeile 40)

unsigned long cv;

uint8_t PartFound, tmpPartFound;	//das gefundene Bauteil
char outval[8];
unsigned int adcv[4];
unsigned int gthvoltage;	//Gate-Schwellspannung
uint8_t tmpval, tmpval2;

char outval2[6];

//Programmbeginn
int main(void) {
  //Einschalten
  ON_DDR = (1<<ON_PIN);
  //ON_PIN muss high sein, damit die Schaltung läuft
  ON_PORT = (1<<ON_PIN) | (1<<RST_PIN);	//Pullup für Reset-Pin
  uint8_t tmp;
  //ADC-Init
  ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);	//Vorteiler=8
  lcd_init();

  //Konstanten aus EEPROM laden
  unsigned int rhval = eeprom_read_word(&R_H_VAL);	//R_H
  unsigned int rlval = eeprom_read_word(&R_L_VAL);	//R_L
  ctmode = eeprom_read_byte(&CapTestMode);
  cp1 = (ctmode & 12) >> 2;
  cp2 = ctmode & 3;
  ctmode = (ctmode & 48) >> 4;

  wdt_disable();
  if(MCU_STATUS_REG & (1<<WDRF)) {	
    /*
		Überprüfen auf Watchdog-Reset 
     		Das tritt ein, wenn der Watchdog 2s nicht zurückgesetzt wurde
     		Kann vorkommen, wenn sich das Programm in einer Endlosschleife "verheddert" hat.
     		*/
    lcd_eep_string(TestTimedOut);	//Timeout-Meldung
    _delay_ms(3000);
    /* Abschalten */
    ON_PORT &= ~(1<<ON_PIN);
    _delay_ms(500);
    return 0;
  }
  LCDLoadCustomChar(0);	//Custom-Zeichen
  //Diodensymbol in LCD laden
  lcd_eep_customchar(DiodeIcon);

  Line1();	//1. Zeile

  //Einsprungspunkt, wenn Start-Taste im Betrieb erneut gedrückt wird
start:
#ifdef WDT_enabled
  wdt_enable(WDTO_2S);	//Watchdog an
#endif
  PartFound = PART_NONE;
  tmpPartFound = PART_NONE;
  NumOfDiodes = 0;
  PartReady = 0;
  PartMode = 0;
  ca = 0;
  cb = 0;
  lcd_clear();
  //Versorgungsspannung messen
  ReadADC(5 | (1<<REFS1));	//Dummy-Readout
  hfe[0] = ReadADC(5 | (1<<REFS1)); 	//mit interner Referenz
  if (hfe[0] < 720) {			//Vcc < 1,9V; Warnung anzeigen
    lcd_eep_string(Bat);		//Anzeige: "Batterie"
    if(hfe[0] < 680) {					//Vcc < 1,8V; zuverlässiger Betrieb nicht mehr möglich
      lcd_eep_string(BatEmpty);		//Batterie leer!
      _delay_ms(1000);
      /* Abschalten */
      ON_PORT &= ~(1<<ON_PIN);
      _delay_ms(500);
      return 0;
    }
    lcd_eep_string(BatWeak);		//Batterie schwach
    Line2();
  }
  //Test beginnen
  lcd_eep_string(TestRunning);	//String: Test läuft
  //Alle 6 Kombinationsmöglichkeiten für die 3 Pins prüfen
  CheckPins(TP1, TP2, TP3);
  CheckPins(TP1, TP3, TP2);
  CheckPins(TP2, TP1, TP3);
  CheckPins(TP2, TP3, TP1);
  CheckPins(TP3, TP2, TP1);
  CheckPins(TP3, TP1, TP2);
  //Separate Messung zum Test auf Kondensator
  if(((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) && (ctmode > 0)) {
    //Kondensator entladen; sonst ist evtl. keine Messung möglich
    R_PORT = R_PORT_DEF;
    R_DDR = (1<<(TP1 * 2)) | (1<<(TP2 * 2)) | (1<<(TP3 * 2)) | R_DDR_DEF;
    _delay_ms(10);
    R_DDR = R_DDR_DEF;
    //Kapazität in allen 6 Pin-Kombinationen messen
    if(ctmode == 1) {
      ReadCapacity(cp1, cp2);
      ReadCapacity(cp2, cp1);
    } 
    else {
      ReadCapacity(TP3, TP1);
      ReadCapacity(TP3, TP2);
      ReadCapacity(TP2, TP3);
      ReadCapacity(TP2, TP1);
      ReadCapacity(TP1, TP3);
      ReadCapacity(TP1, TP2);
    }
  }
  //Fertig, jetzt folgt die Auswertung
  lcd_clear();
  if(PartFound == PART_DIODE) {
    if(NumOfDiodes == 1) {
      //Standard-Diode
      lcd_eep_string(Diode);	//"Diode: "
      lcd_eep_string(Anode);
      lcd_data(diodes[0].Anode + 49);
      lcd_eep_string(NextK);//";K="
      lcd_data(diodes[0].Cathode + 49);
      Line2();	//2. Zeile
      lcd_eep_string(Uf);	//"Uf = "
      lcd_string(itoa(diodes[0].Voltage, outval, 10));
      lcd_eep_string(mV);
      goto end;
    } 
    else if(NumOfDiodes == 2) {
      //Doppeldiode
      if(diodes[0].Anode == diodes[1].Anode) {
        //Common Anode
        lcd_eep_string(DualDiode);	//Doppeldiode
        lcd_eep_string(CA);	//"CA"
        Line2(); //2. Zeile
        lcd_eep_string(Anode);
        lcd_data(diodes[0].Anode + 49);
        lcd_eep_string(K1);	//";K1="
        lcd_data(diodes[0].Cathode + 49);
        lcd_eep_string(K2);	//";K2="
        lcd_data(diodes[1].Cathode + 49);
        goto end;
      } 
      else if(diodes[0].Cathode == diodes[1].Cathode) {
        //Common Cathode
        lcd_eep_string(DualDiode);	//Doppeldiode
        lcd_eep_string(CC);	//"CC"
        Line2(); //2. Zeile
        lcd_eep_string(K);	//"K="
        lcd_data(diodes[0].Cathode + 49);
        lcd_eep_string(A1);		//";A1="
        lcd_data(diodes[0].Anode + 49);
        lcd_eep_string(A2);		//";A2="
        lcd_data(diodes[1].Anode + 49);
        goto end;
      } 
      else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
        //Antiparallel
        lcd_eep_string(TwoDiodes);	//2 Dioden
        Line2(); //2. Zeile
        lcd_eep_string(Antiparallel);	//Antiparallel
        goto end;
      }
    } 
    else if(NumOfDiodes == 3) {
      //Serienschaltung aus 2 Dioden; wird als 3 Dioden erkannt
      b = 3;
      c = 3;
      /* Überprüfen auf eine für eine Serienschaltung von 2 Dioden mögliche Konstellation
       				Dafür müssen 2 der Kathoden und 2 der Anoden übereinstimmen.
       				Das kommmt daher, dass die Dioden als 2 Einzeldioden und ZUSÄTZLICH als eine "große" Diode erkannt werden.
       			*/
      if((diodes[0].Anode == diodes[1].Anode) || (diodes[0].Anode == diodes[2].Anode)) b = diodes[0].Anode;
      if(diodes[1].Anode == diodes[2].Anode) b = diodes[1].Anode;

      if((diodes[0].Cathode == diodes[1].Cathode) || (diodes[0].Cathode == diodes[2].Cathode)) c = diodes[0].Cathode;
      if(diodes[1].Cathode == diodes[2].Cathode) c = diodes[1].Cathode;
      if((b<3) && (c<3)) {
        lcd_eep_string(TwoDiodes);//2 Dioden
        Line2(); //2. Zeile
        lcd_eep_string(InSeries); //"in Serie A="
        lcd_data(b + 49);
        lcd_eep_string(NextK);
        lcd_data(c + 49);
        goto end;
      }
    }
  } 
  else if (PartFound == PART_TRANSISTOR) {
    if(PartReady == 0) {	//Wenn 2. Prüfung nie gemacht, z.B. bei Transistor mit Schutzdiode
      hfe[1] = hfe[0];
      uBE[1] = uBE[0];
    }
    if((hfe[0]>hfe[1])) {	//Wenn der Verstärkungsfaktor beim ersten Test höher war: C und E vertauschen!
      hfe[1] = hfe[0];
      uBE[1] = uBE[0];
      tmp = c;
      c = e;
      e = tmp;
    }

    if(PartMode == PART_MODE_NPN) {
      lcd_eep_string(NPN);
    } 
    else {
      lcd_eep_string(PNP);
    }
    lcd_eep_string(bstr);	//B=
    lcd_data(b + 49);
    lcd_eep_string(cstr);	//;C=
    lcd_data(c + 49);
    lcd_eep_string(estr);	//;E=
    lcd_data(e + 49);
    Line2(); //2. Zeile
    //Verstärkungsfaktor berechnen
    //hFE = Emitterstrom / Basisstrom
    lhfe = hfe[1];

    lhfe *= (((unsigned long)rhval * 100) / (unsigned long)rlval);	//Verhältnis von High- zu Low-Widerstand
    if(uBE[1]<11) uBE[1] = 11;
    lhfe /= uBE[1];
    hfe[1] = (unsigned int) lhfe;
    lcd_eep_string(hfestr);	//"hFE="
    lcd_string(utoa(hfe[1], outval, 10));
    SetCursor(2,7);			//Cursor auf Zeile 2, Zeichen 7
    if(NumOfDiodes > 2) {	//Transistor mit Schutzdiode
      lcd_data(LCD_CHAR_DIODE);	//Diode anzeigen
    } 
    else {
      lcd_data(' ');
    }
    for(c=0;c<NumOfDiodes;c++) {
      if(((diodes[c].Cathode == e) && (diodes[c].Anode == b) && (PartMode == PART_MODE_NPN)) || ((diodes[c].Anode == e) && (diodes[c].Cathode == b) && (PartMode == PART_MODE_PNP))) {
        lcd_eep_string(Uf);	//"Uf="
        lcd_string(itoa(diodes[c].Voltage, outval, 10));
        lcd_data('m');
        goto end;
      }
    }
    goto end;
  } 
  else if (PartFound == PART_FET) {	//JFET oder MOSFET
    if(PartMode&1) {	//N-Kanal
      lcd_data('N');
    } 
    else {
      lcd_data('P');	//P-Kanal
    }
    if((PartMode==PART_MODE_N_D_MOS) || (PartMode==PART_MODE_P_D_MOS)) {
      lcd_eep_string(dmode);	//"-D"
      lcd_eep_string(mosfet);	//"-MOS"
    } 
    else {
      if((PartMode==PART_MODE_N_JFET) || (PartMode==PART_MODE_P_JFET)) {
        lcd_eep_string(jfet);	//"-JFET"
      } 
      else {
        lcd_eep_string(emode);	//"-E"
        lcd_eep_string(mosfet);	//"-MOS"
      }
    }
    if(PartMode < 3) {	//Anreicherungs-MOSFET
      lcd_eep_string(GateCap);	//" C="
      ReadCapacity(b,e);	//Messung
      hfe[0] = (unsigned int)cv;
      if(hfe[0]>2) hfe[0] -= 3;
      utoa(hfe[0], outval2, 10);

      tmpval = strlen(outval2);
      tmpval2 = tmpval;
      if(tmpval>4) tmpval = 4;	//bei Kapazität >100nF letze Nachkommastelle nicht mehr angeben (passt sonst nicht auf das LCD)
      lcd_show_format_cap(outval2, tmpval, tmpval2);
      lcd_data('n');
    }
    Line2(); //2. Zeile
    lcd_eep_string(gds);	//"GDS="
    lcd_data(b + 49);
    lcd_data(c + 49);
    lcd_data(e + 49);
    if((NumOfDiodes > 0) && (PartMode < 3)) {	//MOSFET mit Schutzdiode; gibt es nur bei Anreicherungs-FETs
      lcd_data(LCD_CHAR_DIODE);	//Diode anzeigen
    } 
    else {
      lcd_data(' ');	//Leerzeichen
    }
    if(PartMode < 3) {	//Anreicherungs-MOSFET
      gthvoltage=(gthvoltage/8);
      lcd_eep_string(vt);
      lcd_string(utoa(gthvoltage, outval, 10));	//Gate-Schwellspannung, wurde zuvor ermittelt
      lcd_data('m');
    }
    goto end;
  } 
  else if (PartFound == PART_THYRISTOR) {
    lcd_eep_string(Thyristor);	//"Thyristor"
    Line2(); //2. Zeile
    lcd_eep_string(GAK);	//"GAK="
    lcd_data(b + 49);
    lcd_data(c + 49);
    lcd_data(e + 49);
    goto end;
  } 
  else if (PartFound == PART_TRIAC) {
    lcd_eep_string(Triac);	//"Triac"
    Line2(); //2. Zeile
    lcd_eep_string(Gate);
    lcd_data(b + 49);
    lcd_eep_string(A1);		//";A1="
    lcd_data(e + 49);
    lcd_eep_string(A2);		//";A2="
    lcd_data(c + 49);
    goto end;
  } 
  else if(PartFound == PART_RESISTOR) {
    lcd_eep_string(Resistor); //"Widerstand: "
    lcd_data(ra + 49);	//Pin-Angaben
    lcd_data('-');
    lcd_data(rb + 49);
    Line2(); //2. Zeile
    if(rv[0]>512) {		//Überprüfen, wie weit die an den Testwiderständen anliegenden Spannungen von 512 abweichen
      hfe[0] = (rv[0] - 512);
    } 
    else {
      hfe[0] = (512 - rv[0]);
    }
    if(rv[1]>512) {
      hfe[1] = (rv[1] - 512);
    } 
    else {
      hfe[1] = (512 - rv[1]);
    }
    if(hfe[0] > hfe[1])  {
      radcmax[0] = radcmax[1];
      rv[0] = rv[1];	//Ergebnis verwenden, welches näher an 512 liegt (bessere Genauigkeit)
      rv[1] = rhval;	//470k-Testwiderstand	
    } 
    else {
      rv[1] = rlval;	//680R-Testwiderstand
    }
    if(rv[0]==0) rv[0] = 1;
    lhfe = (unsigned long)((unsigned long)((unsigned long)rv[1] * (unsigned long)rv[0]) / (unsigned long)((unsigned long)radcmax[0] - (unsigned long)rv[0]));	//Widerstand berechnen
    ultoa(lhfe,outval,10);

    if(rv[1]==rhval) {	//470k-Widerstand?
      ra = strlen(outval);	//Nötig, um Komma anzuzeigen
      for(rb=0;rb<ra;rb++) {
        lcd_data(outval[rb]);
        if(rb==(ra-2)) lcd_data('.');	//Komma
      }
      lcd_data ('k'); //Kilo-Ohm, falls 470k-Widerstand verwendet
    } 
    else {
      lcd_string(outval);
    }
    lcd_data(LCD_CHAR_OMEGA);	//Omega für Ohm 
    goto end;

  } 
  else if(PartFound == PART_CAPACITOR) {	//Kapazitätsmessung auch nur auf Mega8 verfügbar
    lcd_eep_string(Capacitor);
    lcd_data(ca + 49);	//Pin-Angaben
    lcd_data('-');
    lcd_data(cb + 49);
    Line2(); //2. Zeile
    tmpval2 = 'n';
    if(cv > 99999) {	//ab 1µF
      cv /= 1000;
      tmpval2 = LCD_CHAR_U;
    }
    ultoa(cv, outval, 10);
    tmpval = strlen(outval);
    lcd_show_format_cap(outval, tmpval, tmpval);
    lcd_data(tmpval2);
    lcd_data('F');
    goto end;
  }
  if(NumOfDiodes == 0) {
    //Keine Dioden gefunden
    lcd_eep_string(TestFailed1); //"Kein,unbek. oder"
    Line2(); //2. Zeile
    lcd_eep_string(TestFailed2); //"defektes "
    lcd_eep_string(Bauteil);
  } 
  else {
    lcd_eep_string(Bauteil);
    lcd_eep_string(Unknown); //" unbek."
    Line2(); //2. Zeile
    lcd_eep_string(OrBroken); //"oder defekt"
    lcd_data(NumOfDiodes + 48);
    lcd_data(LCD_CHAR_DIODE);
  }
end:
  while(!(ON_PIN_REG & (1<<RST_PIN)));		//warten ,bis Taster losgelassen
  _delay_ms(200);
  for(hfe[0] = 0;hfe[0]<10000;hfe[0]++) {
    if(!(ON_PIN_REG & (1<<RST_PIN))) {
      /*Wenn der Taster wieder gedrückt wurde...
       			wieder zum Anfang springen und neuen Test durchführen
       			*/
      goto start;
    }
    wdt_reset();
    _delay_ms(1);
  }
  /* Abschalten */
  ON_PORT &= ~(1<<ON_PIN);
  _delay_ms(500);
  wdt_disable();	//Watchdog aus
  //Endlosschleife
  while(1) {
    if(!(ON_PIN_REG & (1<<RST_PIN))) {	
      /* wird nur erreicht,
       		 	wenn die automatische Abschaltung nicht eingebaut wurde */
      goto start;
    }
  }
  return 0;
}

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin) {
  /*
	Funktion zum Testen der Eigenschaften des Bauteils bei der angegebenen Pin-Belegung
   	Parameter:
   	HighPin: Pin, der anfangs auf positives Potenzial gelegt wird
   	LowPin: Pin, der anfangs auf negatives Potenzial gelegt wird
   	TristatePin: Pin, der anfangs offen gelassen wird
   
   	Im Testverlauf wird TristatePin natürlich auch positiv oder negativ geschaltet.
   	*/
  unsigned int adcv[6];
  uint8_t tmpval, tmpval2;
  /*
		HighPin wird fest auf Vcc gelegt
   		LowPin wird über R_L auf GND gelegt
   		TristatePin wird hochohmig geschaltet, dafür ist keine Aktion nötig
   	*/
  wdt_reset();
  //Pins setzen
  tmpval = (LowPin * 2);			//nötig wegen der Anordnung der Widerstände
  R_DDR = (1<<tmpval) | R_DDR_DEF;			//Low-Pin auf Ausgang und über R_L auf Masse
  R_PORT = R_PORT_DEF;
  ADC_DDR = (1<<HighPin);			//High-Pin auf Ausgang
  ADC_PORT = (1<<HighPin);		//High-Pin fest auf Vcc
  _delay_ms(5);
  //Bei manchen MOSFETs muss das Gate (TristatePin) zuerst entladen werden
  //N-Kanal:
  DischargePin(TristatePin,0);
  //Spannung am Low-Pin ermitteln
  adcv[0] = ReadADC(LowPin);
  if(adcv[0] < 200) goto next;	//Sperrt das Bauteil jetzt?
  //sonst: Entladen für P-Kanal (Gate auf Plus)
  DischargePin(TristatePin,1);
  //Spannung am Low-Pin ermitteln
  adcv[0] = ReadADC(LowPin);

next:

  if(adcv[0] > 19) {//Bauteil leitet ohne Steuerstrom etwas
    //Test auf N-JFET oder selbstleitenden N-MOSFET
    R_DDR |= (2<<(TristatePin*2));	//Tristate-Pin (vermutetes Gate) über R_H auf Masse
    _delay_ms(20);
    adcv[1] = ReadADC(LowPin);		//Spannung am vermuteten Source messen
    R_PORT |= (2<<(TristatePin*2));	//Tristate-Pin (vermutetes Gate) über R_H auf Plus
    _delay_ms(20);
    adcv[2] = ReadADC(LowPin);		//Spannung am vermuteten Source erneut messen
    //Wenn es sich um einen selbstleitenden MOSFET oder JFET handelt, müsste adcv[1] > adcv[0] sein
    if(adcv[2]>(adcv[1]+100)) {
      //Spannung am Gate messen, zur Unterscheidung zwischen MOSFET und JFET
      ADC_PORT = 0;
      ADC_DDR = (1<<LowPin);	//Low-Pin fest auf Masse
      tmpval = (HighPin * 2);		//nötig wegen der Anordnung der Widerstände
      R_DDR |= (1<<tmpval);			//High-Pin auf Ausgang
      R_PORT |= (1<<tmpval);			//High-Pin über R_L auf Vcc
      _delay_ms(20);
      adcv[2] = ReadADC(TristatePin);		//Spannung am vermuteten Gate messen
      if(adcv[2]>800) {	//MOSFET
        PartFound = PART_FET;			//N-Kanal-MOSFET
        PartMode = PART_MODE_N_D_MOS;	//Verarmungs-MOSFET
      } 
      else {	//JFET (pn-Übergang zwischen G und S leitet)
        PartFound = PART_FET;			//N-Kanal-JFET
        PartMode = PART_MODE_N_JFET;
      }
      b = TristatePin;
      c = HighPin;
      e = LowPin;
    }
    ADC_PORT = 0;

    //Test auf P-JFET oder selbstleitenden P-MOSFET
    ADC_DDR = (1<<LowPin);	//Low-Pin (vermuteter Drain) fest auf Masse, Tristate-Pin (vermutetes Gate) ist noch über R_H auf Plus
    tmpval = (HighPin * 2);			//nötig wegen der Anordnung der Widerstände
    R_DDR |= (1<<tmpval);			//High-Pin auf Ausgang
    R_PORT |= (1<<tmpval);			//High-Pin über R_L auf Vcc
    _delay_ms(20);
    adcv[1] = ReadADC(HighPin);		//Spannung am vermuteten Source messen
    R_PORT = (1<<tmpval) | R_PORT_DEF;			//Tristate-Pin (vermutetes Gate) über R_H auf Masse
    _delay_ms(20);
    adcv[2] = ReadADC(HighPin);		//Spannung am vermuteten Source erneut messen
    //Wenn es sich um einen selbstleitenden P-MOSFET oder P-JFET handelt, müsste adcv[0] > adcv[1] sein
    if(adcv[1]>(adcv[2]+100)) {
      //Spannung am Gate messen, zur Unterscheidung zwischen MOSFET und JFET
      ADC_PORT = (1<<HighPin);	//High-Pin fest auf Plus
      ADC_DDR = (1<<HighPin);		//High-Pin auf Ausgang
      _delay_ms(20);
      adcv[2] = ReadADC(TristatePin);		//Spannung am vermuteten Gate messen
      if(adcv[2]<200) {	//MOSFET
        PartFound = PART_FET;			//P-Kanal-MOSFET
        PartMode = PART_MODE_P_D_MOS;	//Verarmungs-MOSFET
      } 
      else {	//JFET (pn-Übergang zwischen G und S leitet)
        PartFound = PART_FET;			//P-Kanal-JFET
        PartMode = PART_MODE_P_JFET;
      }
      b = TristatePin;
      c = LowPin;
      e = HighPin;
    }
  }
  //Pins erneut setzen
  tmpval = (LowPin * 2);			//nötig wegen der Anordnung der Widerstände
  R_DDR = (1<<tmpval) | R_DDR_DEF;			//Low-Pin auf Ausgang und über R_L auf Masse
  R_PORT = R_PORT_DEF;
  ADC_DDR = (1<<HighPin);			//High-Pin auf Ausgang
  ADC_PORT = (1<<HighPin);		//High-Pin fest auf Vcc
  _delay_ms(5);

  if(adcv[0] < 200) {	//Wenn das Bauteil keinen Durchgang zwischen HighPin und LowPin hat
    //Test auf pnp
    tmpval2 = (TristatePin * 2);		//nötig wegen der Anordnung der Widerstände
    R_DDR |= (1<<tmpval2);			//Tristate-Pin über R_L auf Masse, zum Test auf pnp
    _delay_ms(2);
    adcv[1] = ReadADC(LowPin);		//Spannung messen
    if(adcv[1] > 700) {
      //Bauteil leitet => pnp-Transistor o.ä.
      //Verstärkungsfaktor in beide Richtungen messen
      R_DDR = (1<<tmpval) | R_DDR_DEF;		//Tristate-Pin (Basis) hochohmig
      tmpval2++;
      R_DDR |= (1<<tmpval2);		//Tristate-Pin (Basis) über R_H auf Masse

      _delay_ms(10);
      adcv[1] = ReadADC(LowPin);		//Spannung am Low-Pin (vermuteter Kollektor) messen
      adcv[2] = ReadADC(TristatePin);	//Basisspannung messen
      //Prüfen, ob Test schon mal gelaufen
      if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) PartReady = 1;
      hfe[PartReady] = adcv[1];
      uBE[PartReady] = adcv[2];

      if(PartFound != PART_THYRISTOR) {
        if(adcv[2] > 200) {
          PartFound = PART_TRANSISTOR;	//PNP-Transistor gefunden (Basis wird "nach oben" gezogen)
          PartMode = PART_MODE_PNP;
        } 
        else {
          if(adcv[0] < 20) {	//Durchlassspannung im gesperrten Zustand gering genug? (sonst werden D-Mode-FETs fälschlicherweise als E-Mode erkannt)
            PartFound = PART_FET;			//P-Kanal-MOSFET gefunden (Basis/Gate wird NICHT "nach oben" gezogen)
            PartMode = PART_MODE_P_E_MOS;
            //Messung der Gate-Schwellspannung
            tmpval = (1<<LowPin);
            tmpval2 = R_DDR;
            ADMUX = TristatePin | (1<<REFS0);
            gthvoltage = 0;
            for(b=0;b<13;b++) {
              wdt_reset();
              DischargePin(TristatePin,1);
              while (!(ADC_PIN&tmpval));  // Warten, bis der MOSFET schaltet und Drain auf high geht
              R_DDR = R_DDR_DEF;
              ADCSRA |= (1<<ADSC);
              while (ADCSRA&(1<<ADSC));
              gthvoltage += (1023 - ADCW);
              R_DDR = tmpval2 | R_DDR_DEF;
            }
            gthvoltage *= 3;	//Umrechnung in mV, zusammen mit der Division durch 8 (bei der LCD-Anzeige)
          }
        }
        b = TristatePin;
        c = LowPin;
        e = HighPin;
      }
    }

    //Tristate (vermutete Basis) auf Plus, zum Test auf npn
    ADC_PORT = 0;					//Low-Pin fest auf Masse
    tmpval = (TristatePin * 2);		//nötig wegen der Anordnung der Widerstände
    tmpval2 = (HighPin * 2);		//nötig wegen der Anordnung der Widerstände
    R_DDR = (1<<tmpval) | (1<<tmpval2) | R_DDR_DEF;			//High-Pin und Tristate-Pin auf Ausgang
    R_PORT = (1<<tmpval) | (1<<tmpval2) | R_PORT_DEF;		//High-Pin und Tristate-Pin über R_L auf Vcc
    ADC_DDR = (1<<LowPin);			//Low-Pin auf Ausgang
    _delay_ms(10);
    adcv[1] = ReadADC(HighPin);		//Spannung am High-Pin messen
    if(adcv[1] < 500) {
      if(PartReady==1) goto testend;
      //Bauteil leitet => npn-Transistor o.ä.

      //Test auf Thyristor:
      //Gate entladen

      R_PORT = (1<<tmpval2) | R_PORT_DEF;			//Tristate-Pin (Gate) über R_L auf Masse
      _delay_ms(10);
      R_DDR = (1<<tmpval2) | R_DDR_DEF;			//Tristate-Pin (Gate) hochohmig
      //Test auf Thyristor
      _delay_ms(5);
      adcv[3] = ReadADC(HighPin);		//Spannung am High-Pin (vermutete Anode) erneut messen

        R_PORT = R_PORT_DEF;						//High-Pin (vermutete Anode) auf Masse
      _delay_ms(5);
      R_PORT = (1<<tmpval2) | R_PORT_DEF;			//High-Pin (vermutete Anode) wieder auf Plus
      _delay_ms(5);
      adcv[2] = ReadADC(HighPin);		//Spannung am High-Pin (vermutete Anode) erneut messen
      if((adcv[3] < 500) && (adcv[2] > 900)) {	//Nach Abschalten des Haltestroms muss der Thyristor sperren
        //war vor Abschaltung des Triggerstroms geschaltet und ist immer noch geschaltet obwohl Gate aus => Thyristor
        PartFound = PART_THYRISTOR;
        //Test auf Triac
        R_DDR = R_DDR_DEF;
        R_PORT = R_PORT_DEF;
        ADC_PORT = (1<<LowPin);	//Low-Pin fest auf Plus
        _delay_ms(5);
        R_DDR = (1<<tmpval2) | R_DDR_DEF;	//HighPin über R_L auf Masse
        _delay_ms(5);
        if(ReadADC(HighPin) > 50) goto savenresult;	//Spannung am High-Pin (vermuteter A2) messen; falls zu hoch: Bauteil leitet jetzt => kein Triac
        R_DDR |= (1<<tmpval);	//Gate auch über R_L auf Masse => Triac müsste zünden
        _delay_ms(5);
        if(ReadADC(TristatePin) < 200) goto savenresult; //Spannung am Tristate-Pin (vermutetes Gate) messen; Abbruch falls Spannung zu gering
        if(ReadADC(HighPin) < 150) goto savenresult; //Bauteil leitet jetzt nicht => kein Triac => Abbruch
        R_DDR = (1<<tmpval2) | R_DDR_DEF;	//TristatePin (Gate) wieder hochohmig
        _delay_ms(5);
        if(ReadADC(HighPin) < 150) goto savenresult; //Bauteil leitet nach Abschalten des Gatestroms nicht mehr=> kein Triac => Abbruch
        R_PORT = (1<<tmpval2) | R_PORT_DEF;	//HighPin über R_L auf Plus => Haltestrom aus
        _delay_ms(5);
        R_PORT = R_PORT_DEF;				//HighPin wieder über R_L auf Masse; Triac müsste jetzt sperren
        _delay_ms(5);
        if(ReadADC(HighPin) > 50) goto savenresult;	//Spannung am High-Pin (vermuteter A2) messen; falls zu hoch: Bauteil leitet jetzt => kein Triac
        PartFound = PART_TRIAC;
        PartReady = 1;
        goto savenresult;
      }
      //Test auf Transistor oder MOSFET
      tmpval++;
      R_DDR |= (1<<tmpval);		//Tristate-Pin (Basis) auf Ausgang
      R_PORT |= (1<<tmpval);		//Tristate-Pin (Basis) über R_H auf Plus
      _delay_ms(50);
      adcv[1] = ReadADC(HighPin);		//Spannung am High-Pin (vermuteter Kollektor) messen
      adcv[2] = ReadADC(TristatePin);	//Basisspannung messen

        if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) PartReady = 1;	//prüfen, ob Test schon mal gelaufen
      hfe[PartReady] = 1023 - adcv[1];
      uBE[PartReady] = 1023 - adcv[2];
      if(adcv[2] < 500) {
        PartFound = PART_TRANSISTOR;	//NPN-Transistor gefunden (Basis wird "nach unten" gezogen)
        PartMode = PART_MODE_NPN;
      } 
      else {
        if(adcv[0] < 20) {	//Durchlassspannung im gesperrten Zustand gering genug? (sonst werden D-Mode-FETs fälschlicherweise als E-Mode erkannt)
          PartFound = PART_FET;			//N-Kanal-MOSFET gefunden (Basis/Gate wird NICHT "nach unten" gezogen)
          PartMode = PART_MODE_N_E_MOS;
          //Gate-Schwellspannung messen
          tmpval2 = R_DDR;
          tmpval=(1<<HighPin);
          ADMUX = TristatePin | (1<<REFS0);
          gthvoltage = 0;
          for(b=0;b<13;b++) {
            wdt_reset();
            DischargePin(TristatePin,0);
            while ((ADC_PIN&tmpval));  // Warten, bis der MOSFET schaltet und Drain auf low geht
            R_DDR = R_DDR_DEF;
            R_PORT = R_PORT_DEF;
            ADCSRA |= (1<<ADSC);
            while (ADCSRA&(1<<ADSC));
            gthvoltage += ADCW;
            R_DDR = tmpval2 | R_DDR_DEF;
            R_PORT = tmpval2 | R_PORT_DEF;
          }
          gthvoltage *= 3;	//Umrechnung in mV, zusammen mit der Division durch 8 (bei der LCD-Anzeige)
        }
      }
savenresult:
      b = TristatePin;
      c = HighPin;
      e = LowPin;
    }
    ADC_DDR = 0;
    ADC_PORT = 0;
    //Fertig
  } 
  else {	//Durchgang
    //Test auf Diode
    tmpval2 = (2<<(2*HighPin));	//R_H
    tmpval = (1<<(2*HighPin));	//R_L
    ADC_PORT = 0;
    ADC_DDR = (1<<LowPin);	//Low-Pin fest auf Masse, High-Pin ist noch über R_L auf Vcc
    DischargePin(TristatePin,1);	//Entladen für P-Kanal-MOSFET
    _delay_ms(5);
    adcv[0] = ReadADC(HighPin) - ReadADC(LowPin);
    R_DDR = tmpval2 | R_DDR_DEF;	//High-Pin über R_H auf Plus
    R_PORT = tmpval2 | R_PORT_DEF;
    _delay_ms(5);
    adcv[2] = ReadADC(HighPin) - ReadADC(LowPin);
    R_DDR = tmpval | R_DDR_DEF;	//High-Pin über R_L auf Plus
    R_PORT = tmpval | R_PORT_DEF;
    DischargePin(TristatePin,0);	//Entladen für N-Kanal-MOSFET
    _delay_ms(5);
    adcv[1] = ReadADC(HighPin) - ReadADC(LowPin);
    R_DDR = tmpval2 | R_DDR_DEF;	//High-Pin über R_H  auf Plus
    R_PORT = tmpval2 | R_PORT_DEF;
    _delay_ms(5);
    adcv[3] = ReadADC(HighPin) - ReadADC(LowPin);
    /*Ohne das Entladen kann es zu Falscherkennungen kommen, da das Gate eines MOSFETs noch geladen sein kann.
     			Die zusätzliche Messung mit dem "großen" Widerstand R_H wird durchgeführt, um antiparallele Dioden von
     			Widerständen unterscheiden zu können.
     			Eine Diode hat eine vom Durchlassstrom relativ unabhängige Durchlassspg.
     			Bei einem Widerstand ändert sich der Spannungsabfall stark (linear) mit dem Strom.
     		*/
    if(adcv[0] > adcv[1]) {
      adcv[1] = adcv[0];	//der höhere Wert gewinnt
      adcv[3] = adcv[2];
    }

    if((adcv[1] > 30) && (adcv[1] < 950)) { //Spannung liegt über 0,15V und unter 4,64V => Ok
      if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) PartFound = PART_DIODE;	//Diode nur angeben, wenn noch kein anderes Bauteil gefunden wurde. Sonst gäbe es Probleme bei Transistoren mit Schutzdiode
      diodes[NumOfDiodes].Anode = HighPin;
      diodes[NumOfDiodes].Cathode = LowPin;
      diodes[NumOfDiodes].Voltage = (adcv[1]*54/11);	// ca. mit 4,9 multiplizieren, um aus dem ADC-Wert die Spannung in Millivolt zu erhalten
      NumOfDiodes++;
      for(uint8_t i=0;i<NumOfDiodes;i++) {
        if((diodes[i].Anode == LowPin) && (diodes[i].Cathode == HighPin)) {	//zwei antiparallele Dioden: Defekt oder Duo-LED
          if((adcv[3]*64) < (adcv[1] / 5)) {	//Durchlassspannung fällt bei geringerem Teststrom stark ab => Defekt
            if(i<NumOfDiodes) {
              for(uint8_t j=i;j<(NumOfDiodes-1);j++) {
                diodes[j].Anode = diodes[j+1].Anode;
                diodes[j].Cathode = diodes[j+1].Cathode;
                diodes[j].Voltage = diodes[j+1].Voltage;
              }
            }
            NumOfDiodes -= 2;
          }
        }
      }
    }
  }

  //Test auf Widerstand
  tmpval2 = (2<<(2*HighPin));	//R_H
  tmpval = (1<<(2*HighPin));	//R_L
  ADC_PORT = 0;
  ADC_DDR = (1<<LowPin);	//Low-Pin fest auf Masse
  R_DDR = tmpval | R_DDR_DEF;	//High-Pin über R_L auf Plus
  R_PORT = tmpval | R_PORT_DEF;
  adcv[2] = ReadADC(LowPin);
  adcv[0] = ReadADC(HighPin) - adcv[2];
  R_DDR = tmpval2 | R_DDR_DEF;	//High-Pin über R_H auf Plus
  R_PORT = tmpval2 | R_PORT_DEF;
  adcv[3] = ReadADC(LowPin);
  adcv[1] = ReadADC(HighPin) - adcv[3];

  //Messung der Spannungsdifferenz zwischen dem Pluspol von R_L und R_H und Vcc
  tmpval2 = (2<<(2*LowPin));	//R_H
  tmpval = (1<<(2*LowPin));	//R_L
  ADC_DDR = (1<<HighPin);		//High-Pin auf Ausgang
  ADC_PORT = (1<<HighPin);	//High-Pin fest auf Plus
  R_PORT = R_PORT_DEF;
  R_DDR = tmpval | R_DDR_DEF;				//Low-Pin über R_L auf Masse
  adcv[2] += (1023 - ReadADC(HighPin));
  R_DDR = tmpval2 | R_DDR_DEF;				//Low-Pin über R_H auf Masse
  adcv[3] += (1023 - ReadADC(HighPin));

  if(((adcv[0] - adcv[2]) < 900) && ((adcv[1] - adcv[3]) > 20)) goto testend; 	//Spannung fällt bei geringem Teststrom nicht weit genug ab
  if(((adcv[1] * 32) / 31) < adcv[0]) {	//Abfallende Spannung fällt bei geringerem Teststrom stark ab und es besteht kein "Beinahe-Kurzschluss" => Widerstand
    if((PartFound == PART_DIODE) || (PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
      if((tmpPartFound == PART_RESISTOR) && (ra == LowPin) && (rb == HighPin)) {
        /* Das Bauteil wurde schon einmal mit umgekehrter Polarität getestet.
         					Jetzt beide Ergebnisse miteinander vergleichen. Wenn sie recht ähnlich sind,
         					handelt es sich (höchstwahrscheinlich) um einen Widerstand. */
        if(!((((adcv[0] + 100) * 6) >= ((rv[0] + 100) * 5)) && (((rv[0] + 100) * 6) >= ((adcv[0] + 100) * 5)) && (((adcv[1] + 100) * 6) >= ((rv[1] + 100) * 5)) && (((rv[1] + 100) * 6) >= ((adcv[1] + 100) * 5)))) {
          //min. 20% Abweichung => kein Widerstand
          tmpPartFound = PART_NONE;
          goto testend;
        }
        PartFound = PART_RESISTOR;
      }
      rv[0] = adcv[0];
      rv[1] = adcv[1];

      radcmax[0] = 1023 - adcv[2];	//Spannung am Low-Pin ist nicht ganz Null, sondern rund 0,1V (wird aber gemessen). Der dadurch entstehende Fehler wird hier kompenis
      rt
        radcmax[1] = 1023 - adcv[3];
      ra = HighPin;
      rb = LowPin;
      tmpPartFound = PART_RESISTOR;
    }
  }


testend:
  ADC_DDR = 0;
  ADC_PORT = 0;
  R_DDR = R_DDR_DEF;
  R_PORT = R_PORT_DEF;
}

void ReadCapacity(uint8_t HighPin, uint8_t LowPin) {
  //Test auf Kondensator (auch nur auf ATMega8 möglich)
  if(PartFound == PART_CAPACITOR) goto end;	//Schon einen Kondensator gefunden
  unsigned long gcval = 0;
  unsigned int tmpint = 0;
  uint8_t extcnt = 0;
  uint8_t tmpx = 0;

  tmpval2 = (2<<(2*HighPin));	//R_H
  tmpval = (1<<(2*HighPin));	//R_L
  ADC_PORT = 0;
  R_PORT = R_PORT_DEF;
  R_DDR = R_DDR_DEF;
  ADC_DDR = (1<<LowPin);	//Low-Pin fest auf Masse
  R_DDR = tmpval2 | R_DDR_DEF;		//HighPin über R_H auf Masse
  _delay_ms(5);
  adcv[0] = ReadADC(HighPin);
  DischargePin(HighPin,1);
  adcv[2] = ReadADC(HighPin);
  _delay_ms(5);
  adcv[1] = ReadADC(HighPin);
  wdt_reset();
  if((adcv[1] > (adcv[0] + 1)) || (adcv[2] > (adcv[0] + 1))) {	//Spannung ist gestiegen
    R_DDR = tmpval | R_DDR_DEF;			//High-Pin über R_L auf Masse
    while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
      wdt_reset();
      tmpint++;
      if(tmpint==0) {
        extcnt++;
        if(extcnt == 30) break; //Timeout für Entladung
      }
    }
    tmpint = 0;
    extcnt = 0;
    R_PORT = tmpval | R_PORT_DEF;			//High-Pin über R_L auf Plus
    _delay_ms(5);
    adcv[2] = ReadADC(HighPin);
    _delay_ms(80);
    adcv[3] = ReadADC(HighPin);
    if((adcv[3] < (adcv[2] + 3)) && (adcv[3] < 850)) goto end;	//Spannung ist nicht nennenswert gestiegen => Abbruch
    if((NumOfDiodes > 0) && (adcv[3] > 950) && (PartFound != PART_FET)) goto end; //höchstwahrscheinlich eine (oder mehrere) Diode(n) in Sperrrichtung, die sonst fälschlicherweise als Kondensator erkannt wird
    R_PORT = R_PORT_DEF;
    while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
      wdt_reset();
      tmpint++;
      if(tmpint==0) {
        extcnt++;
        if(extcnt == 30) break; //Timeout für Entladung
      }
    }
    tmpint = 0;
    extcnt = 0;
    ADC_DDR = 7;					//alle Pins auf Ausgang und aus Masse
    R_PORT = tmpval | R_PORT_DEF;  	   			// HighPin über R_L auf Plus
    tmpval=(1<<HighPin);
    _delay_ms(2);
    ADC_DDR = (1<<LowPin);          // Kondensator über R_L langsam laden
    while (!(ADC_PIN & tmpval)) {  // Warten, bis HighPin auf High geht; Schleife dauert 7 Zyklen
      wdt_reset();
      tmpint++;
      if(tmpint==0) {
        extcnt++;
        if(extcnt == 30) break; //Timeout für Ladung
      }
    }
    if((extcnt == 0) && (tmpint<256)) {	//Niedrige Kapazität
      ADC_DDR = (1<<LowPin);
      //mit R_H erneut messen
      R_PORT = R_PORT_DEF;
      tmpint = 0;
      extcnt = 0;
      while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
        wdt_reset();
        tmpint++;
        if(tmpint==0) {
          extcnt++;
          if(extcnt == 30) break; //Timeout für Entladung
        }
      }
      tmpint = 0;
      extcnt = 0;
      ADC_DDR = 7;					//alle Pins auf Ausgang
      ADC_PORT = 0;					//alle Pins fest auf Masse
      R_DDR = tmpval2 | R_DDR_DEF;        		// HighPin über R_H auf Ausgang
      R_PORT = tmpval2 | R_PORT_DEF;  	   			// HighPin über R_H auf Plus
      _delay_ms(2);
      if(PartFound == PART_FET) {
        ADC_DDR = (7 & ~tmpval);          // Kondensator über R_H langsam laden, dabei freien Pin (Drain) für Gate-Kapazitäts-Messung auf Masse
      } 
      else {
        ADC_DDR = (1<<LowPin);          // Kondensator über R_H langsam laden
      }
      while (!(ADC_PIN & tmpval)) {  // Warten, bis HighPin auf High geht; Schleife dauert 7 Zyklen
        wdt_reset();
        tmpint++;
        if(tmpint==0) {
          extcnt++;
          if(extcnt == 30) break; //Timeout für Kapazitätsmessung
        }
      }
      tmpx = 1;
    }
    if(tmpx) {
      gcval = eeprom_read_word(&H_CAPACITY_FACTOR);
      if((extcnt == 0) && (tmpint < 5)) goto end;	//Kapazität zu gering
      cv = 1;
    } 
    else {
      gcval = eeprom_read_word(&L_CAPACITY_FACTOR);
      cv = 1000;
    }

    gcval *= (unsigned long)(((unsigned long)extcnt * 65536) + (unsigned long)tmpint);	//Wert unrechnen und speichern
    gcval /= 100;
    cv *= gcval;

    PartFound = PART_CAPACITOR;	//Kondensator gefunden

    ca = HighPin;
    cb = LowPin;
    //Kondensator wieder entladen
    tmpint = 0;
    extcnt = 0;
    R_DDR = (1<<(2*HighPin)) | R_DDR_DEF;			//High-Pin über R_L auf Masse
    R_PORT = R_PORT_DEF;
    while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
      wdt_reset();
      tmpint++;
      if(tmpint==0) {
        extcnt++;
        if(extcnt == 30) break; //Timeout für Entladung
      }
    }
    ADC_DDR = 7;	//komplett entladen
    ADC_PORT = 7;
    _delay_ms(10);
    //Fertig
  }
end:
  ADC_DDR =  0;				
  ADC_PORT = 0;
  R_DDR = R_DDR_DEF;
  R_PORT = R_PORT_DEF; 
}


void lcd_show_format_cap(char outval[], uint8_t strlength, uint8_t CommaPos) {
  if(strlength < 3) {
    if(strlength==1) {
      lcd_string("0.");
      lcd_data('0');
      lcd_data(outval[0]);
    } 
    else {
      lcd_string("0.");
      lcd_data(outval[0]);
      lcd_data(outval[1]);
    }
  } 
  else {
    for(PartReady=0;PartReady<strlength;PartReady++) {
      if((PartReady + 2) == CommaPos) lcd_data('.');
      lcd_data(outval[PartReady]);
    }
  }
}



#endif


