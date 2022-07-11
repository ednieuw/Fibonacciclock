/* ===================================== 
 *  
 *  
 *  Use a Arduino Nano Every when Sketch uses 21864 bytes (71%) of program storage space.
 *  When you enter in the menu m500 and the brighness changes to 1 and LEDs go out then 
 *  the sketch is too large. SK6812 memory management in the Arduino interferes with the program.
 *  or other libraries used. 
 *  Use preferable FibonacciKlok_V007 when an Arduino Nano is used
 *  
 Title : Fibonacciklok
 Author: Ed Nieuwenhuys

 V001 Stripped version from Fibonacci_Colour_Clock_SK6812_WS2812_1284-328-V003
 V002 Optimized software. Working version for Arduino Nano
 V003 Development
 V004 Development 
 V005 In Fibonacciklok No 5&6
 V006 Derived from VierkantbuisklokV005. Rotary added, Keypad wirh 7 pins and for One-wire keypad. Menu changed
 V007 Release version in Fibonacci-klok No7
 V008 Optimized HC-12 receiver code from: https://forum.arduino.cc/index.php?topic=396450.0
 V009 Changed time input. Only 6 digits allowed in format hhmmss. T124500 for time entry in menu added
 V010 Added coding for the stick clock that uses no PCB
 V011 Changed palettes P0-P9 and colours in it. Corrected BT pinnumbers. corrected initialisation: int Previous_LDR_read = 51 --> 512;
 V012 Palette stored in EEPROM. Darkgray in palette choices added
 V013 Removed Secpulse and simplified Brightness calculation. Cleaned coding
 V014 12-edge clock, Added MAX7219, Added time MAX7219 On/Off
 V015 Normal, Extreme, Ultimate clock in Fibonacci display added. Time display LEDs On/Off added
 V016 Eyecare Fibonacciklok
 V017 sizeof(menu) / sizeof(menu[0]). Added PrintLine sub routine. RTC.->RTCklok.
 V018 Store EEPROM in struct mem
 V019 Added 3x1 membrane button instead of rotary. KEYPAD3x1
 V020 SConstrainInt added. Optimizing code.Sketch uses 19686 bytes. Global variables use 1457 bytes
 V021 V022 Failure. Memory and serial input trouble with ADAfruit KEYPAD.         
 V023 Used for two Fibonacci Stick clock 50cm. One for Addy & Hans.   Working version for Arduino Nano
 V024 Added One wire 3x1 membrane button ONEWIREKEYPAD3x1 
 V025 Store 24 bytes of the last 24 hour LDR readings to DS3231 NVRAM.  Corrected rotary encoder bug in processing key press looping forever
 V026 In Fibonacci clocks with 13 WS2812 LEDs
 V027 Added two SK6812 libraries FAB LED and Arduino-SK6812 instead of Ada Neopixel that corrupts memory in a Arduino Nano with this program size
      Removed  compiler warnings when set to 'all'in preferences
 V028 Changed constrain(mem,0,250) -->= min(mem,250)
 V029 Save to NVRAM after Reset().   Tekstprintln() changed coding
 V030 Test
 V031 Added HT16K33tijd 4-digit display
 V032 Added Edsoft_SK6812 library. Removed FABLED & SK6812 Library
 V033 Corrected error with ONEWIREKEYPAD3x4. Software in fibinacci F013 12-segmentklok met gat
 V034 For 12Segment clock without hole and 36 LEDs for AEM
 V035 issues with time settings (loosing time to 00:00:00) and program crashes 
 V036 Derived from V033. BluetoothString was not empty and set the time to 00:00:00; Made input control in ReworkString() stronger
 V037 Added MAXBRIGHTNESS and Mem.UpperBrightness to regulate maximum luminosity. Changed MAXBRIGHTNESS to SLOPEBRIGHTNESS
      Mem.LightReducer is the slope. 
      Added (Bri.../4) after (byte) in SetBrightnessLeds((byte) (BrightnessCalcFromLDR/4)); 
 V038 Removed from EverySecondcheck() //   if(Isecond % 30 == 0)  DimLeds(true);    // Test LED intensity control + seconds tick print every 30 seconds   
                                      //   else
 V039 Added Mem.Checksum in EEPROM storage
 
 **** Before you compile compile    
 Choose the desired modules and settings with the #defines. A double slash // before the #define turns compilation fot this definition off 
 Time is presented with colour LEDs that are grouped in the {1,1,2,3,5) compartments.
 Change the number of LEDs used in the fibonacci at line +/- 65: const byte NUM_LEDS  = 12; 
 Also update the LEDs used per compartment in  the function setPixel()  
*/

//     =====================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
// ------------------>   Define one type of LED strip 
//#define LED2812                // Use  RGB LED strip WS2812
#define LED6812                  // Use RGBW LED strip SK6812
// ------------------>   Define which library
#define NEOPIXEL               // Adafruit Neopixel for WS2812 or SK6812 LEDs
//#define LIBSK6812                // SK6812 library. Only with SK6812 LEDs (saves 614 bytes with NEOPIXEL)
const int NUM_LEDS = 14;         // How many leds in fibonacci clock?
                                 // If Not 14 LEDs --->  Update also LED positions in  setPixel() (at +/- line 1180) TOO!  

// ------------------>  Define one type of clock
//#define STICKCLOCK               // If wires of clock and LEDs are directly connected to Arduino Nano
#define PCBCLOCK                 // If Fibonacci clock is made on PCB. PCB uses other connections, see PIN Assigments line 85

// ------------------>  Define if a module is present. If not desired uncomment it by adding // before a #define 
//#define ROTARYMOD                // Use rotary encoder
#define MOD_DS3231               // DS3231 RTC module installed
//#define BTMOD_PIN6_PIN7          // Bluetooth module attached to pin 6 & pin 7. If connected to PIN 0 & 1 disconnect it when uploading

//#define KEYPAD3x4                // Use a 3x4 keypad with 7 wires
//#define KEYPAD3x1                // Use a 3x1 keypad with 4 wires   
//#define ONEWIREKEYPAD3x1         // Use a 3x1 keypad with one wire   
#define ONEWIREKEYPAD3x4         // Use a 3x4 keypad with one wire
//#define MAX7219                  // MAX7219 display
//#define HT16K33tijd              // 4-digit HT16K33 time display installed https://www.adafruit.com/product/879 Adafruit GFX library 

char VERSION[] = "FibonacciKlok_V039";         // Version of this coding

//--------------------------------------------
// ARDUINO Includes defines and initialisations
// All libraries can be installed from the Arduino IDE
// Tools --> Manage libraries
//--------------------------------------------
                     #ifdef ROTARYMOD
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>             // http://www.pjrc.com/teensy/td_libs_Encoder.html
                     #endif //ROTARYMOD
#include <Wire.h>                // Default Arduino library
#include <RTClib.h>              // https://github.com/adafruit/RTClib 
#include <EEPROM.h>              // Default Arduino library To store data in EEPROM
#include <TimeLib.h>             // Arduino library Time by Michael Margolis   http://playground.arduino.cc/Code/Time/   
                     #ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>                  // https://github.com/adafruit/Adafruit_NeoPixel   for LED strip WS2812 or SK6812
                     #endif  // NEOPIXEL
                     #ifdef LIBSK6812
#include <EdSoft_SK6812.h>                      // https://github.com/ednieuw/EdSoft_SK6812
                     #endif // LIBSK6812
                     #ifdef KEYPAD3x1
#include <Keypad.h>              
                     #endif //KEYPAD3x1 
                     #ifdef BTMOD_PIN6_PIN7
#include <SoftwareSerial.h>                     // Arduino library for Bluetooth communication
                     #endif //BTMOD_PIN6_PIN7    
                     #ifdef MAX7219
#include <LedControl.h> 
                      #endif //MAX7219 
                     #ifdef HT16K33tijd
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"                //https://learn.adafruit.com/adafruit-led-backpack/overview?view=all
                     #endif //HT16K33tijd
//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
enum DigitalPinAssignments {
                    #ifdef STICKCLOCK  
 EmptyD00     = 0,                // EmptyD00
 EmptyD01     = 1,                // EmptyD01 
 EncoderPow   = 2,                // give power to Encoder
 clearButton  = 3,                // switch (labeled SW on decoder)
 encoderPinA  = 4,                // right (labeled DT on decoder)
 encoderPinB  = 5,                // left (labeled CLK on decoder)
 LED_PIN      = 11,               // Pin to control colour SK6812 WS2812 LEDs 
                     #endif //STICKCLOCK
                     #ifdef PCBCLOCK  
 EmptyD02     = 2,                // EmptyD02
 encoderPinA  = 3,                // right (labeled DT on Rotary decoder)
 clearButton  = 4,                // switch (labeled SW on Rotary decoder)
 LED_PIN      = 5,                // Pin to control colour SK6812 WS2812 LEDs
 encoderPinB  = 8,                // left (labeled CLK on Rotary decoder)
                     #endif //PCBCLOCK
                     #ifdef BTMOD_PIN6_PIN7
 BT_TX        = 6,                // Bluetooth TX
 BT_RX        = 7,                // Bluetooth RX
                     #endif //BTMOD_PIN6_PIN7    
                     #ifdef MAX7219
 MAX7219CLK   = 10,               // MAX7219CLK
 MAX7219CS    = 11,               // MAX7219CS 
 MAX7219DataIn= 12,               // MAX7219DataIn
                     #endif //MAX7219 
 HeartbeatLED = 9,                // EmptyD09 
 secondsPin   = 13};              // if set to 13 led will blink on board     SCK
                                   // Analogue hardware constants ----
enum AnaloguePinAssignments {
 EmptyA0      = 0,                // EmptyA0
 EmptyA1      = 1,                // EmptyA1 
 PhotoCellPin = 2,                // LDR pin
 OneWirePin   = 3,                // OneWire Keypad
 SDA_pin      = 4,                // SDA pin geel
 SCL_pin      = 5,                // SCL pin groen
 EmptyA6      = 6,                // EmptyA6
 EmptyA7      = 7};               // EmptyA7
//--------------------------------------------
// HC-12 Long Range Wireless Communication Module
//--------------------------------------------
                         #ifdef BTMOD_PIN6_PIN7
SoftwareSerial Bluetooth(BT_RX, BT_TX);                             // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
                         #endif //BTMOD_PIN6_PIN7                       
//--------------------------------------------
// LED
//--------------------------------------------
bool  LEDsAreOff         = false;                                   // If true LEDs are off except time display
int   Previous_LDR_read  = 0;                                       // Previous light sensor value
const byte BRIGHTNESS    = 32;                                      // BRIGHTNESS 0 - 255

                         #ifdef LED6812 
                            #ifdef NEOPIXEL   
Adafruit_NeoPixel LEDstrip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                             #endif //NEOPIXEL
                             #ifdef LIBSK6812
EdSoft_SK6812 LEDstrip(NUM_LEDS, LED_PIN);                          // Initialyse SK6812 library
                             #endif //LIBSK6812
                         #endif //LED6812  
                         #ifdef LED2812
                            #ifdef NEOPIXEL  
Adafruit_NeoPixel LEDstrip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                            #endif //NEOPIXEL
                         #endif //LED2812
//--------------------------------------------
// COLOURS
//--------------------------------------------   
#define CLOCK_PIXELS    5                                           // Number of cells in clock = 5 (1,1,2,3,5)
#define DISPLAY_PALETTE 1                                           // Default palette to start with
bool    FiboChrono =  true;                                         // true = Fibonacci, false = chrono clock display
byte    NormalExtremeUltimate = 0;                                  // 0 = Normal, 1 = Extreme, 2 = Ultimate display of colours                
byte    bits[CLOCK_PIXELS+1];                                       // Stores the hours=1 and minutes = 2 to set in LEDsetTime(byte hours, byte minutes)
byte    BitSet[CLOCK_PIXELS+1];                                     // For calculation of the bits to set
                         #ifdef LED2812
const uint32_t white  = 0xFFFFFF, lgray   = 0x666666;               // R, G and B on together gives white light
const uint32_t gray   = 0x333333, dgray   = 0x222222;                
                         #endif //LED2812
                         #ifdef LED6812    
const uint32_t white  = 0xFF000000,lgray  = 0x66000000;             // The SK6812 LED has a white LED that is pure white
const uint32_t dgray  = 0x22000000,gray   = 0x33000000;
                         #endif //LED6812  
const uint32_t black  = 0x000000, red     = 0xFF0000;
const uint32_t orange = 0xFF7600, yellow  = 0xFFDD00;
const uint32_t dyellow= 0xFFAA00, apple   = 0x80FF00;
const uint32_t brown  = 0xC65F00, green   = 0x00FF00;
const uint32_t grass  = 0x00FF80, sky     = 0x00FFFF;
const uint32_t marine = 0x0080FF, blue    = 0x0000FF;
const uint32_t pink   = 0xFF0080, purple  = 0xFF00FF;
const uint32_t colors[][5] =                                        // The colour palettes
   {//off   hours   minutes both;
   { white, red   , yellow , blue  , green },  // #1 Mondriaan
   { white, red   , dyellow, blue  , green },  // #2 Mondriaan1 
   { white, red   , green  , blue  , green },  // #0 RGB  
   { white, apple , green  , grass , blue  },  // #3 Greens
   { white, red   , grass  , purple, green },  // #4 Pastel                                                                 
   { white, orange, green  , marine, blue  },  // #5 Modern
   { white, sky   , purple , blue  , green },  // #6 Cold
   { white, red   , yellow , orange, green },  // #7 Warm
   { white, brown , grass  , sky   , green },  // #8 Earth
   { green, red   , dyellow, blue  , white }} ;// #9 Mondriaan 2         
//--------------------------------------------
// KY-040 ROTARY
//-------------------------------------------- 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);                            // Use digital pin  for encoder
                          #endif //ROTARYMOD      
long     Looptime             = 0;
byte     RotaryPress          = 0;                                  // Keeps track displaychoice and how often the rotary is pressed.
uint32_t RotaryPressTimer     = 0;
byte     NoofRotaryPressed    = 0;

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
const byte SLOPEBRIGHTNESS    = 80;                                 // Steepness of with luminously of the LED increases
const int  MAXBRIGHTNESS      = 1023;                               // Maximun value in bits  for luminously of the LEDs (1 - 1023)
const byte LOWBRIGHTNESS      = 5;                                  // Lower limit in bits of Brightness ( 0 - 255)   
byte     TestLDR              = 0;                                  // If true LDR inf0 is printed every second in serial monitor
byte     LDRread[24];                                               // Store average LDR-readings/4 per hour in this array
int      MinPhotocell         = 999;                                // Stores minimum reading of photocell;
int      MaxPhotocell         = 1;                                  // Stores maximum reading of photocell;
uint32_t SumLDRreadshour      = 0;
uint32_t NoofLDRreadshour     = 0;


//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define  MAXTEXT 80                                                 // Maximum characters for text printing
static   unsigned long msTick;                                      // The number of millisecond ticks since we last incremented the second counter
int      count; 
int      Delaytime            = 200;
byte     ChangeTime           = false;
byte     ChangeLightIntensity = false;
byte     Demo                 = 0;
byte     Toggle_HetWasIsUit   = 0;                                  // Turn off HetIsWas after 10 sec
byte     Is                   = true;                               // Toggle of displaying Is or Was
byte     Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte     lastday =0, lastminute = 0, lasthour = 0, sayhour = 0;
char     sptext[MAXTEXT+2];                                         // for common print use

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;
        #else
RTC_Millis RTCklok;   
        #endif
DateTime Inow;
                      #ifdef HT16K33tijd
//--------------------------------------------
// HT16K33 IC2 display
//--------------------------------------------
Adafruit_7segment Tijd_displayIC2 = Adafruit_7segment();
                      #endif //HT16K33tijd
//--------------------------------------------
// MAX7219 display
//--------------------------------------------               
                      #ifdef MAX7219
// const byte NUMBEROF7219DISPLAYS = 1;                             // 1 or 2. Second chained with first one
// pin 10 is connected to MAX7219CLK 
// pin 11 is connected to MAX7219CS 
// pin 12 is connected to MAX7219DataIn 
 LedControl lc= LedControl(MAX7219DataIn,MAX7219CLK,MAX7219CS,1);   // NUMBEROF7219DISPLAYS); in this case 1
                       #endif //MAX7219  

                      #if defined  ONEWIREKEYPAD3x1  || defined ONEWIREKEYPAD3x4  
unsigned long KeyLooptime = 0;
 String        KeypadString;
                       #endif
                       #ifdef KEYPAD3x1
//--------------------------------------------
// KEYPAD 3x1
//          -------- GND
//  R Y G   -------- Pin 2
//          -------- Pin 3
//          -------- Pin 4
// COLPIN is used as dummy pin that must always be LOW
// 
//--------------------------------------------
unsigned long KeyLooptime = 0;
const byte ROWS   = 3; 
const byte COLS   = 1; 
const byte COLPIN = 8;                                               // Column that is always LOW
char keys[ROWS][COLS] = {{'R'}, {'Y'}, {'G'}};
byte rowPins[ROWS] = {4, 3, 2};                                      // Connect to the row pinouts of the keypad
byte colPins[COLS] = {COLPIN};                                       // Connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
                       #endif //KEYPAD3x1                       
                       #ifdef KEYPAD3x4
//--------------------------------------------
// KEYPAD 3 x 4
//--------------------------------------------
unsigned long KeyLooptime = 0;
const byte    ROWS = 4; //four rows
const byte    COLS = 3; //three columns
char keys[ROWS][COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'} };
byte rowPins[ROWS] = {5, 4, 3, 2};                                    // Connect to the row pinouts of the keypad
byte colPins[COLS] = {8, 7, 6};                                       // Connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
                        #endif //KEYPAD3x4
byte   KeyInputactivated = false;                                     // if pressing the keys stop other events like EveryMinuteUpdate()

//----------------------------------------
// Common  
//----------------------------------------
struct EEPROMstorage {                                                // Data storage in EEPROM to maintain them after power loss
  int  Checksum;
  byte LightReducer;
  byte LowerBrightness;
  int  UpperBrightness;
  byte DisplayPalette;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
  byte FiboChrono;
  byte NoExUl;   
} Mem; // = {0};                                                      // Initialyse struct with zero's

//--------------------------------------------
// Menu
//--------------------------------------------
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890  
const char menu[][42] PROGMEM = {
 "Fibonacci-klok",
 "D D14042020 is date 14 April 2020",
// "E Normal, Extreme or Ultimate mode",
// "F Fibonacci or Chrono display",
 "L (L5)   Min light intensity (1-255)",
 "M (M999) Max light intensity(1-999)",
 "N (N2306)Turn OFF LEDs between Nhhhh",
 "P (P1) to select a palette (0-9)",
 "I for this info",
 "R Reset to default settings",
 "T Thhmmss is time (T031500)",
 "W Test LDR reading every second",
// "X Demo mode",
 "Y (Y50) Slope light intensity(1-250)",
 "Ed Nieuwenhuys March 2022" };
 
// --------------------------------------------------------------------------
// End Definitions                                                    
// --------------------------------------------------------------------------

//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
          InputDevicesCheck();                                      // Checks input from serial port, BT, keypads, rotarys and so on
 if(Demo) Demomode();
 else     EverySecondCheck();
}  
//--------------------------------------------
// ARDUINO Setup initialise the hardware  
//--------------------------------------------
void setup()                                                        // initialise the hardware 
{
 int32_t Tick = millis();                                           // Start the timer 
 Wire.begin();                                                      // start the wire communication I2C for RTC module
 Serial.begin(9600);                                                // Setup the serial port to 9600 baud 

while (!Serial)                                                    // Wait until serial port is started 
      {if (millis() - Tick >2000) break;}                           // Prevents hanging if serial monitor/port is not connected  Tekstprintln("*********\nSerial started"); 
 Tekstprintln("\n*********\nSerial started");
 pinMode(secondsPin,   OUTPUT );
 pinMode(HeartbeatLED, OUTPUT );  
                          #ifdef BTMOD_PIN6_PIN7 
                          #if defined ARDUINO_SAMD_MKRWIFI1010      // || defined ARDUINO_AVR_NANO_EVERY
 Serial1.begin(9600);                                               // Bluetooth connected to Serial1
 Tekstprintln("Bluetooth MKR");  
                          #else
 Bluetooth.begin(9600); 
 Tekstprintln("Bluetooth");  
                          #endif //ARDUINO_SAMD_MKRWIFI1010
                          #endif //BTMOD_PIN6_PIN7
                          #ifdef ROTARYMOD
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP);
                          #ifdef STICKCLOCK
 pinMode(EncoderPow,   OUTPUT );
 digitalWrite(EncoderPow,HIGH);                                     // Provide the rotary encoder with power
                          #endif //STICKCLOCK
 Tekstprintln("Rotary encoder");  

                          #endif //ROTARYMOD
                          #ifdef KEYPAD3x1
 // keypad.addEventListener(keypadEvent);                           // No interrupt needed. Loop is fast enought. Start the 3x1 keypad
 Tekstprintln("3*1 keypad");  
 digitalWrite(COLPIN,LOW);
                          #endif //KEYPAD3x1
                          #ifdef KEYPAD3x4
 keypad.addEventListener(keypadEvent);                              // Add an event listener for this keypad
 Tekstprintln("4*3 keypad");                                        // Start the 3x4 keypad
                          #endif //KEYPAD3x4
                          #ifdef ONEWIREKEYPAD3x4  
 Tekstprintln("Onewire keypad");  // The one wire keypad is enabled
                          #endif //ONEWIREKEYPAD3x4
                          #ifdef ONEWIREKEYPAD3x1  
 Tekstprintln("Onewire keypad3x1");  // The one wire keypad is enabled
                          #endif //ONEWIREKEYPAD3x1                    
                          #ifdef MOD_DS3231
 Tekstprintln("RTC DS3231");  
                          #else
 Tekstprintln("Internal clock");                        
                          #endif //MOD_DS3231
                          #ifdef NEOPIXEL 
 LEDstrip.begin();                                                  // Start communication to LED strip
 LEDstrip.setBrightness(BRIGHTNESS);                                // Set brightness of LEDs
 Tekstprintln("LIB NEOPIXEL");   
                          #endif //NEOPIXEL 
                          #ifdef LIBSK6812
 LEDstrip.setBrightness(BRIGHTNESS);                                // Set brightness of LEDs
 Tekstprintln("Lib Edsoft_SK6812");                                 // Initialyse SK6812 library
                          #endif //LIBSK6812
                          #ifdef LED6812    
 Tekstprintln("LEDs SK6812 enabled");
                          #endif //LED6812  
                          #ifdef LED2812
 Tekstprintln("LEDs WS2812");
                          #endif //LED2812 
                          #ifdef MAX7219                     
 InitialyseMAX7219();
 sprintf(sptext, "12 34 56");  PrintStringToSegDisplay(sptext);         
 Tekstprintln("MAX7219");    
 lc.clearDisplay(0);
                          #endif //MAX7219 
                          #ifdef HT16K33tijd
 Tijd_displayIC2.begin(0x70);
 Tijd_displayIC2.setBrightness(3);                                  // Set the display brightness (0-7):
 Tekstprintln("4-digit time HT16K33 display installed"); 
                          #endif //HT16K33tijd
                          #ifdef MOD_DS3231
 RTCklok.begin();                                                   // start the RTC-module
                          #else 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                 // if no RTC module is installed use the ATMEGAchip clock
                          #endif //MOD_DS3231
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(F(__DATE__), F(__TIME__));
 if (now.unixtime() < compiled.unixtime()) 
  {
   Tekstprint("RTC is older than compile time! Updating\n"); 
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
// RTCklok.adjust(DateTime(10,04,20, 12, 00, 10 ));
 EEPROM.get(0,Mem);                                                 // Get the data from EEPROM
 Mem.LightReducer    = constrain(Mem.LightReducer,1,250);           // and prevent unwanted value with the constaints
 Mem.LowerBrightness = min(Mem.LowerBrightness,250);                // 
 Mem.DisplayPalette  = min(Mem.DisplayPalette,9);                   // 
 Mem.TurnOffLEDsAtHH = min(Mem.TurnOffLEDsAtHH,23);                 //
 Mem.TurnOnLEDsAtHH  = min(Mem.TurnOnLEDsAtHH,23);                  //
 Mem.FiboChrono      = min(Mem.FiboChrono,1);                       //
 Mem.NoExUl          = min(Mem.NoExUl,2);                           // 
 EEPROM.put(0,Mem);                                                 // update EEPROM if some data are out of the constrains  
 DS3231NVRAMRead(0,LDRread);                                        // Read the LDR reading from NVRAM DS3231 clock module if present
 Previous_LDR_read = analogRead(PhotoCellPin);                      // A start value
 MinPhotocell      = Previous_LDR_read+1;                           // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                             // Stores maximum reading of photocell;
 //Selftest();                                                      // Play the selftest
 GetTijd(0);                                                        // Get the time and store it in the proper variables
 SWversion();                                                       // Display the version number of the software
// Displaytime();
 if( Mem.Checksum != 25065) Reset();                                // If the checksum is incorrect the data were not set to default values
 Looptime = millis();                                               // Used in KY-040 rotary
 msTick   = millis();                                               // Used in second loop 
 } 
//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 Mem.Checksum         = 25065;
 Mem.LightReducer     = SLOPEBRIGHTNESS;                            // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.UpperBrightness  = MAXBRIGHTNESS;                              // Upper limit of Brightness in bits ( 1 - 1023)
 Mem.LowerBrightness  = LOWBRIGHTNESS;                              // Lower limit of Brightness in bits ( 0 - 255)
 Mem.DisplayPalette   = DISPLAY_PALETTE;
 Mem.TurnOffLEDsAtHH  = 0;
 Mem.TurnOnLEDsAtHH   = 0;
 Mem.FiboChrono       = true;                                       // true = Fibonacci, false = chrono clock display
 Mem.NoExUl           = 0;                                          // 0 = Normal, 1 = Extreme, 2 = Ultimate display of colours
 Previous_LDR_read    = analogRead(PhotoCellPin);                   // To have a start value
 MinPhotocell         = Previous_LDR_read;                          // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                          // Stores maximum reading of photocell;
 TestLDR              = 0;                                          // If true LDR display is printed every second
 ChangeTime           = false;
 ChangeLightIntensity = false;
 for (int i=0;i<24;i++) LDRread[i] = 1;                             // Reset LDR readings 
 DS3231NVRAMWrite(0,LDRread);                                       // And write it of NVRAM of DS3231
 EEPROM.put(0,Mem);                                                 // Update EEPROM if some data are out of the constrains                                                 
// Selftest();                                                        // Play the selftest
 GetTijd(0);                                                        // Get the time and store it in the proper variables
 SWversion();                                                       // Display the version number of the software
 Displaytime();
}
//--------------------------------------------
// Version
//-------------------------------------------- 
void SWversion(void) 
{ 
 unsigned int i;
 PrintLine(40);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)   {strcpy_P(sptext, menu[i]);            Tekstprintln(sptext);}
 PrintLine(46);
// sprintf(sptext,"LDR average bits measured per hour");                                       Tekstprintln(sptext); 
 for (i=0;i<12;i++) { sprintf(sptext," %02d ",i );          Tekstprint(sptext); }              Tekstprintln("");
 for (i=0;i<12;i++) { sprintf(sptext,"%03d ",4*LDRread[i]); Tekstprint(sptext); }              Tekstprintln("");
 for (i=12;i<24;i++){ sprintf(sptext,"%03d ",4*LDRread[i]); Tekstprint(sptext); }              Tekstprintln("");
 PrintLine(46);
 sprintf(sptext,"Brightness Min: %3d  Max: %3d Slope: %3d%%",Mem.LowerBrightness, Mem.UpperBrightness, Mem.LightReducer);  Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min: %3d  Max: %3d",MinPhotocell, MaxPhotocell);          Tekstprintln(sptext);
 sprintf(sptext,"Number of LEDs: %d Palette: %d", NUM_LEDS, Mem.DisplayPalette);             Tekstprintln(sptext); 
 sprintf(sptext," LEDs off from: %02d - %02d",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);        Tekstprintln(sptext);
 sprintf(sptext,"Version: %s",VERSION);                                                        Tekstprintln(sptext); 
 PrintLine(40);
 GetTijd(1);
}
void PrintLine(byte Lengte)
{ 
  for (int n=0; n<Lengte; n++) {Serial.print(F("_"));} 
  Serial.println(); 
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck()
{
 uint32_t ms = millis() - msTick;                                  // A Digitalwrite() is very time consuming. 
 static bool Dpin;                                                 // Only write once to improve program speed in the loop()
 // Heartbeat();                                                   // Show a heartbeat on a LED
 if ( ms > 1 && Dpin) {Dpin = LOW; digitalWrite(secondsPin,LOW);}  // Turn OFF the second on pin 13 
 if ( ms >999)                                                     // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                              // second++; 
   digitalWrite(secondsPin,HIGH);                                  // Turn ON the second on pin 13
   Dpin = HIGH;
   GetTijd(0);                                                     // Synchronize time with RTC clock
                          #ifdef MAX7219
   PrintTimetoMAX7219();
                          #endif //MAX7219                                                           
   DimLeds(TestLDR);                                               // Every second an intensitiy check and update from LDR reading 
   if (Mem.FiboChrono)                                             // If Fibonacci display and not chrono (clock display) mode
     {if(Iminute == 0 && Isecond <9) Displaytime();}               // Force the clock to calculate a new combination
   else Displaytime();  //if(Isecond % 5 == 0)                     // if Chrono clock display then Display seconds every second
   if(!KeyInputactivated) EveryMinuteUpdate();                     // If keyboard input then do not update display
  }
}

//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
void EveryMinuteUpdate(void)
{
 if (Iminute != lastminute)                                         // Show time every minute
  { 
    if ( (uint32_t) (millis() - RotaryPressTimer) > 60000)          // 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }   
   GetTijd(0);
                           #ifdef HT16K33tijd
    sprintf(sptext,"%02d:%02d",Inow.hour(),Inow.minute());
    for (int i=0; i<5; i++)  Tijd_displayIC2.writeDigitNum(i, sptext[i]- '0');
    Tijd_displayIC2.drawColon(true); 
    Tijd_displayIC2.writeDisplay();
                           #endif //HT16K33tijd
    lastminute = Iminute;
    Displaytime();
  } 
 if (Ihour != lasthour) EveryHourUpdate(); 
}

//--------------------------------------------
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 GetTijd(0); 
 if(Ihour == Mem.TurnOffLEDsAtHH) LEDsAreOff = true;                // Is it time to turn off the LEDs?
 if(Ihour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;               // Or on?
 LDRread[lasthour] = (byte)((SumLDRreadshour / NoofLDRreadshour)/4);
 SumLDRreadshour  = 0;
 NoofLDRreadshour = 0;
 DS3231NVRAMWrite(0,LDRread); 
 lasthour = Ihour;
 if (Iday != lastday) EveryDayUpdate(); 
}

//--------------------------------------------
// CLOCK Update routine done every day
//--------------------------------------------
void EveryDayUpdate(void)
{
 lastday = Iday;                                                    // Not much to do at the moment
 Previous_LDR_read = analogRead(PhotoCellPin);                      // to have a start value
 MinPhotocell      = Previous_LDR_read+1;                             // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                             // Stores maximum reading of photocell;
//  MinPhotocell = 999;                                                // Stores minimum reading of photocell;
//  MaxPhotocell = 1;                                                  // Stores maximum reading of photocell;
}


//--------------------------------------------
// CLOCK Check for input from devices
// This fubction is called from with the loop()
//--------------------------------------------
void InputDevicesCheck(void)
{
 SerialCheck();
                           #ifdef BTMOD_PIN6_PIN7   
 BluetoothCheck(); 
                           #endif //BTMOD_PIN6_PIN7  
                           #ifdef ROTARYMOD      
 RotaryEncoderCheck(); 
                           #endif //ROTARYMOD
                           #ifdef KEYPAD3x4   
 Keypad3x4Check(); 
                           #endif //KEYPAD3x4
                           #ifdef KEYPAD3x1   
 Keypad3x1Check(); 
                           #endif //KEYPAD3x1
                           #ifdef ONEWIREKEYPAD3x4   
 OnewireKeypadCheck(); 
                           #endif //ONEWIREKEYPAD3x1
                           #ifdef ONEWIREKEYPAD3x1   
 OnewireKeypad3x1Check(); 
                           #endif //ONEWIREKEYPAD3x1

}
//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
                           #ifdef BTMOD_PIN6_PIN7   
 Bluetooth.print(tekst);                                 // If Bluetooth module is not connected to pin 0 and pin 1 
                           #endif //BTMOD_PIN6_PIN7
}

void Tekstprintln(char const *tekst)
{
 sprintf(sptext,"%s\n",tekst);
 Tekstprint(sptext);   
} 
//--------------------------------------------
// CLOCK Heart beat in LED
//--------------------------------------------
void Heartbeat() 
{
 static byte hbval         = 128;    // Heartbeat initial intensity
 static byte hbdelta       = 10;     // Determines how fast heartbeat is
 static uint32_t last_time = 0;  
 unsigned long now         = millis();
 if ((now - last_time) < 40)    return;
 last_time = now;
 if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
 hbval += hbdelta;
 analogWrite(HeartbeatLED, hbval);
}
//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
 if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);        // Turn OFF the second on pin 13
 if ( millis() - msTick >999)                                       // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                // second++; 
  digitalWrite(secondsPin,HIGH);                                    // turn ON the second on pin 13
  if (++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
  if (    Ihour >23)  Ihour = 0;
  DimLeds(false);
  Displaytime();
 }
}

//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String   SerialString ="";
 char c = 0;
 while (Serial.available() && c!=13)
  { 
   delay(3);
   c = Serial.read();
   if (c>31 && c<123) SerialString += c;                            // Allow input from Space - Del
  }
 if (SerialString.length()>0)  {Serial.println(SerialString) ;  ReworkInputString(SerialString);}  // Rework ReworkInputString();
 SerialString = "";
}
                           #ifdef BTMOD_PIN6_PIN7
//--------------------------------------------
// BLUETOOTH check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 String  BluetoothString = "";
 char c = 0;
                           #ifdef ARDUINO_SAMD_MKRWIFI1010 
 while (Serial1.available()) 
  {
   c = Serial1.read();
   Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;     // delete a CR
  }
                           #else
 Bluetooth.listen();                                                                  //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  { 
   c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;
   delay(3);
  }
                           #endif  // 
 if (BluetoothString.length()>0) 
   {   
    ReworkInputString(BluetoothString);                                                      // Rework ReworkInputString();
    BluetoothString = "";
   }
}
                          #endif //BTMOD_PIN6_PIN7  
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  Play_Lights();     
}
                           #ifdef ONEWIREKEYPAD3x4
 //--------------------------------------------
// KEYPAD check for Onewire Keypad input 
//--------------------------------------------
void OnewireKeypadCheck(void)
{
 int keyvalue = 99;
 char Key;
 int sensorValue = analogRead(OneWirePin); // read the value from the sensor:   Serial.println(sensorValue);
 switch(sensorValue)
  {
    case   0 ... 120:  keyvalue = 13; break;   // noise
    case 121 ... 132:  keyvalue = 12; Key = '*'; break;   // * 
    case 133 ... 154:  keyvalue =  0; Key = '0'; break;   // 0 
    case 155 ... 216:  keyvalue = 11; Key = '#'; break;   // # 
    case 217 ... 281:  keyvalue =  7; Key = '7'; break;   // 7 
    case 282 ... 318:  keyvalue =  4; Key = '4'; break;   // 4 
    case 319 ... 349:  keyvalue =  1; Key = '1'; break;   // 1 
    case 350 ... 390:  keyvalue =  8; Key = '8'; break;   // 8 
    case 391 ... 463:  keyvalue =  5; Key = '5'; break;   // 5 
    case 464 ... 519:  keyvalue =  2; Key = '2'; break;   // 2 
    case 520 ... 619:  keyvalue =  9; Key = '9'; break;   // 9 
    case 620 ... 848:  keyvalue =  6; Key = '6'; break;   // 6 
    case 849 ... 1023: keyvalue =  3; Key = '3'; break;   // 3
  }
 if(keyvalue<13) { Serial.println(Key);  delay(300); }
 if (keyvalue == 11)    // #                                             // Pressing a * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString ="";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                            // Turn all LEDs green
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (keyvalue>=0 && keyvalue<10))
   {
    delay(20); 
    KeypadString += Key;                                            // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                // Turn all LEDs red
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if (KeypadString.length()>5)                                       // If six numbers are entered rework this to a time hhmmss
   {       
   if(KeypadString=="999999")                                       // If 999999 is entered reset the clock to default settings
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock settings resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                              // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }    
   }
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) ) 
   {  
    KeyInputactivated = false;                                      // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
}
                           #endif //ONEWIREKEYPAD3x4
                           #ifdef KEYPAD3x4
//--------------------------------------------
// KEYPAD check for Keypad input
//--------------------------------------------                           
void Keypad3x4Check(void)
{ 
  char    Key = 0;
  String  KeypadString;
  unsigned long KeyLooptime = 0;
  Keypad3x4.tick(); 
  while(Keypad3x4.available())
   {
    keypadEvent e = Keypad3x4.read();  
    if(e.bit.EVENT == KEY_JUST_PRESSED)
     {
 //     Serial.println(F(" pressed"));  
      delay(20);
      }  
    else  if(e.bit.EVENT == KEY_JUST_RELEASED) 
     {
      Key = (char) e.bit.KEY;
//      Serial.print(Key);  Serial.println(F(" released"));
     Keypad3x4.clear();
     delay(20);
    }
   }
 if (Key == 42)   // *                                              // Pressing * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();                                         // Start 60 sec timer
    KeypadString = "";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                            // Turn all LEDs green
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>47 && Key<58))
   {
    delay(20); 
    KeypadString += Key;                                            // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                // Turn all LEDs red
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if(KeypadString.length()>5)                                        // if six numbers are entered rework this to a time hhmmss
   {  
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock setting resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                              // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }
   }
  if (Key == 35)   // #                                             // Pressing # changes palettes. 
   { 
    KeypadString ="";
    Mem.DisplayPalette++;
    Mem.DisplayPalette = min(Mem.DisplayPalette,9);
    Displaytime();
   }
   
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 60000) )     // Stop keyboard entry after 60 seconds
   {  
    KeyInputactivated = false;                                      // Stop data entry after 60 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
} 
                           #endif //KEYPAD3x4   
                           #ifdef KEYPAD3x1
 //--------------------------------------------
// KEYPAD check for Keypad input
//--------------------------------------------                           
void Keypad3x1Check(void)
{ 
 char    Key = 0;
 String  KeypadString;
 digitalWrite(COLPIN,LOW);                                          // Mimic a key prees on pin 6 in order to select the first column
 char Key = keypad.getKey();
 if(Key)
  {
   Serial.println(Key);
   if (Key == 'R') ProcessKeyPressTurn(1);                          // Pressing Red increases hour or minute. 
   if (Key == 'G') ProcessKeyPressTurn(-1);                         // Pressing Green decreases hour or minute. 
   if (Key == 'Y') ProcessKeyPressTurn(0);                          // Pressing Yellow activates the keyboard input. 
   delay(200);
  }
} 
                           #endif //KEYPAD3x1          
                           #ifdef ONEWIREKEYPAD3x1
//--------------------------------------------
// KEYPAD check for Onewire Keypad input with 5V and 1.1, 4.7, 4.7, 4.7 kOhm resistors
//--------------------------------------------
void OnewireKeypad3x1Check(void)
{
 char          keyvalue =99, Key;
 String        KeypadString;
 int           sensorValue = analogRead(OneWirePin);               // Read the value from the sensor:
 switch(sensorValue)
   {
    case   0 ... 385:  keyvalue = 99;            break;            // Noise
    case 386 ... 635:  keyvalue = -1; Key = 'G'; break;            // G 
    case 636 ... 910:  keyvalue =  0; Key = 'Y'; break;            // Y 
    case 911 ... 1024: keyvalue =  1; Key = 'R'; break;            // R 
   }
 if(keyvalue<2) 
    { 
     Serial.print(sensorValue); Serial.println(Key); 
     if (Key == 'R') ProcessKeyPressTurn(1);                          // Pressing Red increases hour or minute. 
     if (Key == 'G') ProcessKeyPressTurn(-1);                         // Pressing Green decreases hour or minute. 
     if (Key == 'Y') ProcessKeyPressTurn(0);                          // Pressing Yellow activates the keyboard input. 
     delay(200);     
    }
}
                           #endif //ONEWIREKEYPAD3x1
//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
                           #ifdef ROTARYMOD
void RotaryEncoderCheck(void)
{
 int ActionPress=0;
 ActionPress = myEnc.read();
 if(ActionPress !=0 ) ProcessKeyPressTurn(ActionPress);
 if (digitalRead(clearButton)==LOW ) 
                     {ProcessKeyPressTurn(0); delay(200); }        // Set the time by pressing rotary button
 myEnc.write(0);                                                   // Set encoder pos back to 0
}
                           #endif //ROTARYMOD
//--------------------------------------------
// KY-040 or Membrane 3x1 processing input
// encoderPos < 1 left minus 
// encoderPos = 0 attention and selection choice
// encoderPos > 1 right plus
//--------------------------------------------
void ProcessKeyPressTurn(int encoderPos)
{
 if (ChangeTime || ChangeLightIntensity)                            // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos && ( (millis() - Looptime) > 250))              // If rotary turned avoid debounce within 0.25 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                             // Increase  MINUTES of light intensity
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }        // If time < 60 sec then adjust light intensity with 5 bits up
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              {if( ++Ihour >23) { Ihour = 0; } }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              {
               Isecond = 0;
               if( ++Iminute >59) { Iminute = 0; if( ++Ihour >23) { Ihour = 0; } }   
              }  }  }    
     if (encoderPos <0)                                             // Increase the hours
       {
        if (ChangeLightIntensity)   { WriteLightReducer(-5); }      // If time < 60 sec then adjust light intensity with 5 bits down
        if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              { if( Ihour-- ==0) { Ihour = 23; } }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              { 
              Isecond = 0;
              if( Iminute-- == 0) { Iminute = 59; if( Ihour-- == 0) { Ihour = 23; } }   
              }  }  } 
      SetRTCTime();  
      Looptime = millis();   
     }                                                
   }
 if (encoderPos==0)
  {
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    RotaryPressTimer      = millis();                               // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >15 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                      // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true; 
                Tekstprintln("Changing time started for 60s");
                ColorLeds("",0,NUM_LEDS-1,red);
                ShowLeds(); 
                delay(500);                           break;        // Change the hours RED 
       case 2:  ChangeTime = true;
                ColorLeds("",0,NUM_LEDS-1,yellow);
                ShowLeds(); 
                delay(500);                           break;        // Change the minutes YELLOW        
       case 3:  ChangeLightIntensity = true;
                ColorLeds("",0,NUM_LEDS-1,white);
                ShowLeds();
                delay(500);                           break;        // Change intensity  
       case  4:  Mem.DisplayPalette = 1;              break;
       case  5:  Mem.DisplayPalette = 2;              break;        
       case  6:  Mem.DisplayPalette = 3;              break;
       case  7:  Mem.DisplayPalette = 4;              break;
       case  8:  Mem.DisplayPalette = 5;              break;
       case  9:  Mem.DisplayPalette = 6;              break;        
       case 10:  Mem.DisplayPalette = 7;              break;
       case 11:  Mem.DisplayPalette = 8;              break;
       case 12:  Mem.DisplayPalette = 9;              break;
       case 13:  Mem.DisplayPalette = 0;              break;
       case 14:                                       break; 
       case 15: NoofRotaryPressed = 0;   Reset();     break;                     
       default: NoofRotaryPressed = 0;                break;                         
      }  
    sprintf(sptext,"NoofRotaryPressed:%d", NoofRotaryPressed);  Tekstprintln(sptext);
    Looptime = millis();     
    Displaytime(); 
                           #ifdef MAX7219
    PrintTimetoMAX7219();
                           #endif //MAX7219  
   }
 }
//--------------------------------------------
// FIBONACCI Set the color and strip based on the time
//--------------------------------------------
void LEDsetTime(byte hours, byte minutes)
{ 
 hours %=12;                                                       // Keep the hours between 0 and 12
 for(int i=0; i<CLOCK_PIXELS; i++) { bits[i] = 0; BitSet[i] = 0; } // Clear all bits  
 MakeFibonacciList(hours);
 for(int i=0; i<CLOCK_PIXELS; i++) if(BitSet[i]) bits[i] +=1;      // If hour must be lit add 1
 for(int i=0; i<CLOCK_PIXELS; i++)    BitSet[i] = 0;               // Clear  bits  
 MakeFibonacciList(minutes/5);                                     // Block is 5 minutes  
  for(int i=0; i<CLOCK_PIXELS; i++)  
 {
   if( BitSet[i]) bits[i] +=2;    
   setPixel(i, colors[Mem.DisplayPalette][bits[i]]);  
// Serial.println(colors[Mem.DisplayPalette][bits[i]],HEX);
  }
}

//--------------------------------------------
// FIBONACCI Calculate the proper Fibonacci-numbers (Pixels)
//--------------------------------------------
void MakeFibonacciList(byte Value)
{
 byte CalcValue = 0;
 byte pos = CLOCK_PIXELS;
 while (Value != CalcValue  )
  {
   byte Fibonaccireeks[] = {1,1,2,3,5,0};                          // Set up Fibonacci array with 6 numbers.
   for(int i=0; i<=CLOCK_PIXELS; i++) BitSet[i] = 0;               // Clear all bits. NB CLOCK_PIXELS is noof cells / strips in the clock
   CalcValue = 0;
   while ( (Value != CalcValue) &&  CalcValue <=  Value)   
    {
     do { pos = random(CLOCK_PIXELS); } while(Fibonaccireeks[pos] == 0 );   
     CalcValue += Fibonaccireeks[pos];
     BitSet[pos] = 1;                                              // Set pos in array for valid number    
     Fibonaccireeks[pos] = 0;                                      // Number taken from array 
    }
  }
}

//--------------------------------------------
// FIBONACCI Calculate the proper chronological numbers (Pixels)
//--------------------------------------------
void MakeChronoList(byte Hours, byte Minutes,byte Seconds)
{
 Hours %=12;                                                       // Keep the hours between 0 and 12
 byte Secsegment = Seconds / 5;
 byte Minsegment = Minutes / 5;
 byte Bit;
 uint32_t Kleur;                                                   // Color
 for(int i=0; i<12; i++)
  {
   Bit = 0;
   if(i < Hours)        Bit+= 1;                                   // If hours use the second colour
   if(i < Minsegment)   Bit+= 2;                                   // If minute use the third colour. If hours was set the fourth colour is displayed 
   if(Mem.NoExUl>0 && i == Secsegment)  Bit = 4;                   // If second use the fifth colour to display 
   Kleur = colors[Mem.DisplayPalette][Bit];
   if(Mem.NoExUl>1) {if(i<Minutes%5 && Seconds%5<1) Kleur=purple;} // If in Ultimate mode
   ColorLed(i,Kleur); 
   }
}
//--------------------------------------------
// FIBONACCI Turn on the right pixels and colours for 24 hour 
//--------------------------------------------
void setPixel(byte pixel, uint32_t kleur)
{
  switch(pixel)                                                    // 14 LEDs    
  {
   case 0:      ColorLeds("", 0, 0,kleur); break;
   case 1:      ColorLeds("", 1, 1,kleur); break;
   case 2:      ColorLeds("", 2, 3,kleur); break;
   case 3:      ColorLeds("", 4, 7,kleur); break;
   case 4:      ColorLeds("", 8,13,kleur); break;
  }

  /*
 switch(pixel)                                                     // 12 LEDs 
  {
   case 0:      ColorLeds("", 0, 0,kleur); break;
   case 1:      ColorLeds("", 1, 1,kleur); break;
   case 2:      ColorLeds("", 2, 3,kleur); break;
   case 3:      ColorLeds("", 4, 6,kleur); break;
   case 4:      ColorLeds("", 7,11,kleur); break;
  }
                                                                   // For 50x50 cm case with 174 LEDs    
 switch(pixel)  
  {
   case 0:      ColorLeds("",  0,   15,kleur); break;
   case 1:      ColorLeds("", 16,   31,kleur); break;
   case 2:      ColorLeds("", 32,   63,kleur); break;
   case 3:      ColorLeds("", 64,  103,kleur); break;
   case 4:      ColorLeds("", 104, 173,kleur); break; 
  }
                                                                   // For case with 3 x 12 LEDs clock 
  switch(pixel)  
  {
   case 0:      ColorLeds("", 0,   2,kleur); break;
   case 1:      ColorLeds("", 3,   5,kleur); break;
   case 2:      ColorLeds("", 6,  11,kleur); break;
   case 3:      ColorLeds("", 12, 20,kleur); break;
   case 4:      ColorLeds("", 21, 35,kleur); break; 
  }

                                                                       // For case with clock 
   switch(pixel)  
  {
   case 0:      ColorLeds("", 0,   2,kleur); break;
   case 1:      ColorLeds("", 3,   5,kleur); break;
   case 2:      ColorLeds("", 6,  11,kleur); break;
   case 3:      ColorLeds("", 12, 20,kleur); break;
   case 4:      ColorLeds("", 21, 35,kleur); break; 
  }

  switch(pixel)                                                     // for 32 LEDs, 4 strips of 8 LEDs
   {
    case 0:      ColorLeds("", 2, 3,kleur);                            break;
    case 1:      ColorLeds("",12,13,kleur);                            break;
    case 2:      ColorLeds("", 0, 1,kleur); ColorLeds("",14,15,kleur); break;
    case 3:      ColorLeds("",16,19,kleur); ColorLeds("",28,31,kleur); break;
    case 4:      ColorLeds("", 4,11,kleur); ColorLeds("",20,27,kleur); break;
   }
 */  
}

//--------------------------- Time functions --------------------------
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Displaytime(void)
{     
  if (Mem.FiboChrono) LEDsetTime(Ihour , Iminute);                 // Fibonacci display   
  else            MakeChronoList(Ihour , Iminute, Isecond);        // Chrono (clock display) display
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTCklok.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
 Iday    = Inow.day();
 Imonth  = Inow.month();
 Iyear   = Inow.year()-2000;
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 Inow = RTCklok.now();
 sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}
                     
//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = min((byte)Ihour  ,24);
 Iminute = min((byte)Iminute,59); 
 Isecond = min((byte)Isecond,59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(1);                                       // Synchronize time with RTC clock
 Displaytime();
// Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int  temp3231;
 
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    // Temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & 0b01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else   {temp3231 = -273; }  
  return (temp3231);
}
                     #ifdef MAX7219
//--------------------------------------------
//  MAX7219 Print time to MAX7219
//--------------------------------------------
void PrintTimetoMAX7219(void)
{
 sprintf(sptext,"%02d_%02d_%02d",Ihour,Iminute,Isecond);
 PrintStringToSegDisplay(sptext); 
}
//--------------------------------------------
//  MAX7219 Print in MAX7219 with 16 digits
//--------------------------------------------
void PrintStringToSegDisplay(char *text)
{
if(LEDsAreOff) {lc.shutdown(0,false); lc.clearDisplay(0); return; }
if(Mem.TurnOffLEDsAtHH!=Mem.TurnOnLEDsAtHH)
    if(Ihour >= Mem.TurnOffLEDsAtHH && Ihour<24)
      if(Ihour>=0 && Ihour<Mem.TurnOnLEDsAtHH) 
               {lc.shutdown(0,false); lc.clearDisplay(0); return;  }  
 for (int n = 0; n<16;n++)
    {
     int d = text[n] - '0';  
     lc.setDigit(n/8, 7-n%8, d ,false);
     }
}
//--------------------------------------------
//  MAX7219 Initialyse MAX7219 with two units
//--------------------------------------------
void InitialyseMAX7219(void)
{
 lc.shutdown(0,false); 
 lc.setIntensity(0,0);                                               // Set the brightness 0 - 15
 lc.clearDisplay(0);                                                 // and clear the display
}
                     #endif //MAX7219 
// --------------------Light functions -----------------------------------
//--------------------------------------------
//  LED Set color for LED
//--------------------------------------------
void ColorLeds(const char *Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{   
 Stripfill(RGBWColor, FirstLed, ++LastLed - FirstLed );    //for (int n = FirstLed; n <= LastLed; n++)  strip.setPixelColor(n,RGBWColor  );//  Serial.println(RGBWColor,HEX); 
 if (strlen(Tekst) > 0 ){sprintf(sptext,"%s ",Tekst); Tekstprint(sptext); }   // Print the Tekst  
}
//--------------------------------------------
//  LED Set color for one LED
//--------------------------------------------
void ColorLed(int Lednr, uint32_t RGBWColor)
{   
 Stripfill(RGBWColor, Lednr, 1 );
}
//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{
 LEDstrip.show();
}
//--------------------------------------------
//  LED Set brighness of LEDs
//  Value between 0 and 255
//--------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
 LEDstrip.setBrightness(Bright);                                           // Set brightness of LEDs   
 ShowLeds();
}
//--------------------------------------------
//  LED Fill the strip array for LEDFAB library
//--------------------------------------------
void Stripfill(uint32_t RGBWColor, int FirstLed, int NoofLEDs)
{
 LEDstrip.fill(RGBWColor, FirstLed, NoofLEDs);
}
//--------------------------------------------
//  LED Strip Get Pixel Color 
//--------------------------------------------
uint32_t StripGetPixelColor(int Lednr)
{
return(LEDstrip.getPixelColor(Lednr));
}
//--------------------------------------------
//  LED function to make RGB color
//-------------------------------------------- 
uint32_t FuncCRGB(uint32_t Red, uint32_t Green, uint32_t Blue)
{
return ( (Red<<16) + (Green<<8) + Blue);
}
//--------------------------------------------
//  LED function to make RGBW color
//-------------------------------------------- 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red <<16) + (Green <<8) + Blue );
}
//--------------------------------------------
//  LED functions to extract RGBW colors
//-------------------------------------------- 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }

//--------------------------------------------
//  LED Dim the leds measured by the LDR and print values
// LDR reading are between 0 and 1024. The Brightness send to the LEDs is between 0 and 255
//--------------------------------------------
void DimLeds(bool print) 
{                                                                                                       
 int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;                   // Read lightsensor and avoid rapid light intensity changes
 Previous_LDR_read = LDR_read;                                                            // by using the previous reads
 int OutPhotocell = (int)((Mem.LightReducer * sqrt(63.5*LDR_read))/25);                       // Linear --> hyperbolic with sqrt. Result is between 0-1024
 MinPhotocell = min(MinPhotocell, LDR_read);                                              // Lowest LDR measurement
 MaxPhotocell = max(MaxPhotocell, LDR_read);                                              // Highest LDR measurement
 int BrightnessCalcFromLDR  
          = constrain(OutPhotocell, Mem.LowerBrightness, Mem.UpperBrightness);            // Keep result between lower and upper boundery ( 0 and 1024)
 SumLDRreadshour += LDR_read;    NoofLDRreadshour++;                                      // For statistics LDR readings per hour
 if(print)
 {
  sprintf(sptext,"LDR:%3d Avg:%3d",analogRead(PhotoCellPin),LDR_read);        Tekstprint(sptext);
  sprintf(sptext," (%3d-%3d)",MinPhotocell,MaxPhotocell);                     Tekstprint(sptext);
  sprintf(sptext," [%d]Out:%3d",Mem.UpperBrightness, BrightnessCalcFromLDR);  Tekstprint(sptext);
  sprintf(sptext,"=%2d%%",(int)(BrightnessCalcFromLDR / 10));                 Tekstprint(sptext);
  sprintf(sptext," Temp:%2dC ",get3231Temp()-1);                              Tekstprint(sptext);// Correct the reported temperature 
  Print_tijd();  
 }
 if(LEDsAreOff) BrightnessCalcFromLDR = 0;
 SetBrightnessLeds((byte) (BrightnessCalcFromLDR/4));                                      // Value is now between 0-1024 and must be between 0 - 255
}

//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
 for(int j=0; j<3; j++) WhiteOverRainbow(50,50,3);     // Loop three times            
}

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(int amount)
{
 Mem.LightReducer = (byte) min(Mem.LightReducer + amount, 255);                                                 // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
 Tekstprintln(sptext);
}
/*/------------------------------------------------------------------------------
//  LED Write highest allowable light intensity to EEPROM
//------------------------------------------------------------------------------
void WriteUpperBrightness(int waarde)
{
 Mem.UpperBrightness = max(waarde, 1023);                                                      // Range between 0 and 255
 sprintf(sptext,"Upper brightness: %3d bits", Mem.UpperBrightness);
 Tekstprintln(sptext);
}
//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(int value)
{
 Mem.LowerBrightness = min(value, 255);                // Range between 0 and 255
 sprintf(sptext,"Lower brightness: %3d bits", Mem.LowerBrightness);
 Tekstprintln(sptext);
}
*/
//--------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//--------------------------------------------

uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}
/*
//--------------------------------------------
//  LED RainbowCycle
//--------------------------------------------
// Slightly different, this makes the rainbow equally distributed throughout
void RainbowCycle(uint8_t wait) 
{
  uint16_t i, j;
  for(j=0; j<256 * 5; j++)          // 5 cycles of all colors on wheel
   { 
    for(i=0; i< NUM_LEDS; i++) 
     {
 //   strip.setPixelColor(i, Wheel(((i * 256 / NUM_LEDS) + j) & 255));
      ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
     }
   ShowLeds();
   delay(wait);
  }
}
*/
//--------------------------------------------
//  LED WhiteOverRainbow
//--------------------------------------------
void WhiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) 
{
  if(whiteLength >= NUM_LEDS) whiteLength = NUM_LEDS - 1;
  int head = whiteLength - 1;
  int tail = 0;
  int loops = 1;
  int loopNum = 0;
  static unsigned long lastTime = 0;
  while(true)
  {
    for(int j=0; j<256; j++) 
     {
      for(int i=0; i<NUM_LEDS; i++) 
       {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) )
              ColorLeds("",i,i+3,white);     // White
        else  ColorLeds("",i,i+3,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
       }
      if(millis() - lastTime > whiteSpeed) 
       {
        head++;        tail++;
        if(head == NUM_LEDS) loopNum++;
        lastTime = millis();
      }
      if(loopNum == loops) return;
      head %= NUM_LEDS;
      tail %= NUM_LEDS;
      ShowLeds();
      delay(wait);
    }
  }  // end while
}

// --------------------End Light functions 

//--------------------------------------------
//  CLOCK Constrain a string with integers
// The value between the first and last character in a string is returned between the low and up bounderies
//--------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 InputString.trim();                                       // Remove trailing spaces
 if (InputString.length()>10) return;                      // If string is too long for some reason
 if (InputString[0] > 47 && InputString[0] <123)           // If the first charater is a letter
  {
  sprintf(sptext,"**** Length fault ****");                // Default message 
  switch (InputString[0]) 
   {
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              Iday   = (byte) SConstrainInt(InputString,1,3,0,31);
              Imonth = (byte) SConstrainInt(InputString,3,5, 0, 12); 
              Jaar   =        SConstrainInt(InputString,5,9, 2000, 3000); 
              RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
             }
            break;
/*   case 'E':
   case 'e':
            if(InputString.length() == 1)
              {
               if (++Mem.NoExUl>2) Mem.NoExUl=0;
               sprintf(sptext,"Only working in chrono modus\nDisplay is %s", Mem.NoExUl==1?"Extreme":Mem.NoExUl==2?"Ultimate":"Normal" );
              }
            break;    
   case 'F':
   case 'f':
            if(InputString.length() == 1)
              {
               Mem.FiboChrono = !Mem.FiboChrono;
               sprintf(sptext,"Display is %s", Mem.FiboChrono?"Fibonacci":"Chrono" );
              }
            break;
 */
    case 'L':                                 // Lowest value for Brightness
    case 'l':    
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Lower brightness: %d bits",Mem.LowerBrightness);
               }
             break;                      
            
    case 'M':                                                                                 // factor to multiply brighness (0 - 255) with 
    case 'm':
            if (InputString.length() < 6)
               {    
                Mem.UpperBrightness = SConstrainInt(InputString,1,1,1023);
                sprintf(sptext,"Upper brightness changed to: %d bits",Mem.UpperBrightness);
               }
              break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 5 )
              {
               Mem.TurnOffLEDsAtHH = SConstrainInt(InputString,1,3,0,23);
               Mem.TurnOnLEDsAtHH  = SConstrainInt(InputString,3,5,0,23); 
               sprintf(sptext,"LEDs OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
              }
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
               }
             break;
    case 'P':
    case 'p':
             if (InputString.length() != 2) break;
             Mem.DisplayPalette = InputString.substring(1).toInt();
             sprintf(sptext,"Palette: %d",Mem.DisplayPalette);
             break;
    case 'I':
    case 'i':
             if (InputString.length() >1) break;   
             SWversion();
             sptext[0]=0;
             break;    
    case 'R':
    case 'r':
             if (InputString.length() >1) break;   
             Reset();                                                // Reset to default values
             sprintf(sptext,"Reset done");
             break;
    case 'S':
    case 's':
             if (InputString.length() >1) break;   
             Selftest();                                                // Reset to default values
             sprintf(sptext,"Selftest done");
             break;             
    case 'T':
    case 't':
             if(InputString.length() >= 7)  // T125500
              {              
              Ihour   = (byte) SConstrainInt(InputString,1,3,0,23);
              Iminute = (byte) SConstrainInt(InputString,3,5,0,59); 
              Isecond = (byte) SConstrainInt(InputString,5,7,0,59); 
              sprintf(sptext,"Time set");
              SetRTCTime();
              }
              break;         
    case 'W':
    case 'w':
             if (InputString.length() >1) break;   
             TestLDR = 1 - TestLDR;                                 // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR: %s",TestLDR? "On" : "Off");
             break;
//    case 'X':
//    case 'x':    
//            Demo = 1 - Demo;                                         // Toggle Demo mode
//            //Play_Lights();
//            GetTijd(0);  
//            Displaytime();
//            sprintf(sptext,"Demo: %s",Demo? "On" : "Off");
//            break;
    case 'Y':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'y':
            if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Slope brightness changed to: %d%%",Mem.LightReducer);
               }
              break;  
    case '0':
    case '1':
    case '2':        
           if (InputString.length() == 6 )                                // For compatibility input with only the time digits
             {
              Ihour   = (byte) SConstrainInt(InputString,0,2,0,23);
              Iminute = (byte) SConstrainInt(InputString,2,4,0,59); 
              Isecond = (byte) SConstrainInt(InputString,4,6,0,59);
              sprintf(sptext,"Time set");  
              SetRTCTime();
               }
    default:
            break;
   }
}
 Tekstprintln(sptext);
 Displaytime();
 EEPROM.put(0,Mem);                                                  // Update EEPROM    
 InputString = "";                                            
}

//--------------------------------------------
//  EEPROM DS3231 Read NVRAM at I2C address 0x57
//  The size of the NVRAM is 32 bytes
//--------------------------------------------
template <class T> int DS3231NVRAMWrite(int EEPROMaddress, const T& value)
{
 const byte* p = (const byte*)(const void*)&value;
 unsigned int i,x;
 for (x=0; x< sizeof(value)/32 +1; x++)   // Write in blocks of 32 bytes
  {
  Wire.beginTransmission(0x57);
  Wire.write((int)(EEPROMaddress >> 8));      // MSB
  Wire.write((int)(EEPROMaddress & 0xFF));    // LSB
  for (i = 0; i < 32; i++)   Wire.write(*p++);
  Wire.endTransmission();
  delay(5);
//  sprintf(sptext,"Size:%d writeP: %d x:%d",sizeof(value), p, x); Tekstprintln(sptext);    
  }
return i;
}
//--------------------------------------------
//  EEPROM DS3231 Write NVRAM at I2C address 0x57
//  The size of the NVRAM is 32 bytes
//--------------------------------------------

template <class T> int DS3231NVRAMRead(int EEPROMaddress, T& value)
{
 byte* p = (byte*)(void*)&value;
 unsigned int i,x;
 for (x=0; x< sizeof(value)/32 +1; x++)   // Read in blocks of 32 bytes
  {
   Wire.beginTransmission(0x57);
   Wire.write((int)(EEPROMaddress >> 8));     // MSB
   Wire.write((int)(EEPROMaddress & 0xFF));   // LSB
   Wire.endTransmission();
   Wire.requestFrom(0x57,32);                 
   for (i = 0; i<32; i++) {if(Wire.available()) *p++ = Wire.read(); }
   delay(5);
//   sprintf(sptext,"Size:%d readP; %d x:%d",sizeof(value), p, x); Tekstprintln(sptext); 
  }
return i;
}
