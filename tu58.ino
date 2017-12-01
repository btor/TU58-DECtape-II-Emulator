/*
  TU58 DECtape II emulator 

  This is a standalone extended version of the TU58 emulator of William T. Kranz.
  This port was developed using his tu58.c version 1.04 files with his permission.
  Most of the code is his work, see changes in the history.

  Written by: William T. Kranz (Version 1.0x for DOS) & Bela Torok (Version 2.0, Windows, DOS and standalone port)

  Contact via: 

  William T. Kranz, http://www.willsworks.net/contact.htm (version 1.x-specific code)
  Version 1.x for DOS is still available from http://www.willsworks.net/pdp11/rt11arc.exe


  Bela Torok, http://www.torok.info & http://www.belatorok.com
  (Version 2.x code for Windows, DOS and Arduino and serial library for Windows and DOS)

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation version 2 of the 
  License (http://www.gnu.org/licenses/gpl.txt).

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.


  Hardware requirements:
  
  Boards tested: Serial Arduino, Arduino Duemilanove, Mega1280, Leonardo
 
  MicroSD card breakout board, or Adafruit GPS-shield without GPS-receiver shield, Adafruit Data logging shield
  2x16 LCD modul with custom-made keyboard, or Arduino LCD-shield. 
  
  SD-Card:
  A SD-card breakout board or a shield with SD-card must be installed, e.g., the Adafruit GPS-shield without
  GPS-receiver or the Adafruit Data logging shield.
 
  Breakout board connections (a breakout board with 5V to 3.3V active level converter needed):
  The best way to conect the SD card to arduino is to use the 6 pin ICSP connector which is available on all modern arduino boards.
  The CS (chip select) terminal on the breakout board should be connected to Arduino pin 10. 
  
  Connection diagram:
  ICSP Pin     SD Breakout board signal name    
  
  1 (MISO)     DO (Data output)
  2 (+VCC)     +5V
  3 (SCK)      CLK
  4 (MOSI)     DI (Data input)
  6 (Gnd)      GND
               CS  -> connect to Arduino pin 10 
  5 (Reset)    Is not Connected to the Beakout board!!!
  
  
  Alternative SD card connection possibilities (not recommended, since hardware specific):  
  For the serial Arduino, Arduino Duemilanove, Uno and other Atmega328-based boards: 
  connect GND to ground, 5V to 5V, CLK to pin 13, DO to pin 12, DI to pin 11, and CS to pin 10.
  
  For the Arduino Mega1280 or Mega2560:
  connect GND to ground, 5V to 5V, CLK to pin 52, DO to pin 50, DI to pin 51, and CS to pin 53.
 
  Software requirements:
  Adruino software (http://www.arduino.cc is available for Windows, MacOS X and Linux)
  This verson was tested with Version 1.0.5-r2 and 1.5.6-r2
   
  To keep as much RAM free as possible I tried optimize the code to by eliminating variables
  and subroutine calls. With the current version of the Arduino software and sdfatlib library 
  about 380 bytes free RAM are needed for error-free operation.
  
  The current version of the program was tested with RT11 5.3 running on 
  a PDP11/23, PDP11/53 and on different simulated PDP11s on Ersatz 11 (E11).
  
  For safe operation with @38400 Baud and LCD display "Delay" should be adjusted to
  >= 17 ms on PDP11/23
  >=  8 ms on PDP11/53
  
  Delay required with E11 PDP11 emulator
  >=  0 ms with Ersatz 11 Tested with Windows 7 and 8.1 64 bit, 2.4 GHz dual and quadcore CPUs, UART: 16550 PCI or PCIe
  >  16 ms with Ersatz 11 Tested with Windows 7 and 8.1 64 bit, 2.4 GHz dual and quadcore CPUs, UART: FTDI USB-to Serial converter
  >  16 ms with Ersatz 11 Tested with Windows 7 32 bit, dualore 1.2 GHz CPU, UART: FTDI USB-to Serial converter

  Known issue:
  For some reason hard boot does not work with Ersatz11 on a 2.53 GHz quadcore Xeon with Windows 7 64 bit. 
   
*/

// Start hardware specific definitions
// Enable the following line if a LCD display + Buttons conncted to the analog Input 0 are connected to Arduino
#define LCD

// Enable the following line if a VT100 compatible serial terminal is used to monitor activity and settup purposes
// Use this setting only with Arduinos with multiple Serial lines, e.g., with Leonardo or Mega 1280 or 2560 
//#define VT100

// Warning: Due to limited RAM capacity on Leonardo do not use #define LCD and #define VT100 simultaeneously!

// Define the chip select line for the SD card
#define SD_CS 10  

// LEDs showing Red/Write activity
// Connect cathodes to ground, anode with one 1 kOhm resistor in series to pin 2 (green LED) and 3 (red LED)
#define WRITE_LED 3
#define READ_LED 2

// End hardware specific definitions

#ifdef LCD
#include <LiquidCrystal.h>
#endif

#include <EEPROM.h>

#include <SD.h>
#include <SPI.h> // required for Arduino >= 1.5.4

#include "tu58.h"

#define MODE_BAUD         0
#define MODE_DELAY        1
#define MODE_PROTOCOL     2
#define MODE_D0_FILENAME  3
#define MODE_D1_FILENAME  4
#define MODE_WRITE_EEPROM 5
#define MODE_EXIT         6
#define NUM_MODES         7
#define TIMEOUT -1

#define ANALOG_INPUT 0

File TU58File;

const long BaudRate[5] = {9600, 19200, 38400, 57600, 115200};
const char FilenameBeg[] = {"TU58-"};
const char FilenameEnd[] = {".dsk"};

// The following variables will be updated from the EEPROM
unsigned char Baud = 0;
unsigned char TU58Delay = 0;
unsigned char MrspEnable = 0;
unsigned char Filename[2];

unsigned char data[BLOCK_SIZE]; // global data packet

#ifdef VT100
 #define BUF_LENGTH 60
#else
 #define BUF_LENGTH 20
#endif

char buf[BUF_LENGTH];

union {
  unsigned char data[sizeof(RSP_CMD)];      // command packet size
  RSP_CMD cmd;
} command;
 
// global flags
unsigned char VerboseMode = 1; // 0 -> off, 1 -> on

unsigned char BreakReceived = 1;

unsigned char Switches = 0;

int Flag, LastFlag = 0;

char XoffReceived = 0;

#ifdef LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
#endif

void setup()
{
  pinMode(ANALOG_INPUT, INPUT);
  pinMode(WRITE_LED, OUTPUT);
  pinMode(READ_LED, OUTPUT);

  digitalWrite(WRITE_LED, LOW);
  digitalWrite(READ_LED, LOW);

#ifdef LCD
  // set up the LCD's number of columns and rows: 
  lcd.begin(16,2);
  lcd.clear();
#endif

#if (defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)) 
  #ifdef LCD
    pinMode(10, OUTPUT);
    digitalWrite(10, 1);  // turn lcd backlight on
  #endif
  #define SD_CS 53  
#else
  #define SD_CS 10  
//  #define SD_CS 4
#endif
  pinMode(SD_CS, OUTPUT);
  if (!SD.begin(SD_CS))
  {
  #ifdef LCD 
    lcd.clear();
    lcd.print("Card init Error!");
  #endif

  #ifdef VT100
    Serial.print("\r\nCard init Error!");
  #endif
    while(1) {};
  }
  
  Baud        = EEPROM.read(MODE_BAUD);
  TU58Delay   = EEPROM.read(MODE_DELAY);
  MrspEnable  = EEPROM.read(MODE_PROTOCOL);
  Filename[0] = EEPROM.read(MODE_D0_FILENAME);
  Filename[1] = EEPROM.read(MODE_D1_FILENAME);
  
  if(Filename[0] > 99) Filename[0] = 0;
  if(Filename[1] > 99) Filename[1] = 0;

#if (defined(__AVR_ATmega32U4__))
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
#endif

#if (defined(__AVR_ATmega32U4__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  Serial.begin(115200);
  // TU58 port
  Serial1.begin(BaudRate[Baud]);
#else
  Serial.begin(BaudRate[Baud]);
#endif    

 
#ifdef LCD
  lcd.clear();
  lcd.print("RSP");
  if(MrspEnable == 0)
  {
    lcd.print(" only");
  } else {
    lcd.print("/MRSP");
  }
  lcd.setCursor(0, 1);
  lcd.print("Delay = "); lcd.print(TU58Delay, DEC); lcd.print(" ms");
  delay(2000);
  
  lcd.clear();
  lcd.print(BaudRate[Baud], DEC); lcd.print(" Baud");
  
  lcd.setCursor(0, 1);
  lcd.print(FreeRam(), DEC); lcd.print(" Bytes free");
#endif

#ifdef VT100
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print("TU58 Dectape II emulator");
  Serial.println();
  
  Serial.print("\r\nRSP");
  if(MrspEnable == 0)
  {
    Serial.print(" only");
  } else {
    Serial.print("/MRSP");
  }
  Serial.print("\r\nDelay = "); Serial.print(TU58Delay, DEC); Serial.print(" ms");
  Serial.println();
  Serial.print(BaudRate[Baud], DEC); Serial.print(" Baud");
  
  Serial.println();
  Serial.print(FreeRam(), DEC); Serial.println(" Bytes free");
  Serial.flush();
#endif
}

#ifdef VT100
void SetupMenuVT100()
{
#define ESCAPE 27
  const char *ModeName[7] = {"Baudrate        ", 
                             "Delay           ",
                             "Protocol        ",
                             "D0 Filename     ",
                             "D1 Filename     ",
                             "Write to EEPROM?",
                             "Exit setup      "}; 

  unsigned char key, mode;
  char unit = 0;

  Serial.flush();
  mode = -1;

  Serial.print("\x1b[?25l"); // cursor off (25 + lowercase L

  // print direction for use on terminal
  Serial.print("\x1b[H"); //cursor home
  Serial.print("\x1b[J"); //clear screen
  Serial.print("Setup TU58 parameters. TU58 is now offline!\r\n\r\n");
  Serial.print("Press");
  Serial.print("<SPACE> to select parameter\r\n");
  Serial.print("+ or - to change its value\r\n");
  Serial.print("S to save parameter in EEPROM\r\n\r\n");
  Serial.print("Escape to exit\r\n");    
        
  while(1)
  {  
    key = 0;
    if(Serial.available() > 0)
    {
      key = Serial.read();
    }

    if(mode == -1 ) key = ' ';  //first run
    
    if(key == ESCAPE) 
    {
      Serial.print("\x1b[H"); //cursor home
      Serial.print("\x1b[J"); //clear screen
      Serial.print("Setup is terminated. TU58 is online!\r\n");
//BreakReceived = 0;  // force reinit
//Serial1.flush();
//Tu58Putc(CONTINUE);
      return; // timeout after 3 seconds
    }
    
    if(key > 0) 
    {
        
      if(key == ' ')
      {
        mode++;
        if(mode >= NUM_MODES - 2) mode = 0;
//        Serial.print("\x1b[9;0H"); // cursor -> x,y = 0, 10
//        Serial.print("\x1b[K"); // clear line
//        Serial.print(ModeName[mode]);
      }
      
      if(key == '+' || key == '-' || key == ' ')
      {
        Serial.print("\x1b[9;0H"); // cursor -> x,y = 0, 9
        Serial.print("\x1b[K"); // clear line

        switch (mode)
        {
        case MODE_BAUD:
          if(key == '+') Baud++;
          if(key == '-') 
          {
            Baud--;
            if(Baud == 255) Baud = 4;
          }
          if(Baud > 4) Baud = 0;
          sprintf(buf, "Baudrate = %ld ", BaudRate[Baud]);
          Serial.print(buf);
          break;  
        case MODE_DELAY:
          if(key == '+') TU58Delay++;
          if(key == '-') TU58Delay--;
          sprintf(buf, "Delay = %3d ms ", TU58Delay);
          Serial.print(buf);
          break;
        case MODE_PROTOCOL:
          if(key == '+') MrspEnable++;
          if(key == '-') MrspEnable--;
          MrspEnable &= 1; // value is 0 or 1
          if(MrspEnable == 0)
          {
            sprintf(buf, "Protocol: RSP only ");
          } else {
            sprintf(buf, "Protocol: RSP/MRSP ");
          }
          Serial.print(buf);
          break;
        case MODE_D0_FILENAME:
        case MODE_D1_FILENAME:
          unit = 0;
          if(mode == MODE_D1_FILENAME) unit = 1;
          if(key == '+') Filename[unit]++;
          if(key == '-') Filename[unit]--;
          if(Filename[unit] == 255) Filename[unit] = 99;
          if(Filename[unit] > 99) Filename[unit] = 0;
if(TU58File) TU58File.close();
          sprintf(buf, "%s%02d%s", FilenameBeg, Filename[unit], FilenameEnd);
          TU58File = SD.open(buf, FILE_READ);
          sprintf(buf, "Tape%d filename:  %s%02d%s ", unit, FilenameBeg, Filename[unit], FilenameEnd);
          if(!TU58File)
          {
            strcat(buf, " Unable to open! ");
          } else {
            TU58File.close();
            strcat(buf, " OK ");
          }      
          Serial.print(buf);
          break;
        } // end switch(mode)
      }
      
      if(key == 's' || key == 'S')
      {
        if(Baud        != EEPROM.read(MODE_BAUD))        EEPROM.write(MODE_BAUD, Baud);
        if(TU58Delay   != EEPROM.read(MODE_DELAY))       EEPROM.write(MODE_DELAY, TU58Delay);
        if(MrspEnable  != EEPROM.read(MODE_PROTOCOL))    EEPROM.write(MODE_PROTOCOL, MrspEnable);
        if(Filename[0] != EEPROM.read(MODE_D0_FILENAME)) EEPROM.write(MODE_D0_FILENAME, Filename[0]);
        if(Filename[1] != EEPROM.read(MODE_D1_FILENAME)) EEPROM.write(MODE_D1_FILENAME, Filename[1]);
        Serial.print("EEPROM Written  ");
//        delay(1000);
      }
    
    if(key != ' ' && key != '+' && key != '-' && key == 's' && key == 'S') Serial.write(7);
    
    }
  }
}
#endif

#ifdef LCD
void SetupMenuLCD()
{
#define KEY_INCREASE      1
#define KEY_DECREASE      2
#define KEY_SELECT_MODE   3
#define KEY_ENTER         4
#define KEY_NOT_PRESSED   127

  const char *ModeName[7] = {"Baudrate        ", 
                             "Delay           ",
                             "Protocol        ",
                             "D0 Filename     ",
                             "D1 Filename     ",
                             "Write to EEPROM?",
                             "Exit setup      "}; 
  unsigned char key, last_key = KEY_NOT_PRESSED, mode, YesNo = 0;
  int AnalogIn;
  char unit, inc_dec = 0;

  mode = 0;
  lcd.setCursor(0, 0);
  lcd.print(ModeName[mode]);
     
  while(1)
  {  
    // wait for a button
    do{
      AnalogIn = analogRead(ANALOG_INPUT);
      delay(10);
//    } while(AnalogIn != analogRead(ANALOG_INPUT));   // wait until reading does not change
    } while(abs(AnalogIn - analogRead(ANALOG_INPUT)) > 8);   // wait until reading does not change

    key = KEY_NOT_PRESSED;
    
    if(AnalogIn < 896) key = KEY_INCREASE;
    if(AnalogIn < 640) key = KEY_DECREASE;
    if(AnalogIn < 384) key = KEY_ENTER;
    if(AnalogIn < 128) key = KEY_SELECT_MODE;

    last_key = key;

    do
    {
      AnalogIn = analogRead(ANALOG_INPUT);      
      delay(10);
    } while(AnalogIn < 900); // Key released ?
                
      if(key == KEY_SELECT_MODE)
      {
        mode++;
        if(mode >= NUM_MODES) mode = 0;
        lcd.setCursor(0, 0);
        lcd.print(ModeName[mode]);
      }
      
      lcd.setCursor(0, 1);


      if(key == KEY_INCREASE || key == KEY_DECREASE || key == KEY_SELECT_MODE)
      {
        inc_dec = 0;
        if(key == KEY_INCREASE) inc_dec = 1;
        if(key == KEY_DECREASE) inc_dec = -1;
        
        switch(mode)
        {
        case MODE_BAUD:
          Baud += inc_dec;
          if(Baud == 255) Baud = 4;
          if(Baud > 4) Baud = 0;
          sprintf(buf, "%6ld          ", BaudRate[Baud]);
          lcd.print(buf);
          break;  
        case MODE_DELAY:
          TU58Delay += inc_dec;
          sprintf(buf, "%3d ms          ", TU58Delay);
          lcd.print(buf);
          break;
        case MODE_PROTOCOL:
          MrspEnable += inc_dec;
          MrspEnable &= 1; // value is 0 or 1
          if(MrspEnable == 0)
          {
            lcd.print("RSP only        ");
          } else {
            lcd.print("RSP/MRSP        ");
          }
          break;
        case MODE_D0_FILENAME:
        case MODE_D1_FILENAME:
          unit = 0;
          if(mode == MODE_D1_FILENAME) unit = 1;
          Filename[unit] += inc_dec;
          if(Filename[unit] == 255) Filename[unit] = 99;
          if(Filename[unit] > 99) Filename[unit] = 0;
if(TU58File) TU58File.close();
          sprintf(buf, "%s%02d%s", FilenameBeg, Filename[unit], FilenameEnd);
          TU58File = SD.open(buf, FILE_READ);
          if(!TU58File)
          {
            sprintf(&buf[11], " Err!");
          } else {
            TU58File.close();
            sprintf(&buf[11], " OK  ");
          }      
          lcd.print(buf);
          break;
        case MODE_WRITE_EEPROM:
        case MODE_EXIT:
          YesNo += inc_dec;
          YesNo &= 1;
          if(YesNo == 0)
          {
            lcd.print("No              ");
          } else {
            lcd.print("Yes             ");
          }
          break;
        } // end switch(mode)
      }
      if(mode == MODE_WRITE_EEPROM && YesNo == 1 && key == KEY_ENTER)
      {
        if(Baud        != EEPROM.read(MODE_BAUD))        EEPROM.write(MODE_BAUD, Baud);
        if(TU58Delay   != EEPROM.read(MODE_DELAY))       EEPROM.write(MODE_DELAY, TU58Delay);
        if(MrspEnable  != EEPROM.read(MODE_PROTOCOL))    EEPROM.write(MODE_PROTOCOL, MrspEnable);
        if(Filename[0] != EEPROM.read(MODE_D0_FILENAME)) EEPROM.write(MODE_D0_FILENAME, Filename[0]);
        if(Filename[1] != EEPROM.read(MODE_D1_FILENAME)) EEPROM.write(MODE_D1_FILENAME, Filename[1]);
        lcd.setCursor(0, 1);
        lcd.print("EEPROM Written  ");
        delay(1000);
        mode = MODE_EXIT; YesNo = 0;
        lcd.setCursor(0, 0);
        lcd.print(ModeName[mode]);
        lcd.setCursor(0, 1);
        lcd.print("No              ");       
      }
      if(mode == MODE_EXIT && YesNo == 1 && key == KEY_ENTER)
      {
        lcd.clear();
        lcd.print("TU58 RSP");
        if(MrspEnable == 0)
        {
          lcd.print(" only");
        } else {
          lcd.print("/MRSP");
        }
 
//        BreakReceived = 0;  // force reinit
        //Tu58Putc(CONTINUE);
        return;
      }
  }
}
#endif

int SerialGetc(void)
{
  unsigned long StartTime;

  StartTime = millis();

#if (defined(__AVR_ATmega32U4__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  while(Serial1.available() == 0) {
    if((millis() - StartTime) > 1000) return TIMEOUT; // timeout after 1000 ms
  }
  return (int) Serial1.read();
#else
  while(Serial.available() == 0) {
    if((millis() - StartTime) > 1000) return TIMEOUT; // timeout after 1000 ms
  }
  return (int) Serial.read();
#endif
}

void SerialClearRxBuffer()
{
#if (defined(__AVR_ATmega32U4__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  while(Serial1.available() > 0)
  {
    Serial1.read();
  }
#else
  while(Serial.available() > 0)
  {
    Serial.read();
  }
#endif
}

void Tu58Putc(unsigned char h)
{
  int SerialRxCh;

  if(MrspEnable != 0 && (Switches & MRSP_SWITCH ))
  {
    do
    {
      SerialRxCh = SerialGetc();
      if(SerialRxCh < 0) return; 
    } while(SerialRxCh != CONTINUE);
delay(TU58Delay); // required to pass the ZTUUF0 test with the MRSP protocol on the E11 Emulator   
  }

#if (defined(__AVR_ATmega32U4__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  Serial1.write(h);
  Serial1.flush();
#else
  Serial.write(h);
  Serial.flush();
#endif
}

unsigned int CalculateChecksum(unsigned char Flag, unsigned char NumBytes, unsigned char *DataBytes)
{
  // 16 bit arithmetic
  unsigned char i;
  unsigned int PrevChecksum;

  union {
    unsigned int  Word;
    unsigned char Byte[2];
  } Checksum;
  
  Checksum.Byte[0] = Flag;
  Checksum.Byte[1] = NumBytes;
  
  PrevChecksum = Checksum.Word;  
  
  for(i = 0; i < NumBytes; i++) {
    if((i & 1) == 0) {	          // even index
      Checksum.Word += DataBytes[i];
    } else {                      // odd index
      Checksum.Byte[1] += DataBytes[i];
    }
    
    if(Checksum.Word < PrevChecksum) Checksum.Word++; // add carry 

    PrevChecksum = Checksum.Word;  
  }
 
  return Checksum.Word;
}

int rsp_get_packet(unsigned char *MessageByteCount, unsigned char *DataBytes)
{
  unsigned char i;
  int Flag, SerialRxCh;

  union {
    unsigned int  Word;
    unsigned char Byte[2];
  } Checksum;

  Flag = SerialGetc();
  
if(Flag < 0) return FAIL;  // timeout      

  switch(Flag)
  {
    case DATA:
    case CONTROL:
      SerialRxCh = SerialGetc();
      if(SerialRxCh < 0) return FAIL;
      *MessageByteCount = SerialRxCh;
      if(*MessageByteCount > BLOCK_SIZE) return FAIL;
      
      for(i = 0; i < *MessageByteCount; i++)
      {
        SerialRxCh = SerialGetc();
        if(SerialRxCh < 0) return FAIL;
        DataBytes[i] = SerialRxCh;
      }

      // get checksum from host
      SerialRxCh = SerialGetc();  // get low Byte
      if(SerialRxCh < 0) return FAIL;
      Checksum.Byte[0] = SerialRxCh;
      
      SerialRxCh = SerialGetc();  // get high Byte
      if(SerialRxCh < 0) return FAIL;
      Checksum.Byte[1] = SerialRxCh;

      if(Checksum.Word != CalculateChecksum(Flag, *MessageByteCount, DataBytes)) 
      {
        return BAD_CHECK;
      }
      return Flag;
    case BOOT:
      SerialRxCh = SerialGetc();
      if(SerialRxCh < 0) return FAIL;
      *MessageByteCount = SerialRxCh;  // set to drive #
      return Flag;
  }
  return Flag; // single Byte flags, e.g., INIT, CONTINUE and XOFF
} 

/* send arbitrary packet with length bytes 
   size effect, clears byte at cnt+1 is cnt is odd
*/
void rsp_snd_packet(unsigned char type,    /* INIT for End packet, CONTROL for command packet */
                    unsigned char cnt,     /* number of bytes of data */
                    unsigned char *data)
{
  unsigned char i; 
  unsigned int Checksum;

  command.cmd.modifier = OK; // Success Code
  
  Tu58Putc(type);
  Tu58Putc(cnt);

  for(i = 0; i < cnt; i++) {      
    Tu58Putc(data[i]);
  }

  Checksum = CalculateChecksum(type, cnt, data);

  /* send checksum as word, ie two low order bytes */
  Tu58Putc(lowByte(Checksum));
  Tu58Putc(highByte(Checksum));

  return;
}

void EndPacket(unsigned char suc, unsigned int count)
{
  command.cmd.opcode = END_CMD;
  command.cmd.modifier = suc;
//  command.cmd.unit; // keep it
//  command.cmd.switches; // keep it
  command.cmd.sequence = 0;
  command.cmd.count = count;
  command.cmd.block = 0;  // == Summary Status
  if(suc != 0) command.cmd.block |= 0x8000; // always set special condition bit


  if(VerboseMode == 1) {
  #ifdef LCD
    lcd.setCursor(0,0);
    lcd.print(" ");
//    lcd.print("Idle             ");
//    lcd.setCursor(0,1);
//    lcd.print("                 ");
  #endif
  #ifdef VT100
    Serial.print("\x1b[H"); //cursor home
    Serial.println("                                     ");  
    Serial.print(" "); Serial.flush(); // delete the first character in the next line
  #endif
  }

//  delay(TU58Delay);

  rsp_snd_packet(CONTROL, sizeof(RSP_CMD), command.data);
}

/* special case, ignore RSP and stream 512 bytes of boot sector, block 0 */
void rsp_snd_boot(unsigned char unit)
{
  unsigned char i, j, ch;

  if(TU58File) TU58File.close();
  
  // open for read
  sprintf(buf, "%s%02d%s", FilenameBeg, Filename[unit], FilenameEnd);
  TU58File = SD.open(buf, FILE_READ);

//  if (!TU58File) return NO_TAPE;

  if(TU58File.seek(0L) == false) return;
  
  for(i = 0; i < 4; i++)
  {
    for(j = 0; j < BLOCK_SIZE; j++)
    {
      Tu58Putc(TU58File.read());
    }
#ifdef LCD
    lcd.print(".");
#endif

#ifdef VT100
    Serial.print("."); Serial.flush();
#endif
delay(100);
  }
  
  return;
}

void rsp_process_cmd(void)
{
  int flag, ch;
  char suc;
  unsigned char bytes, i;
  unsigned long offset;          // byte offset in data file
  unsigned short count; /* let it be a long so can go negative */

  suc = 0;
  
  count = command.cmd.count;

  switch(command.cmd.opcode)
  {
  case POSITION_CMD:
  case READ_CMD:
  case WRITE_CMD:
    offset = (unsigned long) command.cmd.block;

    if(command.cmd.modifier & 0x80) // special address mode
    {
      offset = offset * 128; // mini block size
    } else {
      offset = offset * 512; // std block size
    }
    
    if(TU58File) TU58File.close();
    // open for read and write
    sprintf(buf, "%s%02d%s", FilenameBeg, Filename[command.cmd.unit], FilenameEnd);
    TU58File = SD.open(buf, FILE_WRITE);

    if(!TU58File) 
    {
      // empty cartridge only error for above
      EndPacket(NO_TAPE, 0);
      return;
    }

    // seek error
    if(TU58File.seek(offset) == false)
    {
      EndPacket(BAD_SEEK, 0);
      return;
    }
    
    delay(TU58Delay);

    break;
  }


  switch(command.cmd.opcode)
  {
  case READ_CMD:
    digitalWrite(READ_LED, HIGH);
    while(count > 0)
    {     
      if(count > BLOCK_SIZE) 
      {
        bytes = BLOCK_SIZE;
      } else {
        bytes = count;
      }
      
      count -= bytes;

      for(i = 0; i < bytes; i++)
      {
        ch = TU58File.read();
        if(ch == -1)
        {
//          suc = NO_TAPE; 
          digitalWrite(READ_LED, LOW);
          EndPacket(NO_TAPE, 0);
          return; // no byte available
        }
        data[i] = ch;
      }

      if(i != bytes)
      {
        if(VerboseMode > 1) 
        {
        #ifdef LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Read error!");
        #endif
        #ifdef VT100
          Serial.print("\nRead error!"); Serial.flush();
        #endif
        }
        digitalWrite(READ_LED, LOW);
        EndPacket(FAIL, 0);
        return; // no byte available
      }
      
      rsp_snd_packet(DATA, bytes, data);
      delay(TU58Delay);
    }
    // send end packet
    EndPacket(OK, command.cmd.count);
    digitalWrite(READ_LED, LOW);
    return;
  break;
  
  case WRITE_CMD:
    digitalWrite(WRITE_LED, HIGH);
    while(count > 0)
    {
      Tu58Putc(CONTINUE);

      do 
      {
	flag = rsp_get_packet(&bytes, data);

	if(flag == INIT && LastFlag == INIT) {
          LastFlag = 0;
          SerialClearRxBuffer();
//          Tu58Putc(CONTINUE);
          digitalWrite(WRITE_LED, LOW);
          return;
        }
        
        if(flag == BAD_CHECK || flag == FAIL)
        {
          EndPacket(flag, 0);
          digitalWrite(WRITE_LED, LOW);
          return;
        }

        LastFlag = flag;
      } while(flag != DATA);


      count -= bytes;
              
      if(bytes < BLOCK_SIZE) 
      {
        memset(&data[bytes], 0, BLOCK_SIZE - bytes);  // zero fill the unused part of the array if the number of bytes < BLOCK_SIZE
      }
      
      if(TU58File.write(data, BLOCK_SIZE) != BLOCK_SIZE) // write block (128 Bytes)
      {
        EndPacket(NO_TAPE, 0);
        digitalWrite(WRITE_LED, LOW);
        return;
      } 
      delay(TU58Delay);
    } 
    // goto send end packet
    EndPacket(OK, command.cmd.count);
    digitalWrite(WRITE_LED, LOW);
    return;
    break;
  
  case GETCHAR_CMD:
    delay(TU58Delay);
    if(MrspEnable == 0)
    { 
      // Drives without MRSP capability send a DATA packet
//      memset(data, 0, 24);   // not necessary !!
      rsp_snd_packet(DATA, 24, data);
    } else {
      // MRSP capable drives send an End Packet
      EndPacket(0, 0);
    }
    return;
    break;

  case INIT_CMD:      // reset & send end packet
  if(TU58File) TU58File.close();
//    if(MrspEnable != 0) Switches = cmd->switches;
    Switches = command.cmd.switches;
    LastFlag = 0;
    XoffReceived = 0;
  case NOP_CMD:       // request end packet 
  case GETSTATUS_CMD:     // get status, sends end packet
  case SETSTATUS_CMD:     // set status, sends end packet
  case DIAGNOSE_CMD:  // run internal diagnostic and send end packet
delay(TU58Delay);
    suc = 0;
    break;
  default:
    suc = BAD_OPCODE;
  }

  EndPacket(suc, 0);
 
  return;
}

void loop()                     // run over and over again
{ 
  unsigned char len;

#if (defined(__AVR_ATmega32U4__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  if(Serial1.available() == 0)
  {
  #ifdef VT100
    if(Serial.available() > 0) SetupMenuVT100();  
  #endif

  #ifdef LCD
    if(analogRead(ANALOG_INPUT) < 800) SetupMenuLCD();
  #endif    
  } 
#else
  if(Serial.available() == 0)
  {
  #ifdef LCD
    if(analogRead(ANALOG_INPUT) < 800) SetupMenuLCD();
  #endif    
  } 
#endif
  //  wait for a command packet
  Flag = rsp_get_packet(&len, command.data);
  switch(Flag)
  {
    case TIMEOUT:
      if(BreakReceived == 0) Tu58Putc(INIT);
      break;
    case BOOT:
      if(len == 0 || len == 1)
      {
	if(VerboseMode) {
          sprintf(buf, "Booting unit %d  ", len);
        #ifdef LCD
          lcd.clear();
          lcd.print(buf);
          lcd.setCursor(0,1);
        #endif
        #ifdef VT100
          Serial.println();
          Serial.print(buf); Serial.flush();
        #endif
        }
        digitalWrite(READ_LED, HIGH);  
        rsp_snd_boot(len);
        digitalWrite(READ_LED, LOW);  
      } /* else {
        sprintf(buf, "Boot Err unit %d ", len);
        lcd.clear();
        lcd.print(buf);
      } */
      break;
    case INIT: /* just send the continue */
      if(LastFlag == INIT) /* two in a row */
      {
      XoffReceived = 0;
//BreakReceived = 1;
        Switches = 0; // clear MRSP switch
        SerialClearRxBuffer();
//if(TU58File) TU58File.close();
	if(VerboseMode == 1) {
        #ifdef LCD
          lcd.setCursor(0, 0);
          lcd.print(" Continue after ");
          lcd.setCursor(0, 1);
          lcd.print(" 2 INIT flags.  ");
        #endif
        #ifdef VT100
          Serial.print("\x1b[?25l"); // cursor off (25 + lowercase L
          Serial.print("\x1b[H"); //cursor home
          Serial.print("Continue after 2 INIT flags."); Serial.flush();
        #endif
        }       
	Tu58Putc(CONTINUE);
	Flag = -1; // set LastFlag to undefined
      }
      break;
    case DATA:
      SerialClearRxBuffer();
/*      lcd.setCursor(0, 0);
      lcd.print("Data packet out ");
      lcd.setCursor(0, 1);
      lcd.print("of sequence..   ");
*/
      Tu58Putc(INIT);
      Tu58Putc(INIT);
      Flag = 0; // force resync
      break;
    case CONTROL:
      if(VerboseMode == 1) 
      {
        switch(command.cmd.opcode)
        {
          case POSITION_CMD:
          case READ_CMD:      
          case WRITE_CMD:
          #ifdef LCD
            sprintf(buf, "   U%d %5d Byte", command.cmd.unit, command.cmd.count);
            if(command.cmd.opcode == READ_CMD)  buf[0] = 'R';
            if(command.cmd.opcode == WRITE_CMD) buf[0] = 'W';
            lcd.setCursor(0, 0);
            lcd.print(buf);
            sprintf(buf, "Block %5d \xf2 %02d", command.cmd.block, Filename[command.cmd.unit]);
            lcd.setCursor(0, 1);
            lcd.print(buf);
          #endif
          #ifdef VT100
            Serial.print("\x1b[?25l"); // cursor off (25 + lowercase L
            Serial.print("\x1b[2;0H"); // cursor -> x,y = 0, 2
//            Serial.print("\x1b[H"); //cursor home
            sprintf(buf, "  Block %5d, %5d Bytes, ", command.cmd.block, command.cmd.count);
            if(command.cmd.opcode == READ_CMD)  buf[0] = 'R';
            if(command.cmd.opcode == WRITE_CMD) buf[0] = 'W';
            Serial.print(buf);
            sprintf(buf, "Tape %d, File: %s%02d%s", command.cmd.unit, FilenameBeg, Filename[command.cmd.unit], FilenameEnd);
            Serial.print(buf);
            Serial.flush();
          #endif
          ; // this line is needed if both LCD and VT100 are not defined
        }
      }
      
      if(command.cmd.unit == 0 || command.cmd.unit == 1) {
        rsp_process_cmd();
      } /*else {
        lcd.setCursor(0, 0);
	sprintf(buf, "Invalid unit: %d ", cmd->unit);
        lcd.print(buf);
        lcd.setCursor(0, 1);
        lcd.print("in command.     ");
      } */
      break;
    case BAD_CHECK:
      // command packet with illegal checksum
      SerialClearRxBuffer();
      if(VerboseMode) {
      #ifdef LCD
        lcd.clear();
        lcd.print("Checksum error! ");
      #endif
      #ifdef VT100
        Serial.print("\r\nChecksum error!"); Serial.flush();
      #endif
        // critical error
        // keep message on the display by disbaling further VerboseMode information        
//        VerboseMode = 0;
      }
//      EndPacket(BAD_SEEK, 0);    
      Flag = -1; // clear INIT LastFlag
      break;
    case CONTINUE:
    case XON:
      XoffReceived = 0;
      break;   
    case XOFF:
      XoffReceived = 1;
      break;
    default: // Other flags are treated as BREAKs
      BreakReceived = 1;
      break;
  }
//  if(Flag > 0) LastFlag = Flag;
  LastFlag = Flag;
} // end loop


