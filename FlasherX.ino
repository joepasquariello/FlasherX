//******************************************************************************
// FlasherX -- firmware OTA update via Intel Hex file over serial or SD stream
//******************************************************************************
//
// Based on Flasher3 (Teensy 3.x) and Flasher4 (Teensy 4.x) by Jon Zeeff
//
// Jon Zeeff 2016, 2019, 2020 This code is in the public domain.
// Please retain my name in distributed copies, and let me know about any bugs
//
// I, Jon Zeeff, give no warranty, expressed or implied for this software and/or
// documentation provided, including, without limitation, warranty of
// merchantability and fitness for a particular purpose.
//
// WARNING: You can brick your Teensy with incorrect flash erase/write, such as
// incorrect flash config (0x400-40F). This code may or may not prevent that.

// 10/09/22 (v2.3) JWP - option for reading hex file from serial or SD
//   - move hex file support functions to new file FXUtil.cpp
//   - update_firmware() now takes two Stream* arguments ("in" and "out")
//   - FlasherX.ino lets user choose between hex file via serial or SD
// 09/01/22 (v2.2) JWP - change FlashTxx from CPP to C file
//   - rename FlashTxx.cpp to FlashTxx.c (resolve link error when calling from C)
//   - FlasherX.ino place #include "FlashTxx.h" inside extern "C" block
// 01/07/22 (v2.1) JWP - use TD 1.56 core functions for T4x wait/write/erase
//   - FlashTxx.h update FLASH_SIZE for Teensy Micromod from 8 to 16 MB
//   - option to artificially increase code size via const array (in flash)
// 11/18/21 JWP - bug fix in file FlashTXX.cpp
//   - fix logic in while loop in flash_block_write() in FlashTXX
// 10/27/21 JWP - add support for Teensy Micromod
//   - define macros for TEENSY_MICROMOD w/ same values as for TEENSY40
//   - update FLASH_SIZE for T4.1 and TMM from 2MB to 8MB
// JWP - merge of Flasher3/4 and new features
//   - FLASH buffer dynamically sized from top of existing code to FLASH_RESERVE
//   - optional RAM buffer option for T4.x via macro RAM_BUFFER_SIZE > 0
//   - Stream* (USB or UART) and buffer addr/size set at run-time
//   - incorporate Frank Boesing's FlashKinetis routines for T3.x
//   - add support for Teensy 4.1 and Teensy LC
//    This code is released into the public domain.
// JWP - Joe Pasquariello - modifications for T3.5 and T3.6 in Dec 2020
//    This code is released into the public domain
// Deb Hollenback at GiftCoder -- Modifications for teensy 3.5/3/6
//    This code is released into the public domain.
//    see https://forum.pjrc.com/threads/43165-Over-the-Air-firmware-updates-changes-for-flashing-Teensy-3-5-amp-3-6
// Jon Zeeff modifications
//    see https://forum.pjrc.com/threads/29607-Over-the-air-updates
// Original by Niels A. Moseley, 2015.
//    This code is released into the public domain.
//    https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/

#include <SD.h>
#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
  #include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

const int cs = BUILTIN_SDCARD;	// SD chip select pin
const int led = LED_BUILTIN;	// LED pin
Stream *serial = &Serial;	// Serial (USB) or Serial1, Serial2, etc. (UART)

#define FLASHERX_VERSION "FlasherX v2.3"
#define HEX_FILE_NAME "FlasherX.ino.hex"	

#define LARGE_ARRAY (0)		// 1 = define large array to test large hex file

#if (LARGE_ARRAY)
// nested arrays of integers to add code size for testing 
#define A0 { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}  // 16  elements 64
#define A1 {A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0}  // 256 elements 1KB 
#define A2 {A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1}  // 4K  elements 16KB
#define A3 {A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2}  // 64K elements 256KB 
#define A4 {A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3}  // 1M  elements 4MB

// const variables reside in flash and get optimized out if never accessed
// use uint8_t -> 1MB, uint16_t -> 2MB, uint32_t -> 4MB, uint64_t -> 8MB)
PROGMEM const uint8_t a[16][16][16][16][16] = A4;
#endif

void setup () 
{
  if (serial == (Stream*)&Serial) {
    Serial.begin(115200);
    while (!Serial) {} 
  }
  else {
    ((HardwareSerial*)serial)->begin( 115200 );
  } 
  
  pinMode( led, OUTPUT );	// assign output
  digitalWrite( led, HIGH );	// set the LED on
  delay(200);			// delay
  digitalWrite( led, LOW );	// set the LED off

  serial->printf( "%s - %s %s\n", FLASHERX_VERSION, __DATE__, __TIME__ );
  serial->printf( "WARNING: this can ruin your device!\n" );
  serial->printf( "target = %s (%dK flash in %dK sectors)\n",
			FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);
			
#if (LARGE_ARRAY) // if true, access array so it doesn't get optimized out
  serial->printf( "Large Array -- %08lX\n", (uint32_t)&a[15][15][15][15][15] );
#endif
}

void loop ()
{
  uint32_t buffer_addr, buffer_size;

  // create flash buffer to hold new firmware
  if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
    serial->printf( "unable to create buffer\n" );
    serial->flush();
    for (;;) {}
  }
  
  serial->printf( "created buffer = %1luK %s (%08lX - %08lX)\n",
		buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
		buffer_addr, buffer_addr + buffer_size );

  // get user input to read from serial or SD
  int user_input = -1;
  char line[32];
  while (user_input != 1 && user_input != 2) {
    serial->printf( "enter 1 for hex file via serial, 2 for hex file via SD\n" );
    read_ascii_line( serial, line, sizeof(line) );
    sscanf( line, "%d", &user_input );
  }
  
  if (user_input == 1) { // serial 
    // read hex file, write new firmware to flash, clean up, reboot
    update_firmware( serial, serial, buffer_addr, buffer_size );
  }
  else if (user_input == 2) { // SD
    if (!SD.begin( cs )) {
      serial->println( "SD initialization failed" );
      return;
    }
    File hexFile;
    serial->println( "SD initialization OK" );
    hexFile = SD.open( HEX_FILE_NAME, FILE_READ );
    if (!hexFile) {
      serial->println( "SD file open failed" );
      return;
    }
    serial->println( "SD file open OK" );
    // read hex file, write new firmware to flash, clean up, reboot
    update_firmware( &hexFile, serial, buffer_addr, buffer_size );
  }
  
  // return from update_firmware() means error or user abort, so clean up and
  // reboot to ensure that static vars get boot-up initialized before retry
  serial->printf( "erase FLASH buffer / free RAM buffer...\n" );
  firmware_buffer_free( buffer_addr, buffer_size );
  serial->flush();
  REBOOT;
}
