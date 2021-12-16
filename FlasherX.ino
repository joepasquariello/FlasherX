//******************************************************************************
// FlasherX -- firmware "OTA" update via Intel Hex file over serial byte stream
//******************************************************************************
//
// WARNING: You can brick your Teensy with incorrect flash erase/write, such as
// incorrect flash config (0x400-40F). This code may or may not prevent that.
//
// Based on Flasher3 (Teensy 3.x) and Flasher4 (Teensy 4.x) by Jon Zeeff
//
// Jon Zeeff 2016, 2019, 2020
// This code is in the public domain.  Please retain my name and
// in distributed copies, and let me know about any bugs
//
// I, Jon Zeeff, give no warranty, expressed or implied for this software
// and/or documentation provided, including, without limitation, warranty of
// merchantability and fitness for a particular purpose.

// 6th Modifications 11/18/21 bug fix in file FlashTXX.cpp by Joe Pasquariello
//   - fix logic in while loop in flash_block_write() in FlashTXX
// 5th Modifications 10/27/21 add support for Teensy Micromod by Joe Pasquariello
//   - define macros for TEENSY_MICROMOD w/ same values as for TEENSY40
//   - update FLASH_SIZE for T4.1 and TMM from 2MB to 8MB
// 4th Modifications merge of Flasher3/4 and new features by Joe Pasquariello
//   - FLASH buffer dynamically sized from top of existing code to FLASH_RESERVE
//   - optional RAM buffer option for T4.x via macro RAM_BUFFER_SIZE > 0
//   - Stream* (USB or UART) and buffer addr/size set at run-time
//   - incorporate Frank Boesing's FlashKinetis routines for T3.x
//   - add support for Teensy 4.1 and Teensy LC
//    This code is released into the public domain.
// 3rd Modifications for T3.5 and T3.6 in Dec 2020 by Joe Pasquariello
//    This code is released into the public domain.
// 2nd Modifications for teensy 3.5/3/6 by Deb Hollenback at GiftCoder
//    This code is released into the public domain.
//    see https://forum.pjrc.com/threads/43165-Over-the-Air-firmware-updates-changes-for-flashing-Teensy-3-5-amp-3-6
// 1st Modifications by Jon Zeeff
//    see https://forum.pjrc.com/threads/29607-Over-the-air-updates
// Original by Niels A. Moseley, 2015.
//    This code is released into the public domain.
//    https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/

#include <Arduino.h>
#include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives

const int ledPin = 13;		// LED
Stream *serial = &Serial;	// Serial (USB) or Serial1, Serial2, etc. (UART)

//******************************************************************************
// hex_info_t	struct for hex record and hex file info
//******************************************************************************
typedef struct {	// 
  char *data;		// pointer to array allocated elsewhere
  unsigned int addr;	// address in intel hex record
  unsigned int code;	// intel hex record type (0=data, etc.)
  unsigned int num;	// number of data bytes in intel hex record
 
  uint32_t base;	// base address to be added to intel hex 16-bit addr
  uint32_t min;		// min address in hex file
  uint32_t max;		// max address in hex file
  
  int eof;		// set true on intel hex EOF (code = 1)
  int lines;		// number of hex records received  
} hex_info_t;

void read_ascii_line( Stream *serial, char *line, int maxbytes );
int  parse_hex_line( const char *theline, char *bytes,
	unsigned int *addr, unsigned int *num, unsigned int *code );
int  process_hex_record( hex_info_t *hex );
void update_firmware( Stream *serial, uint32_t buffer_addr, uint32_t buffer_size );

void setup () 
{
  if (serial == (Stream*)&Serial) {
    Serial.begin(115200);
    while (!Serial) {} 
  }
  else {
    ((HardwareSerial*)serial)->begin( 115200 );
  } 

  serial->printf( "\nFlasherX OTA firmware update v2 %s %s\n", __DATE__, __TIME__ );
  serial->printf( "WARNING: this can ruin your device\n" );
  serial->printf( "target = %s (%dK flash in %dK sectors)\n",
			FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);

  pinMode(ledPin, OUTPUT);	// assign output
  digitalWrite(ledPin, HIGH);	// set the LED on
  delay(200);			// delay
  digitalWrite(ledPin, LOW);	// set the LED off
}

void loop ()
{
  uint32_t buffer_addr, buffer_size;

  if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
    serial->printf( "unable to create buffer\n" );
    serial->flush();
    for (;;) {}
  }

  serial->printf( "buffer = %1luK %s (%08lX - %08lX)\n",
		buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
		buffer_addr, buffer_addr + buffer_size );

  // receive hex file via serial, write new firmware to flash, clean up, reboot
  update_firmware( serial, buffer_addr, buffer_size ); // no return if success
  
  // return from update_firmware() means error or user abort, so clean up and
  // reboot to ensure that static vars get boot-up initialized before retry
  serial->printf( "erase FLASH buffer / free RAM buffer...\n" );
  firmware_buffer_free( buffer_addr, buffer_size );
  serial->flush();
  REBOOT;
}

//******************************************************************************
// update_firmware()	read hex file and write new firmware to program flash
//******************************************************************************
void update_firmware( Stream *serial, uint32_t buffer_addr, uint32_t buffer_size )
{
  static char line[96];					// buffer for hex lines
  static char data[32] __attribute__ ((aligned (8)));	// buffer for hex data
  hex_info_t hex = {					// intel hex info struct
    data, 0, 0, 0,					//   data,addr,num,code
    0, 0xFFFFFFFF, 0, 					//   base,min,max,
    0, 0						//   eof,lines
  };

  serial->printf( "waiting for hex lines...\n" );

  // read and process intel hex lines until EOF or error
  while (!hex.eof)  {

    read_ascii_line( serial, line, sizeof(line) );
    // reliability of transfer via USB is improved by this printf/flush
    if (serial == (Stream*)&Serial) {
      serial->printf( "%s\n", line );
      serial->flush();
    }

    if (parse_hex_line( (const char*)line, hex.data, &hex.addr, &hex.num, &hex.code ) == 0) {
      serial->printf( "abort - bad hex line %s\n", line );
      return;
    }
    else if (process_hex_record( &hex ) != 0) { // error on bad hex code
      serial->printf( "abort - invalid hex code %d\n", hex.code );
      return;
    }
    else if (hex.code == 0) { // if data record
      uint32_t addr = buffer_addr + hex.base + hex.addr - FLASH_BASE_ADDR;
      if (hex.max > (FLASH_BASE_ADDR + buffer_size)) {
        serial->printf( "abort - max address %08lX too large\n", hex.max );
        return;
      }
      else if (!IN_FLASH(buffer_addr)) {
        memcpy( (void*)addr, (void*)hex.data, hex.num );
      }
      else if (IN_FLASH(buffer_addr)) {
        int error = flash_write_block( addr, hex.data, hex.num );
        if (error) {
          serial->printf( "abort - error %02X in flash_write_block()\n", error );
	  return;
        }
      }
    }
    hex.lines++;
  }
    
  serial->printf( "\nhex file: %1d lines %1lu bytes (%08lX - %08lX)\n",
			hex.lines, hex.max-hex.min, hex.min, hex.max );

  // check FSEC value in new code -- abort if incorrect
  #if defined(KINETISK) || defined(KINETISL)
  uint32_t value = *(uint32_t *)(0x40C + buffer_addr);
  if (value == 0xfffff9de) {
    serial->printf( "new code contains correct FSEC value %08lX\n", value );
  }
  else {
    serial->printf( "abort - FSEC value %08lX should be FFFFF9DE\n", value );
    return;
  } 
  #endif

  // check FLASH_ID in new code - abort if not found
  if (check_flash_id( buffer_addr, hex.max - hex.min )) {
    serial->printf( "new code contains correct target ID %s\n", FLASH_ID );
  }
  else {
    serial->printf( "abort - new code missing string %s\n", FLASH_ID );
    return;
  }
  
  // get user input to write to flash or abort
  int user_lines = -1;
  while (user_lines != hex.lines && user_lines != 0) {
    serial->printf( "enter %d to flash or 0 to abort\n", hex.lines );
    read_ascii_line( serial, line, sizeof(line) );
    sscanf( line, "%d", &user_lines );
  }
  
  if (user_lines == 0) {
    serial->printf( "abort - user entered 0 lines\n" );
    return;
  }
  
  // move new program from buffer to flash, free buffer, and reboot
  flash_move( FLASH_BASE_ADDR, buffer_addr, hex.max-hex.min );

  // should not return from flash_move(), but put REBOOT here as reminder
  REBOOT;
}

//******************************************************************************
// read_ascii_line()	read ascii characters until '\n', '\r', or max bytes
//******************************************************************************
void read_ascii_line( Stream *serial, char *line, int maxbytes )
{
  int c=0, nchar=0;
  while (nchar < maxbytes && !(c == '\n' || c == '\r')) {
    if (serial->available()) {
      c = serial->read();
      line[nchar++] = c;
    }
  }
  line[nchar-1] = 0;	// null-terminate
}

//******************************************************************************
// process_hex_record()		process record and return okay (0) or error (1)
//******************************************************************************
int process_hex_record( hex_info_t *hex )
{
  if (hex->code==0) { // data -- update min/max address so far
    if (hex->base + hex->addr + hex->num > hex->max)
      hex->max = hex->base + hex->addr + hex->num;
    if (hex->base + hex->addr < hex->min)
      hex->min = hex->base + hex->addr;
  }
  else if (hex->code==1) { // EOF (:flash command not received yet)
    hex->eof = 1;
  }
  else if (hex->code==2) { // extended segment address (top 16 of 24-bit addr)
    hex->base = ((hex->data[0] << 8) | hex->data[1]) << 4;
  }
  else if (hex->code==3) { // start segment address (80x86 real mode only)
    return 1;
  }
  else if (hex->code==4) { // extended linear address (top 16 of 32-bit addr)
    hex->base = ((hex->data[0] << 8) | hex->data[1]) << 16;
  }
  else if (hex->code==5) { // start linear address (32-bit big endian addr)
    hex->base = (hex->data[0] << 24) | (hex->data[1] << 16)
              | (hex->data[2] <<  8) | (hex->data[3] <<  0);
  }
  else {
    return 1;
  }

  return 0;
}

//******************************************************************************
// Intel Hex record foramt:
//
// Start code:  one character, ASCII colon ':'.
// Byte count:  two hex digits, number of bytes (hex digit pairs) in data field.
// Address:     four hex digits
// Record type: two hex digits, 00 to 05, defining the meaning of the data field.
// Data:        n bytes of data represented by 2n hex digits.
// Checksum:    two hex digits, computed value used to verify record has no errors.
//
// Examples:
//  :10 9D30 00 711F0000AD38000005390000F5460000 35
//  :04 9D40 00 01480000 D6
//  :00 0000 01 FF
//******************************************************************************

/* Intel HEX read/write functions, Paul Stoffregen, paul@ece.orst.edu */
/* This code is in the public domain.  Please retain my name and */
/* email address in distributed copies, and let me know about any bugs */

/* I, Paul Stoffregen, give no warranty, expressed or implied for */
/* this software and/or documentation provided, including, without */
/* limitation, warranty of merchantability and fitness for a */
/* particular purpose. */

// type modifications by Jon Zeeff

/* parses a line of intel hex code, stores the data in bytes[] */
/* and the beginning address in addr, and returns a 1 if the */
/* line was valid, or a 0 if an error occured.  The variable */
/* num gets the number of bytes that were stored into bytes[] */

#include <stdio.h>		// sscanf(), etc.
#include <string.h>		// strlen(), etc.

int parse_hex_line( const char *theline, char *bytes, 
		unsigned int *addr, unsigned int *num, unsigned int *code )
{
  unsigned sum, len, cksum;
  const char *ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':')
    return 0;
  if (strlen (theline) < 11)
    return 0;
  ptr = theline + 1;
  if (!sscanf (ptr, "%02x", &len))
    return 0;
  ptr += 2;
  if (strlen (theline) < (11 + (len * 2)))
    return 0;
  if (!sscanf (ptr, "%04x", (unsigned int *)addr))
    return 0;
  ptr += 4;
  /* Serial.printf("Line: length=%d Addr=%d\n", len, *addr); */
  if (!sscanf (ptr, "%02x", code))
    return 0;
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf (ptr, "%02x", &temp))
      return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256)
      return 0;
  }
  if (!sscanf (ptr, "%02x", &cksum))
    return 0;

  if (((sum & 255) + (cksum & 255)) & 255)
    return 0;     /* checksum error */
  return 1;
}
