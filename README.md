# FlasherX
Over-the-air firmware updates for Teensy LC/3.x/4.x/MicroMod

FlasherX is a merge of Flasher3 and Flasher4 by Jon Zeeff, with some new features and updates.
It works for Teensy LC, 3.2, 3.5, 3.6, 4.0, and 4.1, with a few caveats. I tested 3 transfer methods for each Teensy:

    1) hex file transfer via Serial (USB) using terminal emulator
    2) hex file transfer via Serial1 (UART) using terminal emulator
    3) block file transfer via Serial1 (UART) using custom client

My main objective was to support #3, transfer via custom client, so everything common to the 3 transfer methods
is in file FlashTxx.cpp/h, and FlasherX.ino is so far specific to hex file transfer via USB or UART. My platform
was TeensyDuino 1.53 and Arduino 1.8.13 on Windows 7 Pro 64-bit, and I used TeraTerm for the file transfers.

For method #1, hex file transfer via USB, I found that the reliability of transfer was improved if each hex line
was echoed back by the Teensy, while for method #2, hex file transfer via UART, the transfers were most reliable
without this echo. I don't know why this is the case, perhaps Paul or someone else can explain it.

All of my testing with Serial1 was done at 115200 baud.

For Teensy LC, reliable updates via file transfer required a 1 ms delay after each hex line. TeraTerm supports delays.

For Teensy 3.6, clock frequency had to be reduced to 120 MHz for reliable file transfers. With the custom client the
T3.6 is okay at 180 MHz.

FlasherX buffers the new code in flash by default, and automatically defines the largest possible flash buffer, from
the top of existing code to the bottom of FLASH_RESERVE (settable in FlashTxx.h). After moving the new code from the
buffer to the program flash, the flash buffer is erased. This leaves the flash in the same state as if the code was
uploaded via TeensyDuino. If the new code to be sent via FlasherX is smaller than the available buffer, the update
can be done in one step. If the new code is larger than the available flash, a two-step process can be used. Step 1
would be to send a minimal application with "Flasher" capability, and step2 would be to send the new, larger application.

    |<------------------------------ FLASH_SIZE ------------------------------>|
    ^FLASH_BASE_ADDR
    |<------- code ------->|<--------- buffer ---------->|<-- FLASH_RESERVE -->|

For T4.x, new code can optionally be buffered in RAM by setting macro RAM_BUFFER_SIZE in FlashTxx.h to a value > 0. For
my testing I used RAM_BUFFER_SIZE (256*1024).

To switch between Serial USB and Serial1 (or any UART), just change the "serial" variable definition in FlasherX.ino.
This variable, along with buffer_addr and buffer_size, are arguments to update_firmware(). update_firmware() is
substantially rewritten from Flasher3/4, but does the same things. There is hex_info_t data structure, which I think
makes it easier to see what's going on and what we need to keep track of during the file transfer. When the file transfer
is complete, the number of lines received is displayed and the user is prompted to enter that value to trigger the udpate,
or enter 0 to cancel.

For transfers via custom clients, these functions in FlashTxx.cpp provide the necessary API within your Teensy application:

    flash_buffer_init -- determine the address and size of the flash buffer
    flash_write_block -- write each received "block" of new code to the flash buffer
    flash_check_id -- confirm that the new code was built for the intended target
    flash_move -- move the (complete) new code from buffer to program flash
    flash_buffer_free -- erase the flash buffer or free the RAM buffer in the event of error/abort

The flash write/erase primitives for all Teensy are also in FlashTxx.cpp, conditionally compiled by target. As in Flasher4,
the write/erase functions are taken from Teensy4 core file eeprom.c. For Flasher3, the write/erase functions are based on
Frank Boesing's KinetisFlash module, with changes required to keep interrupts disabled during flash_move(). Note that
kinetis_hsrun_disable/enable are called in flash_exec(), which I found was necessary to avoid baud rate issues with T3.6.
Again, why T3.6 is different isn't clear to me.
