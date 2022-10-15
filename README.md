# FlasherX

OTA (over-the-air) firmware updates for Teensy LC/3.x/4.x/MicroMod.

FlasherX is a merge of Flasher3 and Flasher4 by Jon Zeeff, with updates to work with Teensy LC, 3.2, 3.5, 3.6, 4.0, 4.1, and Micromod.  The update method is to transfer the new firmware to the Teensy via Serial (USB or UART), SD, or any other available interface, buffer the new firmware in unused flash, erase the existing firmware, copy the new firmware from the buffer to the flash base address, erase the buffer, and reboot to run the new firmware.

These transfer methods have been tested so far:

    1) Intel hex file via Serial (USB) using TeraTerm terminal emulator
    2) Intel hex file via Serial1 (UART) using TeraTerm terminal emulato
    3) Binary block file via USB or UART using custom client
    4) Intel hex file on built-in SD card (T3.5, T3.6, T4.1)

The source files are:

    1) FlashTxx.c/h     functions to erase/read/write flash (independent of transfer method)
    2) FXUtils.cpp/h    Intel hex file read via stream and Intel hex record parsing
    3) FlasherX.ino     example program to update via Intel hex file from USB, UART, or SD
    
Notes on my testing:

    - original development test w/ Arduino 1.8.13, TeensyDuino 1.53 on Windows 7, TeraTerm terminal emulator.
    - latest testing w/ Arduino 1.8.19, TeensyDuino 1.58b2
    - for USB Serial, reliability was better if each hex line was echoed back by the Teensy (not sure why)
    - for UART Serial, reliability was better without this echo
    - all of my UART testing on all platforms was done using Serial1 at 115200 baud
    - for Teensy LC, reliable updates via USB/UART required 1-ms delay after each hex line (using TeraTerm)
    - for Teensy 3.6, clock frequency must be reduced to 120 MHz for reliable USB and UART serial transfers

FlasherX buffers the new code in flash by default, and automatically defines the largest possible flash buffer, from
the top of existing code to the bottom of FLASH_RESERVE (settable in FlashTxx.h). After moving the new code from the
buffer to the program flash, the flash buffer is erased. This leaves the flash in the same state as if the code was
uploaded via TeensyDuino. If the new code to be sent via FlasherX is smaller than the available buffer, the update
can be done in one step. If the new code is larger than the available flash, a two-step process can be used.

    1) send a minimal application with "FlasherX" capability
    2) send the new, larger application

The diagram below shows the use of flash, with the low (base) address at the left. Code is executed from the flash base address. Space can be reserved at the top of flash for use by Teensy, LittleFS, or EEPROM emulation. The space between the existing firmware and the flash reserve area is available to buffer the new firmware.

    |<------------------------------ FLASH_SIZE ------------------------------>|
    ^FLASH_BASE_ADDR
    |<------- code ------->|<--------- buffer ---------->|<-- FLASH_RESERVE -->|

For T4.x, the buffer can optionally be placed in RAM by setting macro RAM_BUFFER_SIZE in FlashTxx.h to a value > 0. For my testing I used a 256Kb buffer by setting RAM_BUFFER_SIZE (256*1024).

In the FlasherX.ino file, choose the Serial port for hex file transfer by setting the "serial" variable to "Serial" for USB, or Serial1 (or any available hardware) for UART. The function update_firmware() in FXUtils.cpp takes two Stream* arguments. The first is a Stream* for the hex file input, and the second is a Stream* to the serial monitor for user i/o. The use of Stream* for input allows the same code to be used for hex file transfer via serial or via SD card. When the file transfer is complete, the number of lines read is displayed and the user is prompted to enter that value to trigger the udpate, or 0 to cancel the update.

For transfers via custom clients, these functions in FlashTxx.c/h provide this API:

    flash_buffer_init -- determine the address and size of the flash buffer
    flash_write_block -- write each received "block" of new code to the flash buffer
    flash_check_id    -- confirm that the new code was built for the intended target
    flash_move        -- move the (complete) new code from buffer to program flash
    flash_buffer_free -- erase the flash buffer or free the RAM buffer in the event of error/abort

FlashTxx also contains the flash erase/write functions for all Teensy 3.x. The erase/write functions for T4.x are in the Teensy4 core file eeprom.c, so those functions are used. The flash functions for T3.x are based on Frank Boesing's KinetisFlash module, with changes required to keep interrupts disabled during flash_move(). 

kinetis_hsrun_disable/enable are called in flash_exec(), which is necessary to avoid baud rate issues with T3.6.
