#ifndef _24FC1025_H
#define _24FC1025_H

#include "i2c_helper.h"

#define HIGH_BYTE(address) ((unsigned char)((address) >> 8))
#define LOW_BYTE(address)  ((unsigned char)((address) & 0xFF))

#define I2C_24FC1024_CONTROL(chip_select, block) \
    0b10100000                      /* Hardcoded for 24FC1025 */ \
    | (((chip_select) & 0b11) << 1) /* Choose one of possibly four chips ... */ \
    | (((block)       &  0b1) << 3) /* ... and one of the two blocks. */


void eeprom_24FC1025_write(char chip_select, char block, unsigned int address, const char* data, int length) {
    char control_byte = I2C_24FC1024_CONTROL(chip_select, block);

    i2c_start_and_wait();

    // Write control byte.
    i2c_write_and_wait(control_byte | 0 /* eeprom write mode */);
     // Transmit memory address.
    i2c_write_and_wait(HIGH_BYTE(address));
    i2c_write_and_wait(LOW_BYTE(address));

    // Transmit data.
    int i;
    for (i = 0; i < length; i++) {
        i2c_write_and_wait(data[i]);
    }

    // Finalize data transmission.
    i2c_stop_and_wait();

    // Wait until write is completed.
    i2c_ack_poll(control_byte | 0);
}

void eeprom_24FC1025_read(char chip_select, char block, unsigned int address, char* data, int length) {
    char control_byte = I2C_24FC1024_CONTROL(chip_select, block);

    i2c_start_and_wait();

    // Write control byte.
    i2c_write_and_wait(control_byte | 0 /* eeprom write mode */);
    // Transmit data address.
    i2c_write_and_wait(HIGH_BYTE(address));
    i2c_write_and_wait(LOW_BYTE(address));

    i2c_restart_and_wait();

    // Change to eeprom read mode.
    i2c_write_and_wait(control_byte | 1 /* eeprom read mode */);

    int i;
    for (i = 0; i < length; i++) {
        // Read sequential data.
        data[i] = i2c_read_and_wait(i == length - 1);
    }

    // End the data read.
    i2c_stop_and_wait();
}

#endif // _24FC1025