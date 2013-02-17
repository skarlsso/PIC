// Helper functions to communicate over I2C.

#ifndef I2C_HELPER_H
#define	I2C_HELPER_H

#include "xc.h"

#include <i2c.h>

// Current implementation waits in spin loops.
// TODO: Add support for interrupt based waits.
// TODO: Add some error handling. The current implementation
//       hangs if the slaves don't answer as expected.

void i2c_start_and_wait() {
    StartI2C();
    // Wait till Start sequence is completed
    while (I2CCONbits.SEN);
    // Clear interrput flag
    IFS0bits.MI2CIF = 0;
}

void i2c_restart_and_wait(void) {
    RestartI2C();
    while (I2CCONbits.RSEN);
    IFS0bits.MI2CIF = 0;
}

void i2c_stop_and_wait() {
    // Stop the data write
    StopI2C();
    // Wait till stop sequence is completd
    while (I2CCONbits.PEN);
}

void i2c_write_and_wait_no_ack(unsigned char value) {
    MasterWriteI2C(value);
    while (I2CSTATbits.TBF);
    while (!IFS0bits.MI2CIF);
    IFS0bits.MI2CIF = 0;
}

void i2c_write_and_wait(unsigned char value) {
    MasterWriteI2C(value);
    while (I2CSTATbits.TBF);
    while (!IFS0bits.MI2CIF);
    IFS0bits.MI2CIF = 0;
    while (I2CSTATbits.ACKSTAT);
}

unsigned char i2c_read_and_wait(void) {
    unsigned char c = MasterReadI2C();
    AckI2C();
    while (I2CCONbits.ACKEN);
    return c;
}

// TODO: EEPROM specific. Move this.
void i2c_ack_poll(unsigned char value) {
    i2c_start_and_wait();
    i2c_write_and_wait_no_ack(value);

    while (I2CSTATbits.ACKSTAT) {
        i2c_restart_and_wait();
        i2c_write_and_wait_no_ack(value);
    }

    i2c_stop_and_wait();
}

#endif	/* I2C_HELPER_H */

