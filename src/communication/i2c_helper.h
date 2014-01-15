// Helper functions to communicate over I2C.

#ifndef I2C_HELPER_H
#define	I2C_HELPER_H

#include "xc.h"

#include <i2c.h>

#ifdef _MI2CIF
#define StartI2C_       StartI2C
#define RestartI2C_     RestartI2C
#define StopI2C_        StopI2C
#define MasterWriteI2C_ MasterWriteI2C
#define MasterReadI2C_  MasterReadI2C
#define AckI2C_         AckI2C
#define NotAckI2C_      NotAckI2C
#define MI2CIF_         _MI2CIF
#define I2CCONbits_     I2CCONbits
#define I2C1STATbits_   I2CSTATbits
#elif defined(_MI2C1IF)
#define StartI2C_       StartI2C1
#define RestartI2C_     RestartI2C1
#define StopI2C_        StopI2C1
#define MasterWriteI2C_ MasterWriteI2C1
#define MasterReadI2C_  MasterReadI2C1
#define AckI2C_         AckI2C1
#define NotAckI2C_      NotAckI2C1
#define MI2CIF_         _MI2C1IF
#define I2CCONbits_     I2C1CONbits
#define I2CSTATbits_    I2C1STATbits
#endif

// Current implementation waits in spin loops.
// TODO: Add support for interrupt based waits.
// TODO: Add some error handling. The current implementation
//       hangs if the slaves don't answer as expected.

#ifndef I2C_ERROR
#define I2C_ERROR(message)
#endif

void i2c_start_and_wait() {
    StartI2C_();
    // Wait till Start sequence is completed
    while (I2CCONbits_.SEN);

    if (I2CSTATbits_.BCL) {
        I2C_ERROR("Bus collision in i2c_start_and_wait\n");
    }
    // Clear interrput flag
    MI2CIF_ = 0;
}

void i2c_restart_and_wait(void) {
    RestartI2C_();
    while (I2CCONbits_.RSEN);
    MI2CIF_ = 0;
}

void i2c_stop_and_wait() {
    // Stop the data write
    StopI2C_();
    // Wait till stop sequence is completd
    while (I2CCONbits_.PEN);


    while (!MI2CIF_);
    MI2CIF_ = 0;

    if (I2CSTATbits_.BCL) {
        I2C_ERROR("Bus collision in i2c_stop_and_wait\n");
    }
}

void i2c_write_and_wait_no_ack(unsigned char value) {
    char ret = MasterWriteI2C_(value);
    if (ret == -1) {
        I2C_ERROR("MasterWriteI2C failed\n");
    }
    while (I2CSTATbits_.TBF);
    while (!MI2CIF_);
    MI2CIF_ = 0;
}

void i2c_write_and_wait(unsigned char value) {
    char ret = MasterWriteI2C_(value);
    if (ret != 0) {
        I2C_ERROR("MasterWriteI2C failed\n");
    }
    while (I2CSTATbits_.TBF);
    while (!MI2CIF_);
    MI2CIF_ = 0;
    while (I2CSTATbits_.ACKSTAT);
}

unsigned char i2c_read_and_wait(int last_byte) {
    unsigned char c = MasterReadI2C_();
    if (last_byte) {
        NotAckI2C_();
    } else {
        AckI2C_();
    }
    while (I2CCONbits_.ACKEN);

    while (!MI2CIF_);
    MI2CIF_ = 0;

    if (I2CSTATbits_.BCL) {
        I2C_ERROR("Bus collision in i2c_read_and_wait\n");
    }

    return c;
}

// TODO: EEPROM specific. Move this.
void i2c_ack_poll(unsigned char value) {
    i2c_start_and_wait();
    i2c_write_and_wait_no_ack(value);

    while (I2CSTATbits_.ACKSTAT) {
        i2c_restart_and_wait();
        i2c_write_and_wait_no_ack(value);
    }

    i2c_stop_and_wait(99);
}

#endif	/* I2C_HELPER_H */

