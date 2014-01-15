#ifndef WII_NUNCHUCK_H
#define	WII_NUNCHUCK_H

#include "communication/i2c_helper.h"
// TODO: Move delayer.h
#include "dsPIC30f2011/delayer.h"

#define NUNCHUCK_READ_BYTES 6

void read_bytes_from_nunchuck(unsigned char bytes[NUNCHUCK_READ_BYTES]) {
    int i;

    i2c_start_and_wait();
    i2c_write_and_wait(0xA5);
    for (i = 0; i < NUNCHUCK_READ_BYTES; i++) {
        bytes[i] = i2c_read_and_wait(i == NUNCHUCK_READ_BYTES - 1);
    }
    i2c_stop_and_wait();
}

void init_nunchuck(unsigned char data[NUNCHUCK_READ_BYTES]) {
    // Init nunchuck
    i2c_start_and_wait();
    i2c_write_and_wait(0xA4);
    i2c_write_and_wait(0xF0);
    i2c_write_and_wait(0x55);
    i2c_stop_and_wait();

    delay_ms(1);

    i2c_start_and_wait();
    i2c_write_and_wait(0xA4);
    i2c_write_and_wait(0xFB);
    i2c_write_and_wait(0x00);
    i2c_stop_and_wait();

    delay_ms(1);

    // Read the Device ID
    read_bytes_from_nunchuck(data);

#ifdef HAS_UART
    // Print the Device ID
    printf("Device ID: ");
    int i;
    for (i = 0; i < NUNCHUCK_READ_BYTES; i++) {
        printf("%X ", data[i]);
    }
    printf("\n");
    fflush(stdout);
#endif
}

typedef struct Nunchuck {
    struct {
        unsigned char x;
        unsigned char y;
    } joystick;
    // int since 10 bits resolution
    struct {
        unsigned int x : 10;
        unsigned int y : 10;
        unsigned int z : 10;
    } accelerometer;
    struct {
        unsigned char c;
        unsigned char z;
    } buttons;
} Nunchuck;


Nunchuck bytes_to_nunchuck(unsigned char bytes[NUNCHUCK_READ_BYTES]) {
    Nunchuck n;
    n.joystick.x      = bytes[0];
    n.joystick.y      = bytes[1];
    n.accelerometer.x = bytes[2] << 2;
    n.accelerometer.y = bytes[3] << 2;
    n.accelerometer.z = bytes[4] << 2;

    typedef union {
        unsigned char init_dummy;
        struct {
            unsigned int z  : 1;
            unsigned int c  : 1;
            unsigned int ax : 2;
            unsigned int ay : 2;
            unsigned int az : 2;
        } bits;
    } last_byte;

    last_byte lb = { bytes[5] };

    // Button bits are reversed.
    n.buttons.z        = !lb.bits.z;
    n.buttons.c        = !lb.bits.c;
    // Lower accelerometer bits
    n.accelerometer.x |= lb.bits.ax;
    n.accelerometer.y |= lb.bits.ay;
    n.accelerometer.z |= lb.bits.az;

    return n;
}

Nunchuck read_data_from_nunchuck(unsigned char bytes[NUNCHUCK_READ_BYTES]) {
    i2c_start_and_wait();
    i2c_write_and_wait(0xA4);
    i2c_write_and_wait(0x00);
    i2c_stop_and_wait();

    delay_100us();
    delay_100us();

    read_bytes_from_nunchuck(bytes);
    return bytes_to_nunchuck(bytes);
}

#ifdef HAS_UART
void print_nunchuck(Nunchuck* this) {
    printf("joystick X: %3d Y: %3d "
           "accelermoter X: %3d Y: %3d Z: %3d "
           "buttons c: %d z: %d",
            (int)this->joystick.x,
            (int)this->joystick.y,
            (int)this->accelerometer.x,
            (int)this->accelerometer.y,
            (int)this->accelerometer.z,
            (int)this->buttons.c,
            (int)this->buttons.z);
}
#endif

#endif // WII_NUNCHUCK_H

