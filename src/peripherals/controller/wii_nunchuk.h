#ifndef WII_NUNCHUK_H
#define	WII_NUNCHUK_H

#include "communication/i2c_helper.h"
#include "time/delayer.h"

#define NUNCHUK_READ_BYTES 6

void read_bytes_from_nunchuk(unsigned char bytes[NUNCHUK_READ_BYTES]) {
    int i;

    i2c_start_and_wait();
    i2c_write_and_wait(0xA5);
    for (i = 0; i < NUNCHUK_READ_BYTES; i++) {
        bytes[i] = i2c_read_and_wait(i == NUNCHUK_READ_BYTES - 1);
    }
    i2c_stop_and_wait();
}

void init_nunchuk(unsigned char data[NUNCHUK_READ_BYTES]) {
    // Init nunchuk
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
    read_bytes_from_nunchuk(data);

#ifdef HAS_UART
    // Print the Device ID
    printf("Device ID: ");
    int i;
    for (i = 0; i < NUNCHUK_READ_BYTES; i++) {
        printf("%X ", data[i]);
    }
    printf("\n");
    fflush(stdout);
#endif
}

typedef struct Nunchuk {
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
} Nunchuk;


Nunchuk bytes_to_nunchuk(unsigned char bytes[NUNCHUK_READ_BYTES]) {
    Nunchuk n;
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

Nunchuk read_data_from_nunchuk(unsigned char bytes[NUNCHUK_READ_BYTES]) {
    i2c_start_and_wait();
    i2c_write_and_wait(0xA4);
    i2c_write_and_wait(0x00);
    i2c_stop_and_wait();

    delay_100us();
    delay_100us();

    read_bytes_from_nunchuk(bytes);
    return bytes_to_nunchuk(bytes);
}

#ifdef HAS_UART
void print_nunchuk(Nunchuk* this) {
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

// === Nunchuk limits ===

static const Nunchuk NUNCHUK_FULL_RESOLUTION_MIN = {
    {0, 0},
    {0, 0, 0},
    {0, 0}
};

static const Nunchuk NUNCHUK_FULL_RESOLUTION_MAX = {
    {255, 255},
    {1023, 1023, 1023},
    {1, 1}
};

static const Nunchuk NUNCHUK_HAND_EDITED_MIN = {
    {30, 30},
    {300, 300, 300},
    {0, 0}
};

static const Nunchuk NUNCHUK_HAND_EDITED_MAX = {
    {225, 225},
    {700, 700, 700},
    {1, 1}
};

// The adaptive limit values will be updated when new limits are seen.

static Nunchuk nunchuk_adaptive_min = {
    {127, 127},      // Joystick
    {511, 511, 511}, // Accelerometer
    {0, 0}           // Buttons
};

static Nunchuk nunchuk_adaptive_max = {
    {127, 127},      // Joystick
    {511, 511, 511}, // Accelerometer
    {0, 0}           // Buttons
};

#define MIN_SET(variable, value) \
    if (value < variable) {      \
        variable = value;        \
    }
#define NUNCHUK_MIN_SET(variable, value)                     \
    MIN_SET(variable.joystick.x,      value->joystick.x);      \
    MIN_SET(variable.joystick.y,      value->joystick.y);      \
    MIN_SET(variable.accelerometer.x, value->accelerometer.x); \
    MIN_SET(variable.accelerometer.y, value->accelerometer.y)

#define MAX_SET(variable, value) \
    if (value > variable) {      \
        variable = value;        \
    }
#define NUNCHUK_MAX_SET(variable, value)                     \
    MAX_SET(variable.joystick.x,      value->joystick.x);      \
    MAX_SET(variable.joystick.y,      value->joystick.y);      \
    MAX_SET(variable.accelerometer.x, value->accelerometer.x); \
    MAX_SET(variable.accelerometer.y, value->accelerometer.y)


static void nunchuk_adapt_limits(Nunchuk* nunchuk) {
    NUNCHUK_MIN_SET(nunchuk_adaptive_min, nunchuk);
    NUNCHUK_MAX_SET(nunchuk_adaptive_max, nunchuk);
}

#define NUNCHUK_LIMITS_ADAPTIVE        0
#define NUNCHUK_LIMITS_HAND_EDITED     1
#define NUNCHUK_LIMITS_FULL_RESOLUTION 2

static unsigned int nunchuk_limits_type = NUNCHUK_LIMITS_HAND_EDITED;

static int nunchuk_uses_adaptive_limits() {
    return nunchuk_limits_type == NUNCHUK_LIMITS_ADAPTIVE;
}

static Nunchuk const * nunchuk_requested_min() {
    switch (nunchuk_limits_type) {
        case NUNCHUK_LIMITS_ADAPTIVE:
            return &nunchuk_adaptive_min;
        case NUNCHUK_LIMITS_HAND_EDITED:
            return &NUNCHUK_HAND_EDITED_MIN;
        case NUNCHUK_LIMITS_FULL_RESOLUTION:
        default:
            return &NUNCHUK_FULL_RESOLUTION_MIN;
    }
}

static Nunchuk const * nunchuk_requested_max() {
    switch (nunchuk_limits_type) {
        case NUNCHUK_LIMITS_ADAPTIVE:
            return &nunchuk_adaptive_max;
        case NUNCHUK_LIMITS_HAND_EDITED:
            return &NUNCHUK_HAND_EDITED_MAX;
        case NUNCHUK_LIMITS_FULL_RESOLUTION:
        default:
            return &NUNCHUK_FULL_RESOLUTION_MAX;
    }
}

// Create a copy of 'nunchuk' that are limited by the current limit strategy.
static Nunchuk nunchuk_create_limited(Nunchuk* nunchuk) {
    // Copy values.
    Nunchuk n = *nunchuk;

    // And ensure the values are within the limits.

    Nunchuk const * min = nunchuk_requested_min();
    NUNCHUK_MAX_SET(n, min);

    Nunchuk const * max = nunchuk_requested_max();
    NUNCHUK_MIN_SET(n, max);

    return n;
}

// Create a copy of 'limited_nunchuk', which is limited by the current limit
// strategy, and scale the values to use the full resolution of the Nunchuk.
static Nunchuk nunchuk_create_scaled(Nunchuk* limited_nunchuk) {
    // Copy values.
    Nunchuk n = *limited_nunchuk;

    Nunchuk const * min = nunchuk_requested_min();
    Nunchuk const * max = nunchuk_requested_max();

#define FULL_SPAN(measurement)  (NUNCHUK_FULL_RESOLUTION_MAX.measurement - NUNCHUK_FULL_RESOLUTION_MIN.measurement)
#define LIMIT_SPAN(measurement) (max->measurement - min->measurement)
#define OFFSET(measurement)     (n.measurement - min->measurement)
#define SCALED(measurement)     LIMIT_SPAN(measurement) == 0 ? 0 : (unsigned int)((unsigned long) FULL_SPAN(measurement) * OFFSET(measurement) / LIMIT_SPAN(measurement))
#define SET_SCALED(measurement) n.measurement = SCALED(measurement)

    SET_SCALED(accelerometer.x);
    SET_SCALED(accelerometer.y);
    SET_SCALED(joystick.x);
    SET_SCALED(joystick.y);
    // Ignore the buttons

    return n;
}



#endif // WII_NUNCHUK_H

