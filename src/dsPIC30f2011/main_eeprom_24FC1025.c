#include <xc.h>

#include "dsPIC30f2011/lm16a211_config.h"

#include "communication/i2c_helper.h"
#include "debug/led_debug.h"
#include "peripherals/eeprom/24FC1025.h"
#include "peripherals/lcd/lm16a211.h"
#include "time/delayer.h"

#include <stdio.h>
#include <string.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */

void example0(void);
void example1(void);

int main(void) {
    lcd_init();

    // Setup I2C
    // =========

    // I2C clock and data
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;

    // Baud rate
    // FCY     = 29840000
    // FCL     = 400000
    // config2 = (FCY/FSCL - FCY/1111111) - 1;
    unsigned int config2 = 47;
    // I2C in 7 bit address mode
    unsigned int config1 = (I2C_ON
            & I2C_IDLE_CON
            & I2C_CLK_HLD
            & I2C_IPMI_DIS
            & I2C_7BIT_ADD
            & I2C_SLW_DIS
            & I2C_SM_DIS
            & I2C_GCALL_DIS
            & I2C_STR_DIS
            & I2C_NACK
            & I2C_ACK_DIS
            & I2C_RCV_DIS
            & I2C_STOP_DIS
            & I2C_RESTART_DIS
            & I2C_START_DIS);
    OpenI2C(config1, config2);
    IdleI2C();

    //example0();
    example1();
    
    CloseI2C();

    while (1);

    return 0;
}

void example0(void) {
   // chip select: 2 bits addresses up to four 24FC1025s
    char chip_select = 0;
    // block bit: 0 - first 512K bits, 1 - second 512K bits
    char block = 0;
    // Bit address within the selected chip/block.
    int address = 0;


    // Write some data to the eeprom
    // =============================
    char write_data[] = "Stefan";
    int data_length = strlen(write_data);

    eeprom_24FC1025_write(chip_select, block, address, write_data, data_length);

    // Read the same data from the eeprom
    // ==================================
    char read_data[data_length];

    eeprom_24FC1025_read(chip_select, block, address, read_data, data_length);


    // Output the read data
    // ====================
    lcd_clear();
    int i;
    for (i = 0; i < data_length; i++) {
        lcd_send_char(read_data[i]);
    }
}

void example1(void) {
    char buffer[32];

#define STR_AND_LEN(str) str, strlen(str)

#define TO_STR(chip, block, addr) "chip" # chip ".block" # block ".addr" # addr

#define EEPROM_EXAMPLE_WRITE(chip, block, addr) \
    eeprom_24FC1025_write(chip, block, addr, STR_AND_LEN(TO_STR(chip, block, addr)))

#define EEPROM_EXAMPLE_READ(chip, block, addr) \
    eeprom_24FC1025_read(chip, block, addr, buffer, strlen(TO_STR(chip, block, addr))); \
    lcd_send_char('0' + strncmp(buffer, STR_AND_LEN(TO_STR(chip, block, addr))))

    EEPROM_EXAMPLE_WRITE(0, 0, 0);
    EEPROM_EXAMPLE_WRITE(0, 0, 0xA000);
    EEPROM_EXAMPLE_WRITE(0, 1, 0);
    EEPROM_EXAMPLE_WRITE(1, 0, 0);
    EEPROM_EXAMPLE_WRITE(2, 0, 0);

    EEPROM_EXAMPLE_READ(0, 0, 0);
    EEPROM_EXAMPLE_READ(0, 0, 0xA000);
    EEPROM_EXAMPLE_READ(0, 1, 0);
    EEPROM_EXAMPLE_READ(1, 0, 0);
    EEPROM_EXAMPLE_READ(2, 0, 0);

    // Expected output on lcd: 00000
}