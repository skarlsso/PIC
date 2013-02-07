#include "delayer.h"
#include "led_debug.h"

#include <p30f2011.h> 
#include <libpic30.h>

_FOSC(CSW_FSCM_OFF & FRC_PLL16); // FCY=29.48MHz & FOSC=16x7.37MHz  
_FWDT(WDT_OFF);                  // Watchdog timer off 
_FBORPOR(MCLR_DIS & PWRT_64);    // Disable reset pin & Power-up timer

#include "lm16a211.h"

static struct tagRCONBITS RCONbits_saved;

static void rcon_iopuwr_check() {
#if !NO_DEBUG
    // Have had problems where __delay32 resets with IOPUWR. Show the bit.
    if (RCONbits_saved.IOPUWR) {
        DEBUG_BIT_1 = 1;
    } else {
        DEBUG_BIT_1 = 0;
    }
#endif
}

static void init(void) {
    debug_init();

    // Save RCON for later debugging.
    RCONbits_saved = RCONbits;
    RCON = 0;
}

void example0(void);
void example1(void);
void example2(void);
void example3(void);

int main(void) {
    init();

    // The pin setup has been placed in PRE_LCD_INIT, which is called from lcd_init().
    lcd_init(); 

    rcon_iopuwr_check();    

    while (1) {
        //example0();
        //example1();
        //example2();
        example3();
    }

    return 0;
}

void example0(void) {
    lcd_send_str("0123456789ABCDEF");
    while (1) {
        int i;
        for (i = 0; i < 10; i++) {
            lcd_shift_display_right(1);
            delay_ms(500);
        }    
        for (i = 0; i < 10; i++) {
            lcd_shift_display_left(1);
            delay_ms(500);
        }
    }
}

void example1(void) {
    int delay = 500;
    lcd_clear();
    delay_ms(delay); lcd_send_char('S');
    delay_ms(delay); lcd_send_char('H');
    delay_ms(delay); lcd_send_char('A');
    delay_ms(delay); lcd_send_char('R');
    delay_ms(delay); lcd_send_char('P');
    delay_ms(delay); lcd_send_char(' ');
    delay_ms(delay); lcd_send_char('L');
    delay_ms(delay); lcd_send_char('C');
    delay_ms(delay); lcd_send_char('D');
    delay_ms(delay); lcd_send_char(' ');
    delay_ms(delay); lcd_send_char('U');
    delay_ms(delay); lcd_send_char('N');
    delay_ms(delay); lcd_send_char('I');
    delay_ms(delay); lcd_send_char('T');
    delay_ms(delay); lcd_send_char(' ');
    delay_ms(delay); lcd_send_command(ENTRY_MODE_SET_BITS(ENTRY_INCREMENT_BIT | ENTRY_SHIFT_DISPLAY_BIT));
    delay_ms(delay); lcd_send_char('L');
    delay_ms(delay); lcd_send_char('M');
    delay_ms(delay); lcd_send_char('1');
    delay_ms(delay); lcd_send_char('7');
    delay_ms(delay); lcd_send_char('1');
    delay_ms(delay); lcd_move_cursor_left(1);
    delay_ms(delay); lcd_move_cursor_left(1);
    delay_ms(delay); lcd_send_char('6');
    delay_ms(delay); lcd_shift_display_right();
    delay_ms(delay); lcd_move_cursor_right(1);
    delay_ms(delay); lcd_send_char('5');
    delay_ms(delay); lcd_home();
    delay_ms(delay * 2);
}

void example2(void) {
    int delay = 500;
    lcd_clear();
    delay_ms(delay); lcd_send_char('S');
    delay_ms(delay); lcd_send_char('H');
    delay_ms(delay); lcd_send_char('A');
    delay_ms(delay); lcd_send_char('R');
    delay_ms(delay); lcd_send_char('P');
    delay_ms(delay); lcd_send_char(' ');
    delay_ms(delay); lcd_send_char('L');
    delay_ms(delay); lcd_send_char('C');
    delay_ms(delay); lcd_send_char('D');
    delay_ms(delay); lcd_send_char(' ');
    delay_ms(delay); lcd_send_char('U');
    delay_ms(delay); lcd_send_char('N');
    delay_ms(delay); lcd_send_char('I');
    delay_ms(delay); lcd_send_char('T');
    delay_ms(delay); lcd_move_cursor_to_start_of_second_line();
    delay_ms(delay); lcd_send_char('L');
    delay_ms(delay); lcd_send_char('M');
    delay_ms(delay); lcd_send_char('A');
    delay_ms(delay); lcd_send_char('1');
    delay_ms(delay); lcd_send_char('6');
    delay_ms(delay); lcd_send_char('2');
    delay_ms(delay); lcd_send_char('5');
    delay_ms(delay); lcd_send_char('1');
    delay_ms(delay); lcd_send_char(':');
    delay_ms(delay); lcd_send_char('1');
    delay_ms(delay); lcd_send_char('6');
    delay_ms(delay); lcd_send_char('C');
    delay_ms(delay); lcd_send_char('H');
    delay_ms(delay); lcd_send_char('A');
    delay_ms(delay); lcd_shift_display_left();
    delay_ms(delay); lcd_send_char('R');
    delay_ms(delay); lcd_home();
    delay_ms(delay * 2);
}

// Demo user generated characters.
void example3(void) {
    // Characters to be put in CG RAM
    unsigned char cg_user_data[6][8] = {
        {
            0b01010,
            0b10001,
            0b01000,
            0b00100,
            0b00010,
            0b10001,
            0b01010,
            0b00000
        },
        {
            0b00000,
            0b10101,
            0b00000,
            0b00100,
            0b00000,
            0b00100,
            0b00000,
            0b00000
        },
        {
            0b00000,
            0b10101,
            0b00000,
            0b10100,
            0b00000,
            0b10101,
            0b00000,
            0b00000
        },
        {
            0b00000,
            0b10101,
            0b00000,
            0b10100,
            0b00000,
            0b10000,
            0b00000,
            0b00000
        },
        {
            0b00000,
            0b00100,
            0b01010,
            0b10001,
            0b01010,
            0b10001,
            0b00000,
            0b00000
        },
        {
            0b00000,
            0b10001,
            0b01000,
            0b10101,
            0b00010,
            0b10001,
            0b00000,
            0b00000
        }
    };

    // Write characters to CG RAM
    int index, row;
    for (index = 0; index < 6; index++) {
        for (row = 0; row < 8; row++) {
            lcd_user_char_set_row(index, row, cg_user_data[index][row]);
        }
    }

    // Must reset cursor after lcd_user_char_set_row,
    // since the address counter is chared between CG and DD RAM.
    lcd_home();

    lcd_send_char('<');

    // Use characters written to CG RAM
    for (index = 0; index < 6; index++) {
        lcd_send_user_char(index);
    }

    lcd_send_char('>');

    while (1) {}
}
