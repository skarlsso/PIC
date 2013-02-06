#include "lm16a211.h"

#include "delayer.h"
#include "led_debug.h"

// Timing characteristics from SHARP LM16A211 data sheet [3] (ns) 
// ==============================================================
// Enable cycle time
#define LCD_tcycE      1000
// Enable pulse width
#define LCD_PWEH        450
// Enable rise time (min)
#define LCD_tEr          25
// Enable fall time (min)
#define LCD_tEf          25
// RS,R/W setup time
#define LCD_tAS         140
// Address hold time
#define LCD_tAH          10
// Data setup time
#define LCD_tDSW        195
// Data delay time (min)
#define LCD_tDDR        320
// Data hold time (write)
#define LCD_tH           10
// Data hold time (read)
#define LCD_tDHR         20

// Add extra delay if delay functions are off.
#define DELAY_OFFSET 0L

// The longest time we have to wait, if we use the busy flag.
#define MAX_COMMAND_DELAY_IN_MS 10

#define COMMAND 0
#define DATA    1
#define WRITE   0
#define READ    1

#define lcd_set_mode(rw, type) \
    LCD_RS = type; \
    LCD_RW = rw; \
    delay_ns_with_offset(DELAY_OFFSET, LCD_tAS)

static void lcd_wait_until_not_busy() {
    PREPARE_FOR_READ_FROM_LCD;

    lcd_set_mode(READ, COMMAND);
    
    int busy = 1;
    int loop_count = 0;
    int ignored;

    while (busy) {
        LCD_E = 1;
        // Wait for data to become available
        delay_ns_with_offset(DELAY_OFFSET, LCD_tEr + LCD_PWEH);
        busy = LCD_D7_WHEN_READABLE;
        LCD_E = 0;
        delay_ns_with_offset(DELAY_OFFSET, LCD_tcycE - LCD_tEr - LCD_PWEH);

        LCD_E = 1;
        // Wait for data to become available
        delay_ns_with_offset(DELAY_OFFSET, LCD_tEr + LCD_PWEH);
        ignored = LCD_D7_WHEN_READABLE;
        LCD_E = 0;
        delay_ns_with_offset(DELAY_OFFSET, LCD_tcycE - LCD_tEr - LCD_PWEH);

        loop_count++;
    }

    PREPARE_FOR_WRITE_TO_LCD;
}

static void lcd_wait_busy() {
#if !DONT_USE_RW_PIN
    lcd_wait_until_not_busy();
#else
    delay_ms(MAX_COMMAND_DELAY_IN_MS);
#endif
}

static void lcd_send4_no_wait(unsigned char data) {
    // Write the data to the data pins
    LCD_D4 =    ((data & 0b1) >> 0);
    LCD_D5 =   ((data & 0b10) >> 1);
    LCD_D6 =  ((data & 0b100) >> 2);
    LCD_D7 = ((data & 0b1000) >> 3);
    
    // Send the data
    LCD_E = 1;
    delay_ns_with_offset(DELAY_OFFSET, LCD_tEr);
    LCD_E = 0;
}

static void lcd_send8(unsigned char data, int type) {
    lcd_set_mode(WRITE, type);
    lcd_send4_no_wait(data >> 4);
    lcd_send4_no_wait(data);
    lcd_wait_busy();
}


static void lcd_send_command4(unsigned char data) {
    lcd_set_mode(WRITE, COMMAND);
    lcd_send4_no_wait(data);
    lcd_wait_busy();
}

void lcd_send_command(unsigned char data) {
    lcd_send8(data, COMMAND);
}


void lcd_send_char(unsigned char c) {
    lcd_send8(c, DATA);
}

void lcd_send_str(const char* str) {
    int i;
    
// Until code is stable
#define LCD_SEND_STR_MAX_LENGTH 20

    for (i = 0; i < LCD_SEND_STR_MAX_LENGTH; i++) {
        char c = str[i];
        if (c == '\0') {
            break;
        }
        lcd_send_char(c);
    }
}

#define RIGHT 0
#define LEFT  1
#define DISPLAY 0
#define CURSOR  1

static void lcd_move_cursor_or_shift_display(char display_or_cursor, char right_or_left) {
    int direction = (right_or_left == RIGHT ? SHIFT_RIGHT_BIT : SHIFT_LEFT_BIT);
    int type = (display_or_cursor == DISPLAY ? SHIFT_DISPLAY_BIT : MOVE_CURSOR_BIT);

    lcd_send_command(DISPLAY_CURSOR_SHIFT_BITS(type | direction));
}


void lcd_move_cursor_right(int amount) {
    while (amount-- > 0) {
        lcd_move_cursor_or_shift_display(CURSOR, RIGHT);
    }
}

void lcd_move_cursor_left(int amount) {
    while (amount-- > 0) {
        lcd_move_cursor_or_shift_display(CURSOR, LEFT);
    }
}


void lcd_shift_display_right() {
    lcd_move_cursor_or_shift_display(DISPLAY, RIGHT);
}

void lcd_shift_display_left() {
    lcd_move_cursor_or_shift_display(DISPLAY, LEFT);
}


void lcd_cg_ram_address_set(char address) {
    lcd_send_command(CG_RAM_ADDRESS_SET_BITS(address));
}

void lcd_dd_ram_address_set(char address) {
    lcd_send_command(DD_RAM_ADDRESS_SET_BITS(address));
}

void lcd_clear() {
    lcd_send_command(DISPLAY_CLEAR_BITS);
}

void lcd_home() {
    lcd_send_command(DISPLAY_CURSOR_HOME_BITS);
}

void lcd_init(void) {
#ifdef PRE_LCD_INIT
    PRE_LCD_INIT;
#endif

    // Initial delay > 15ms.
    delay_ns(20UL * 1000UL * 1000UL);

    // Only 4-bit data lenght supported, yet.
    unsigned char fs_bits = FUNCTION_SET_BITS(DATA_LENGTH_4_BIT);
    
    // Setup function set in 8-bit mode
    lcd_send_command4(fs_bits >> 4);

	// Setup function set in 4-bit mode
    lcd_send_command(fs_bits);

    // Turn on display with blinking cursor
    lcd_send_command(DISPLAY_BITS(DISPLAY_BIT | CURSOR_BIT | BLINK_BIT));

    // Entry mode
    lcd_send_command(ENTRY_MODE_SET_BITS(ENTRY_INCREMENT_BIT|ENTRY_FREEZE_DISPLAY_BIT));

    // Clear
    lcd_send_command(DISPLAY_CLEAR_BITS);
}
