#include "lm16a211.h"

#include "delayer.h"
#include "led_debug.h"

#define LCD_CTRL_RS_CHARACTER 1
#define LCD_CTRL_RS_COMMAND   0
#define LCD_CTRL_RW_WRITE     0
#define LCD_CTRL_RW_READ      1

#define RIGHT 1
#define LEFT  0
#define DISPLAY 1
#define CURSOR  0

// Add extra delay if delay functions are off.
#define DELAY_OFFSET 0L

// The longest time we have to wait, if we use the busy flag.
#define MAX_COMMAND_DELAY_IN_MS 10

#define lcd_set_mode(rs, rw) { \
    LCD_RS = rs; \
    LCD_RW = rw; \
    delay_ns_with_offset(DELAY_OFFSET, LCD_tAS); \
}
#define lcd_set_write_character()  lcd_set_mode(LCD_CTRL_RS_CHARACTER, LCD_CTRL_RW_WRITE)
#define lcd_set_write_command()    lcd_set_mode(LCD_CTRL_RS_COMMAND,   LCD_CTRL_RW_WRITE)
#define lcd_set_read_command()     lcd_set_mode(LCD_CTRL_RS_COMMAND,   LCD_CTRL_RW_READ)

#define DONT_USE_RW_PIN 0
static void lcd_wait_busy() {
#if DONT_USE_RW_PIN
    {
        delay_ms(MAX_COMMAND_DELAY_IN_MS);
        return;
    }
#endif
    
    int stored_RS = LCD_RS;
    int stored_RW = LCD_RW;

    PREPARE_FOR_READ_FROM_LCD;
    
    lcd_set_read_command();
    
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

    lcd_set_mode(stored_RS, stored_RW);
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

static void lcd_send8_no_wait(unsigned char data) {
    lcd_send4_no_wait(data >> 4);
    lcd_send4_no_wait(data);
}

static void lcd_send8(unsigned char data) {
    lcd_send8_no_wait(data);
    lcd_wait_busy();
}


void lcd_send_str(const char* str) {
    int i;
    
    lcd_set_write_character();

// Until code is stable
#define LCD_SEND_STR_MAX_LENGTH 20

    for (i = 0; i < LCD_SEND_STR_MAX_LENGTH; i++) {
        char c = str[i];
        if (c == '\0') {
            break;
        }
        lcd_send8(c);
    }
}

// High-level functions

void lcd_send_char(char c) {
    lcd_set_write_character();
    lcd_send8(c);
}

void lcd_send_command(char data) {
    lcd_set_write_command();
    lcd_send8(data);
}


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


void lcd_shift_display_right(int amount) {
    while (amount-- > 0) {
        lcd_move_cursor_or_shift_display(DISPLAY, RIGHT);
    }
}

void lcd_shift_display_left(int amount) {
    while (amount-- > 0) {
        lcd_move_cursor_or_shift_display(DISPLAY, LEFT);
    }
}


void lcd_init(void) {
#ifdef PRE_LCD_INIT
    PRE_LCD_INIT;
#endif

    // Initial delay > 15ms.
    delay_ns(20UL * 1000UL * 1000UL);

    lcd_set_write_command();

    // Only 4-bit data lenght supported, yet.
    unsigned char fs_bits = FUNCTION_SET_BITS(DATA_LENGTH_4_BIT);
    
    // Setup function set in 8-bit mode
	lcd_send4_no_wait(fs_bits >> 4);
    // Can't check busy flag (?), must do a manual wait.
    delay_ns(5 * 1000UL * 1000UL);

	// Setup function set in 4-bit mode
    lcd_send8(fs_bits);

    // Turn on display with blinking cursor
    lcd_send8(DISPLAY_BITS(DISPLAY_BIT | CURSOR_BIT | BLINK_BIT));

    // Entry mode
    lcd_send8(ENTRY_MODE_SET_BITS(ENTRY_INCREMENT_BIT|ENTRY_FREEZE_DISPLAY_BIT));
    
    // Clear
    lcd_send8(DISPLAY_CLEAR_BITS);
}
