// Code to drive the LM16A211 5x7-dot 16-character 2-line dot-matrix LCD panel.
//
// Reference Material
// ==================
//  [0] http://www.epemag.wimborne.co.uk/lcd1.pdf
//  [1] http://www.epemag.wimborne.co.uk/lcd2.pdf
//  [3] http://stalmp3box.sourceforge.net/LM16A211.pdf
//  [4] http://icecap.se/Sublevel/LCD_Dot_Matrix_User_Manual.pdf

// Instruction table from SHARP LM16A211 data sheet [3]
// ================================================
// Instruction                    | RS | R/W | DB7 | DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0
// Display Clear                  | 0  | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 1
// Display/Cursor Home            | 0  | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 1   | *
// Entry mode set                 | 0  | 0   | 0   | 0   | 0   | 0   | 0   | 1   | I/D | S
// Display ON/OFF                 | 0  | 0   | 0   | 0   | 0   | 0   | 1   | D   | C   | B
// Display/Cursor shift           | 0  | 0   | 0   | 0   | 0   | 1   | S/C | R/L | *   | *
// Function set                   | 0  | 0   | 0   | 0   | 1   | DL  | 1   | 0   | *   | *
// CG RAM address set             | 0  | 0   | 0   | 1   | ACG
// DD RAM addrss set              | 0  | 0   | 1   | ADD
// Busy flag/address counter read | 0  | 1   | BF  | AC
// CG RAM/DD RAM data write       | 1  | 0   | Write data
// CG RAM/DD RAM data read        | 1  | 1   | Read data
//
// I/D=1:Increment       I/D=0:Decrement        S/C=1:Shift display   S/C=0:Move cursor
// S  =1:Shift display   s  =0:Freeze display   R/L=1:Shift right     R/L=0:Shift left
// D  =1:Display ON      D  =0:Display OFF      DL =1:8-bit           DL =0:4-bit
// c  =1:Cursor ON       c  =0:Cursor OFF       BF=1:During internal  BF =0:END of internal
// B  =1:Character at    B  =0:Character at          operation              operation
//       cursor position       cursor position
//       blinks.               unblinks.
//

#ifndef LM16A211_H
#define LM16A211_H

// User must provide a lm16a211_config.h, PIC pin to LCD pin mapping.
#include "lm16a211_config.h"

#if !NO_CONFIG_CHECK
// The function is only here to check that all necessary configs are setup.
inline static void compiler_check_for_config_mappings() {
    LCD_RS = 0;
    LCD_E  = 0;
    LCD_RW = 0;
    LCD_D4 = 0;
    LCD_D5 = 0;
    LCD_D6 = 0;
    LCD_D7 = 0;
    LCD_D7_WHEN_READABLE;
    PREPARE_FOR_READ_FROM_LCD;
    PREPARE_FOR_WRITE_TO_LCD;
}
#endif

// Instructions Bits

// Display clear
#define DISPLAY_CLEAR_BITS (0b00000001)

// Display/cursor home
#define DISPLAY_CURSOR_HOME_BITS (0b00000010)

// Entry mode set
#define ENTRY_MODE_SET_BITS(bits) (0b00000100 | bits)
#define ENTRY_INCREMENT_BIT        0b00000010
#define ENTRY_DECREMENT_BIT        0b00000000
#define ENTRY_SHIFT_DISPLAY_BIT    0b00000001
#define ENTRY_FREEZE_DISPLAY_BIT   0b00000000

// Display ON/OFF
#define DISPLAY_BITS(bits) (0b00001000 | bits)
#define DISPLAY_BIT         0b00000100
#define CURSOR_BIT          0b00000010
#define BLINK_BIT           0b00000001

// Display/cursor shift
#define DISPLAY_CURSOR_SHIFT_BITS(bits) (0b00010000 | bits)
#define SHIFT_DISPLAY_BIT                0b00001000
#define MOVE_CURSOR_BIT                  0b00000000
#define SHIFT_RIGHT_BIT                  0b00000100
#define SHIFT_LEFT_BIT                   0b00000000

// Function set
#define FUNCTION_SET_BITS(bits) (0b00101000 | bits)
#define DATA_LENGTH_8_BIT        0b00010000
#define DATA_LENGTH_4_BIT        0b00000000

// CG RAM address set
#define CG_RAM_ADDRESS_SET_BITS(bits) (0b01000000 | (bits & 0b00111111))

// DD RAM address set
#define DD_RAM_ADDRESS_SET_BITS(bits) (0b10000000 | (bits & 0b01111111))


// Public functions
// ================

// Initializes the display.
void lcd_init(void);

void lcd_send_str(const char* str);
void lcd_send_char(unsigned char c);
void lcd_send_command(unsigned char data);

void lcd_move_cursor_right(int amount);
void lcd_move_cursor_left(int amount);
void lcd_move_cursor_to_start_of_second_line();

void lcd_shift_display_right();
void lcd_shift_display_left();

void lcd_cg_ram_address_set(char address);
void lcd_dd_ram_address_set(char address);

void lcd_clear();
void lcd_home();

#endif // LM16A211_H
