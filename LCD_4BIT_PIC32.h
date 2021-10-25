/********************************************************************
* FileName:        LCD_4BIT_PIC32.h
*
* Updated 7-12-21
********************************************************************/
// #include <xc.h>

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

// #define MAX_LCD_STRING 16
#define MAX_LCD_STRING 20

		

#define ENABLE  1                       
#define DISABLE 0			

#define SET  	1                       
#define CLEAR	0			

#define LCD_RS  LATBbits.LATB12
#define LCD_RW  LATBbits.LATB13
#define LCD_EN  LATAbits.LATA10

#define LCD_D4  PORTBbits.RB6
#define LCD_D5  PORTCbits.RC3
#define LCD_D6  PORTCbits.RC4
#define LCD_D7  PORTCbits.RC5

// LCD instructions
#define LCD_COMMAND 1
#define LCD_DATA 0
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// #define	LCD_STROBE {LCD_EN=1; DelayUs(1); LCD_EN=0; DelayMs(10);}
// #define	LCD_STROBE {mPORTBSetBits(BIT_2); DelayUs(1); mPORTBClearBits(BIT_2);}
#define	LCD_STROBE {LATAbits.LATA10 = 1; SetDelay(1); LATAbits.LATA10 = 0;}

void WaitLCD(void);
void WriteNibble(unsigned char CommandFlag, char byte);
void WriteByte(unsigned char CommandFlag, char byte);
void LCDInit(void);
void LCDClear(void);
void LCDGoto(unsigned char Pos,  unsigned char Ln);
void LCDPutChar(char Data);
void LCDPutByte(char Val);
void LCDWriteStr(char  *Str);
void LCDWriteArray (char  *arrayPtr);

