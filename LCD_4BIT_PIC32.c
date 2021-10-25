/********************************************************************
* FileName:        LCD_4BIT_PIC32.c
* Tested with DM1602K-NSW-FBS-3.3v
* Updated for New Haven 4x16 character NHD-0420H1Z-FSW-GBW
* 
* Also should work with any standard 2x16 display
* Processor: 	   PIC 32MX795F512L on YBW32 Board
* Compiler: 	   Microchip XC32 V1.30 IDE: V5.15
*
* 
* Created 8-21-2020 JBS
* 7-12-21 JBS: Updated for Harmony Project and PIC 32MX170F256D
 * 
********************************************************************/
//#include <xc.h>
#include "LCD_4BIT_PIC32.h"
//#include "Delay.h"

//*****************************************************************************
//                            CONSTANT DEFINITION
//*****************************************************************************


void DelayMsLCD(long ms)
{
    SetDelay(ms * 10);
    while (GetDelay() != 0);    
}

void WaitLCD(void){
	DelayMsLCD(4);
}

/*
* Function  		WriteNibble
*
* @brief         	This function writes the specified high nibble to the LCD and
*                  	notifies the (LCD) controller whether it has to interpret
*                  	the nibble as data or command.
*
* @param	        CommandFlag=TRUE for command, FALSE to write character
*                   	
*
* @param	        byte command or character byte
*
*/
/*******************************************************************/

void WriteNibble(unsigned char CommandFlag, char byte)
{
    if (CommandFlag) LCD_RS = 0;
    else LCD_RS = 1;
    
    if (byte & 0x08) LCD_D7 = 1;
    else LCD_D7 = 0;
    
    if (byte & 0x04) LCD_D6 = 1;
    else LCD_D6 = 0;
    
    if (byte & 0x02) LCD_D5 = 1;
    else LCD_D5 = 0;
    
    if (byte & 0x01) LCD_D4 = 1;
    else LCD_D4 = 0;
    
    LCD_STROBE
}


/********************************************************************
* Function:         WriteByte
* 
*
* @brief          	This function writes the specified Byte to the LCD and
*                   notifies the (LCD) controller whether it has to interpret
*                   the Byte as data or command. 
*
* @param	        CommandFlag		This flag specifies whether the data to be written to
*                   		the LCD is a command or data to be displayed.
*
* @param	        byte		Actual data or command to be written to the LCD controller.
*
* @note    			This routine is meant to be used from within this module only.
*/
/*******************************************************************/ 

void WriteByte(unsigned char CommandFlag, char byte)
{
unsigned char ch;

    ch = (byte & 0xF0) >> 4;
    WriteNibble(CommandFlag, ch);     // Send the high nibble
    
    ch = byte & 0x0F;
    WriteNibble(CommandFlag, ch);     // Send low nibble
}

/********************************************************************
* Function:         LCDInit
*
* @brief          	This routine is called once at start up to initialize the
*                   MCU hardware for proper LCD operation.
*
* @note    			Should be called at system start up only.
*/                                                                          
/*******************************************************************/



// initialise the LCD - put into 4 bit mode
#define BIT_DELAY 10
void LCDInit(void)
{	
    LATAbits.LATA0 = 0;
    DelayMsLCD(200);	// power on delay
    
    LCD_D7 = 1;
    LCD_D6 = 1;
    LCD_D5 = 0;
    LCD_D4 = 0;    

    LCD_STROBE
    DelayMsLCD(2);
    LCD_STROBE
    DelayMsLCD(2);
    LCD_STROBE
    DelayMsLCD(20);
    
    WriteByte(LCD_COMMAND, lcd_FunctionSet4bit);    
    DelayMsLCD(BIT_DELAY);	

    WriteByte(LCD_COMMAND, lcd_Clear);    
    DelayMsLCD(BIT_DELAY);	
    
    WriteByte(LCD_COMMAND, lcd_EntryMode);    
    DelayMsLCD(BIT_DELAY);	

    WriteByte(LCD_COMMAND, lcd_DisplayOn);    
    DelayMsLCD(BIT_DELAY);	
}



/********************************************************************
* Function:         LCDClear
* 
* PreCondition: 	None
*
* @brief         	This function is called to wipe the LCD display out.
*
* @note    			None.
*/  
/*******************************************************************/

void LCDClear(void){
  WriteByte(TRUE,0x01);                       // Send clear display command
  WaitLCD();                                  // Wait until command is finished
}

/********************************************************************
* Function:         LCDGoto
*
* @brief          	This function positions the cursor at the specified Line
*                   and column.
*
* @param	        Pos		Column (0 to 15) the cursor should be positioned at.
*
* @param	        Ln		Line (0 or 1) the cursor should be positioned at.
*
* @note    			0 <= Pos <= 15               
* @note				0 <= Ln <= 1
*/                                                                    
/*******************************************************************/

void LCDGoto(unsigned char Pos,  unsigned char Ln)
{
	if (Ln==0) WriteByte(TRUE, 0x80|Pos);
	else if (Ln==1) WriteByte(TRUE, 0xC0|Pos);
    else if (Ln==2) WriteByte(TRUE, 0x80|(Pos + 0x14));
    else WriteByte(TRUE, 0xC0|(Pos+0x54));
	WaitLCD();                                      			 // Wait for the LCD to finish
}

/********************************************************************
* Function:         LCDPutChar
*
* @brief            This function displays the specified ASCII character at
*                   current position on the LCD
*
* @param	    ch - ASCII data character to be displayed.
*
*/ 
/*******************************************************************/

void LCDPutChar(char ch){
  WriteByte(FALSE, ch);              // Go output the character to the display
  WaitLCD();                          // Wait until it's finished
}

/********************************************************************
* Function:         LCDPutByte
*
* @brief            This function displays the specified binary value at
*                   current position on the LCD. It converts the binary
*                   value into displayable ASCII characters.
*
* @param	    Val		Binary data byte to be displayed
*
* @note    			In the present configuration, this routine displays a
*                   2-digit value, by prefilling with '0' any value lower
*                   than 10.
*/
/*******************************************************************/

void LCDPutByte(char Val){
  LCDPutChar(Val/10+'0');                   // Output the high digit
  LCDPutChar(Val % 10+'0');                 // Output low
}

/********************************************************************
* Function:         LCDWriteStr
*
* @brief          	This function displays the specified string starting from
*                   current position on the LCD.
*
* @param	        Str		IF 0; Terminated string to be displayed.
*
* @note    			None
*/
/*******************************************************************/

void LCDWriteStr(char  *Str){
unsigned char i = 0; 
char ch;

  do {
    ch = Str[i++];
    if (!ch) break;
    LCDPutChar(ch); 
  } while (i < MAX_LCD_STRING);
}

void LCDWriteArray (char  *arrayPtr)
{
unsigned char ch, i=0;                                     // Char index buffer

	i=0;
	for(i=0; i<MAX_LCD_STRING; i++){
		ch=arrayPtr[i];
        LCDPutChar(ch);
		//if(ch)LCDPutChar(ch);                          // Go display current char
		//else break;
	}
}

