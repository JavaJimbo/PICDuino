/*******************************************************************************
 * main.c: Main Source File PIC170Dunio
 *
 * MPLABX IDE V5.45
 * Compiler: Microchip XC32 V3.01
 * Uses PIC 32MX170F256D - 44 pin micro running at 40 Mhz from 16 Mhz crystal.
 * 
 * 7-12-21: Got it working with four line New Haven LC
 * 7-13-21: Got AD conversion and PWM working.
 * 7-14-21: Got I2C working with 24LC256 EEprom. Created I2C_EEprom_PIC32_STD files
 *  Implemented single byte and also block read and write
 *  Uses I2C_Errors to track of I2C errors and GetMemoryTimeout() to prevent getting stuck
 * 7-15-21: Got SPI working with manual chip select. Use 25LC256 EEprom for testing.
 * 7-16-21: Finally got UART1 RX interrupts working without using callbacks.
 *          Initialize Timers #4 & #5 as encoder counters in SYS_Initialize().
 * 7-17-21: PID control works nicely with one motor. 
 *          Currently set up for two motors, but can probably do four.
 * 7-22-21: 
 * 7-23-21: Tested with two motors. Got UART 1 working.
 * 7-25-21: Works with Pololu VNH5019 board running two motors.
 * 7-26-21: Added DIR A and B
 * 7-27-21: Retested EEpromWriteBlock(), EEpromReadBlock(). Updated pin assignments.
 *          Got analog current measurement working
 * 7-31-21: Modified I2C routines to work with 24LC1025 and up to four devices  
 * 8-1-21:  Got all four 24LC1025 ICs working. Reading and writing with no delays, just check status.
 * 8-2-21:  Fixed bug in I2C EEprom - block boundaries are 128 bytes for 24LC1025
 *          Also added INT0, INT1 etc.
 *          Got Callbacks more or less working
 * 8-3-21:  GOt Atmel AT45DB161 fully functioning. Not using Timer 3 callbacks.
 * 8-4-21:  Added WRITE PROTECT for Atmel.
 * 8-5-21:  
 * 8-25-21: Recompiled to test new dual VNH5019 boards.
 * 10-25-21: Recompiled and stored modified files on GitHub: initializations.c, plib_gpio.c?, plib_ocmp1.c?, plib_ocmp2.c?, plib_tmr3.c, plib_uart1.c, plib_uart2.c, I2C_EEprom_PIC32_STD.c, main.c
 *******************************************************************************/
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <ctype.h>
#include "definitions.h"       
#include "AT45DB161.h"

#include "device.h"

#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023
#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86

//#define FORWARD 0
//#define REVERSE 1
#define FORWARD 1
#define REVERSE 0



enum {
    POT_MODE = 0,
    DESTINATION_MODE,
    CONTINUOUS_MODE    
};
#define DEFAULT_MODE POT_MODE // CONTINUOUS_MODE
enum
{
    STANDBY = 0,
    RECORD,
    PLAY,
    PAUSE_RECORD,
    PAUSE_PLAY,
    HALT
};


//#define MOTOR_DIR1 LATBbits.LATB12
//#define MOTOR_DIR2 LATBbits.LATB13

#define DIR1_A LATBbits.LATB12
#define DIR1_B LATCbits.LATC3
#define DIR2_A LATAbits.LATA10
#define DIR2_B LATAbits.LATA7

        
// For 26:1 motors
#define PWM_OFFSET 0 
#define KP 8.0
#define KI 0.01 
#define KD 10.0 

#define KK_PWM_OFFSET 100
#define KKP 8.0
#define KKI 0.04
#define KKD 1000.0 // 30


#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

enum 
{
    HALTED=0,
    LOCAL,
    JOG,
    REMOTE    
};
 

#define EncoderOneDir PORTBbits.RB4
#define EncoderTwoDir PORTAbits.RA8

#define MAXNUM 16
#define MAXSUM 500000
#define ACCELERATION 0.1
#define FILTERSIZE 16

#define PWM_MAX 2000
#define PWM1 OC1RS
#define PWM2 OC2RS

#define SPI_READ  0x03
#define SPI_WRITE 0x02
#define SPI_READ_STATUS 0x05
#define SPI_WRITE_ENABLE 0x06

#define EncoderOne TMR4
#define EncoderTwo TMR5

#define MAXPOTS 6
#define FOREVER 1
#define NUMMOTORS 2

struct PIDtype
{
    long  error[FILTERSIZE];
    long  sumError;    
    float kP;
    float kI;
    float kD;
    long PWMoffset;
    long PWMvalue;
    short ADActual;    
    long  Destination;
    float  CommandPos;    
    long ActualPos;
    unsigned short PreviousQuad;
    float Velocity;
    float TargetVelocity;
    float ActualVelocity[FILTERSIZE];
    short ADCommand;
    BYTE saturation;
    BYTE reset;    
    BYTE Mode;    
    BYTE Halted;
    BYTE RemoteDataValid;
} PID[NUMMOTORS];

#define SWITCH_PRESSED_STATE            0   // Active LOW switch
//#define LED_On()                        LED_Set()
//#define LED_Off()                       LED_Clear()


char messageStart[] = "\r\r****  UART echo demo: Non-blocking Transfer with interrupt on CR **** \r**** Yes yes yes said Chicken Little #1****";
char messageError[] = "**** UART error occurred ****\r\n";

extern UART_OBJECT uart1Obj;
unsigned short ADresult[MAXPOTS];
unsigned short InterruptCounter = 0x00;
unsigned short TestCounter = 0x0000;
#define MAXBUFFER 128
char HOSTRxBuffer[MAXBUFFER+1];
char UART1RxBuffer[MAXBUFFER+1];
char UART1TxBuffer[MAXBUFFER+32] = "\rThis has got to be the one";
short UART1TxLength = 0;
// short UART1RxIndex = 0;
short HOSTRxIndex = 0;
UART_SERIAL_SETUP DefaultUART;

int ADC10_ManualInit(void);

unsigned short Timer4Count = 0, Timer5Count = 0;
unsigned char HOSTRxBufferFull = FALSE;

void ResetPID();
void InitPID();
long ENCODERcontrol(long servoID, struct PIDtype *PID);
long POTcontrol(long servoID, struct PIDtype *PID);
unsigned char ReadADC(unsigned short *ptrADResult, unsigned short *ptrADFiltered);
void SetMilliSecondFlag(unsigned char BoolValue);
void SetMilliSecondTimeout(unsigned short TimeoutMilliSeconds);
unsigned char GetMilliSecondTimeout();
unsigned char GetMemoryTimeout();
void SetDelay(long Delay);
long GetDelay();
unsigned char getT3Flag();
void setT3Flag();
void ClearT3Flag();
void ClearUART1RxBuffer();
void Randomize(unsigned char *ptrData);
void DelayMs(long ms);
void FillAtmelPage(unsigned char *ptrBuffer);
void controlLED(GPIO_PIN pin, uintptr_t context);
void APP_WriteCallback(uintptr_t context);
void APP_ReadCallback(uintptr_t context);
void DelayMs(long ms);


unsigned char displayFlag = false;
unsigned short errIndex = 0;

bool errorStatus = false;
bool writeStatus = false;
bool readStatus = false;


typedef enum
{
   APP_STATE_LED_ON,
   APP_STATE_LED_TOGGLE

}APP_STATE;

volatile static APP_STATE  state = APP_STATE_LED_ON;
unsigned char AtmelStoreBuffer[ATMEL_PAGESIZE];
unsigned char AtmelFetchBuffer[ATMEL_PAGESIZE];

void Randomnize(unsigned char *ptrData)
{
static unsigned char TestByte = 0;    
int i;
    
    for (i = 0; i < PAGESIZE; i++)
    {
        ptrData[i] = TestByte;
        TestByte = TestByte + 33;
    }
}


int main (void)
{    
char ch;
unsigned short ADfiltered[MAXPOTS];
unsigned short DisplayJogPot = 0;
BYTE command = 0;
BYTE NUMbuffer[MAXNUM + 1];
float floValue = 0;
BYTE runMode = LOCAL;
BYTE runState = HALTED;   
short p = 0, q = 0, i = 0;
unsigned ADdisplay = true;
float tempCommand = 0;
short PWMvalue = 0;
short runLoopCounter = 0;
unsigned short pageNum = 123;
short ErrorCounter = 0;
unsigned char BufferID = 1;
// char strTestLine[MAXBUFFER+8] = "\r\rYes yes yes said Chicken Little #2";

unsigned char TestMessageOut[PAGESIZE];
unsigned char TestMessageIn[PAGESIZE];
unsigned long StartAddress = 13;
unsigned long address = 0;
//unsigned long EndAddress = 131072 * 4;
int errorCounter = 0;

/*
union
{
	unsigned char b[4];
	unsigned long lng;
} convert;
*/

    SYS_Initialize(NULL);  
    WRITE_PROTECT_ON();
    SS_Set();
    ClearUART1RxBuffer();
        
    GPIO_PinInterruptCallbackRegister(INT0_PIN, controlLED, (uintptr_t)NULL);  
    INT0_InterruptEnable();

    PWM1 = 0;
    PWM2 = 0; 
    DIR1_A = DIR1_B = DIR2_A = DIR2_B = 0;
    
    ADC10_ManualInit();      
    I2C_init(100000);  
    InitPID();   
  
    
    pageNum = 0;
    BufferID = 1;
    printf("\r\rSTART PID MEMORY TEST AND PID LOOP #1...");
    
    while(pageNum <= 100)
    {
        if (0==(pageNum % 100)) printf("\rPage %d", pageNum);
        EraseFLASHpage(pageNum);   
        FillAtmelPage(AtmelStoreBuffer);
        // WriteAtmelPage(BufferID, AtmelStoreBuffer);
        WriteAtmelBytes (BufferID, AtmelStoreBuffer, 0x0000, ATMEL_PAGESIZE);
        ProgramFLASH(BufferID, pageNum);
        TransferFLASH(BufferID, pageNum);
        // ReadAtmelPage(BufferID, AtmelFetchBuffer);
        ReadAtmelBytes (BufferID, AtmelFetchBuffer, 0x0000, ATMEL_PAGESIZE);        
        
        for(i = 0; i < ATMEL_PAGESIZE; i++)
        {
            if (AtmelStoreBuffer[i] != AtmelFetchBuffer[i])
            {           
                ErrorCounter++;
                printf("\rERROR @ Page %d, Byte #%d: %d != %d", pageNum, (int)i, (int)AtmelStoreBuffer[i], (int)AtmelFetchBuffer[i]);
                break;
            }
        }
        pageNum++;
    }
    
    if (!ErrorCounter) printf("\rOK No errors!");
    else printf("\r Write/read ERRORS");

    
    errorCounter = 0;
    address = StartAddress;
    // for(address = StartAddress; address < EndAddress; address = address + PAGESIZE)
    {
        Randomnize(TestMessageOut);
        EEpromWriteBlock(address, TestMessageOut, PAGESIZE);
        EEpromReadBlock(address, TestMessageIn, PAGESIZE);
        
        printf("\rSTART I2C TEST @ %ld", address);
        for (i = 0; i < PAGESIZE; i++) 
        {
            // errorCounter = 0;
            if (TestMessageIn[i] != TestMessageOut[i]) 
            {
                errorCounter++;
                printf("\rError #%d: %d != %d", errorCounter, (int)TestMessageOut[i], (int)TestMessageIn[i]);
                break;
            }
        }
        // if (errorCounter) break;
    }
    printf("\rDONE");
    
    
    // Register callback functions and send start message 
    printf("\rRegistering callbacks...");
    UART1_WriteCallbackRegister(APP_WriteCallback, 0);
    UART1_ReadCallbackRegister(APP_ReadCallback, 0);
    UART1_Write(&messageStart, sizeof(messageStart));    
        
    printf("\rBeginning main loop...");
    
    while(FOREVER)
    {          
        if(errorStatus == true)
        {
            errorStatus = false;
            UART1_Write(&messageError, sizeof(messageError));
        }
        else if(readStatus == true)
        {
            readStatus = false;
            strcpy(UART1TxBuffer, UART1RxBuffer);
            ClearUART1RxBuffer();
            UART1_Write(UART1TxBuffer,  strlen(UART1TxBuffer));            
        }
        else if(writeStatus == true)
        {
            writeStatus = false;
            UART1_Read(UART1RxBuffer, MAXBUFFER);
        }
        // else continue;
        /*
                                                                                 // Find end of received data and
        UART1RxBuffer[UART1_ReadCountGet()] = '\0';                              // zero terminate it so strchr() will work.
        if (strchr(UART1RxBuffer, (char)'\r'))                                   // Carriage return received? If so then...
        {                                                
            UART1TxLength = sprintf(UART1TxBuffer, "\rRX> %s", UART1RxBuffer);   // Copy received data
            UART1_ReadAbort();                                                   // Disable RX interrupts and reset flags
            ClearUART1RxBuffer();
            UART1_Read(UART1RxBuffer, MAXBUFFER);                                // Re-enable RX interrupts 
            UART1_Write(UART1TxBuffer, UART1TxLength);                           // Echo received data 
            SPARE1_Toggle();
        }        
        */
        ReadADC(ADresult, ADfiltered);       
        if (getT3Flag())       
        {
            ClearT3Flag();      
            if (runState)
            {            
                if (runLoopCounter) runLoopCounter--;
                if (!runLoopCounter)
                {
                    runLoopCounter = 50;   
                    //SPARE1_Toggle();
                }                
                
                if (runState == JOG)
                {
                    if (DisplayJogPot) DisplayJogPot--;
                    else DisplayJogPot = 40;
                }
                else DisplayJogPot = 0;
                    
                for (i = 0; i < NUMMOTORS; i++)
                {                                              
                    if (runState == JOG) 
                    {                  
                        PWMvalue = (ADresult[i+4] - 512) * 8; // LOCAL pots on BRAIN BOARD are analog inputs AN1-AN4                        
                        if (!DisplayJogPot)
                        {         
                            if (i == 0) printf("\rAD0 %d > %d, ", (int) PWMvalue, (int)ADresult[0]);
                            if (i == 1) printf("\rAD1 %d > %d, ", (int) PWMvalue, (int)ADresult[1]);
                        }                        
                        if (PWMvalue > PWM_MAX) PWMvalue = PWM_MAX;
                        if (PWMvalue < -PWM_MAX) PWMvalue = -PWM_MAX;                            
                    }
                    else if (!PID[i].Halted)
                    {
                        if (PID[i].Mode == POT_MODE) 
                        {
                                if (runState == LOCAL) 
                                {                                    
                                    tempCommand = ((float)ADfiltered[i+4]) / 1023;   // LOCAL pots on BRAIN BOARD are first four analog inputs    
                                    tempCommand = (tempCommand * (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS)) + MIN_COMMAND_COUNTS;                                    
                                    if (tempCommand < MIN_COMMAND_COUNTS) tempCommand = MIN_COMMAND_COUNTS;
                                    if (tempCommand > MAX_COMMAND_COUNTS) tempCommand = MAX_COMMAND_COUNTS;
                                    PID[i].ADCommand = (short)tempCommand; 
                                    POTcontrol(i, PID);
                                }
                                else
                                {
                                    if (PID[i].RemoteDataValid) POTcontrol(i, PID);
                                    else PID[i].PWMvalue = 0;
                                }
                        }
                        else ENCODERcontrol(i, PID);      
                        PWMvalue = PID[i].PWMvalue;                                                
                    }
                    else PWMvalue = 0;
                    command = 0;

                    if (i == 0) 
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR1_A = REVERSE;
                            DIR1_B = FORWARD;
                            PWMvalue = 0 - PWMvalue;                            
                        }
                        else
                        {
                            DIR1_A = FORWARD;
                            DIR1_B = REVERSE;
                        }
                        PWM1 = (unsigned short)PWMvalue;                        
                    }                        
                    else if (i == 1)
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR2_A = REVERSE;
                            DIR2_B = FORWARD;
                            PWMvalue = 0 - PWMvalue;                            
                        }
                        else
                        {
                            DIR2_A = FORWARD;
                            DIR2_B = REVERSE;
                        }
                        PWM2 = (unsigned short)PWMvalue;                                                
                    }
                }
                errIndex++; 
                if (errIndex >= FILTERSIZE) errIndex = 0;                
            }
            else            
            {
                for (i = 0; i < NUMMOTORS; i++) PID[i].sumError = 0;                
                PWM1 = PWM2 = 0;
                
            }        
        } // End if (getT3Flag())
        
        
        while (UART2_ReceiverIsReady( ))
        {
            ch = toupper (UART2_ReadByte());
            if (HOSTRxIndex < MAXBUFFER)
            {
                if (ch == 0 || ch == '\n') continue;                
                else if ('\r' == ch || (' ' == ch && HOSTRxIndex == 0)) 
                {
                    HOSTRxBuffer[HOSTRxIndex++] = ch;
                    HOSTRxBuffer[HOSTRxIndex] = '\0';                             
                    HOSTRxIndex = 0;
                    HOSTRxBufferFull = TRUE;
                }
                else HOSTRxBuffer[HOSTRxIndex++] = ch;
            }
            else HOSTRxIndex = 0;
        }        

        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;  
            printf("\rReceived: %s", HOSTRxBuffer);
            q = 0;
            command = 0;            
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (ch < ' ' && ch != '\r')
                {
                    command = ch;
                    break;
                }
                if (isalpha(ch) || (ch==' ' && p==0)) command = ch;                
                putchar(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( (isdigit(ch) || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof((const char *)NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'F':
                        if (!runState)
                        {
                            DIR1_A = DIR2_A = FORWARD;
                            DIR1_B = DIR2_B = REVERSE;                            
                            printf("\rFORWARD");
                        }
                        break;
                    case 'R':
                        if (!runState)
                        {
                            DIR1_A = DIR2_A = REVERSE;
                            DIR1_B = DIR2_B = FORWARD;                            
                            printf("\rREVERSE");                            
                        }
                        break;                        
                    case 'P':
                        if (q) PID[0].kP = PID[1].kP = PID[2].kP = PID[3].kP = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", (double)PID[0].kP, (double)PID[0].kI, (double)PID[0].kD, (int)PID[0].PWMoffset);
                        break;
                    case 'I':
                        if (q) PID[0].kI = PID[1].kI = PID[2].kI = PID[3].kI = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", (double)PID[0].kP, (double)PID[0].kI, (double)PID[0].kD, (int)PID[0].PWMoffset);
                        break;
                    case 'D':
                        if (q) PID[0].kD = PID[1].kD = PID[2].kD = PID[3].kD = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", (double)PID[0].kP, (double)PID[0].kI, (double)PID[0].kD, (int)PID[0].PWMoffset);
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = PID[1].PWMoffset = PID[2].PWMoffset = PID[3].PWMoffset = (long) floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", (double)PID[0].kP, (double)PID[0].kI, (double)PID[0].kD, (int)PID[0].PWMoffset);
                        break;
                    case 'X':
                        runMode = REMOTE;
                        runState = STANDBY;
                        printf("REMOTE MODE");
                        break;
                    case 'L':
                        runMode = LOCAL;
                        runState = STANDBY;
                        printf("LOCAL MODE");
                        break;       
                    case 'J':
                        printf("\rJOG MODE");
                        runState = JOG;                        
                        break;                        
                    case ' ':
                        ResetPID();
                        if (runState) 
                        {                            
                            runState = STANDBY; 
                            printf("\rHALT");
                        }
                        else
                        {
                            runState = runMode;
                            if (runState==LOCAL) printf("\rRun State = LOCAL");
                            else if (runState==REMOTE) printf("\rRun State = REMOTE");
                            else if (runState==JOG) printf("\rRun State = JOG");
                            else printf("\rERROR RunMode = %d", runMode);
                            if (PID[3].Velocity > 0) PID[3].Destination = PID[3].Destination + (abs(PID[3].Destination - PID[3].ActualPos));
                            else PID[3].Destination = PID[3].Destination - (abs(PID[3].Destination - PID[3].ActualPos));
                            printf(", Destination: %ld, Velocity: %0.3f", PID[3].Destination, (double)PID[3].Velocity);
                        }                         
                        break;                        
                    case 'M':
                        if (displayFlag)
                        {
                            displayFlag = false;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = true;
                            printf("\rDisplay ON");
                        }   
                        break;                        
                    case 'Q':
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %ld\r", (double)PID[0].kP, (double)PID[0].kI, (double)PID[0].kD, PID[0].PWMoffset);
                        break;                        
                    case 'T':
                        if (ADdisplay)
                        {
                            ADdisplay = false;
                            printf("\rPot display OFF");
                        }
                        else
                        {
                            ADdisplay = true;
                            printf("\rPot display ON");
                        }
                        break;                        
                    case 'V':
                        if (q) PID[3].Velocity = (float)floValue;
                        printf("Velocity: %0.3f", (double)PID[3].Velocity);
                        PID[2].Velocity = PID[3].Velocity;
                        break;                        
                    case 'Z':
                        if (q) PID[3].Destination = (long)floValue;                        
                        printf("Destination: %ld, Velocity: %0.3f", PID[3].Destination, (double)PID[3].Velocity);
                        break;                        
                    default:                        
                        if (command < ' ') printf("\rCommand: #%d => %c", command, (command - 1 + 'A'));
                        else printf("\rCommand: %c", command);
                        break;
                } // end switch command                
                command = 0;
            } // End if (command)
        } // End if HOSTRxBufferFull        
        
        
        // Maintain state machines of all polled MPLAB Harmony modules. 
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

    // Set up four analog inputs: AN0,AN1,AN2,AN3
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 1;
        AD1CSSLbits.CSSL1 = 1;
        AD1CSSLbits.CSSL2 = 1;
        AD1CSSLbits.CSSL3 = 1;
        AD1CSSLbits.CSSL4 = 1;
        AD1CSSLbits.CSSL5 = 1;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 0;
        AD1CSSLbits.CSSL9 = 0;
        AD1CSSLbits.CSSL10 = 0;
        AD1CSSLbits.CSSL11 = 0;
        AD1CSSLbits.CSSL12 = 0;
        AD1CSSLbits.CSSL13 = 0;
        AD1CSSLbits.CSSL14 = 0;
        AD1CSSLbits.CSSL15 = 0;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);   
        dummy = dummy;
                
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return(1);
    }





long POTcontrol(long servoID, struct PIDtype *PID)
{
    short Error;     
    short actualPosition;
    short commandPosition; 
    long totalDerError = 0;
    long derError;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    short i;
    static short displayCounter = 0;    
    unsigned short QuadReading = 0;
    
    commandPosition = PID[servoID].ADCommand; 
    PID[servoID].ADActual = (short)(ADresult[servoID+2]);  
    actualPosition = PID[servoID].ADActual;    
    
    if (PID[servoID].ADActual < QUAD_ONE) QuadReading = QUAD_ONE;
    else if (PID[servoID].ADActual < QUAD_TWO) QuadReading = QUAD_TWO;
    else QuadReading = QUAD_THREE;
    
    if (PID[servoID].PreviousQuad == NO_QUAD) 
        PID[servoID].PreviousQuad = QuadReading;
    else if (QuadReading == QUAD_TWO)
        PID[servoID].PreviousQuad = QUAD_TWO;
    else if (PID[servoID].PreviousQuad == QUAD_TWO)
        PID[servoID].PreviousQuad = QuadReading;
    else if (PID[servoID].PreviousQuad == QUAD_ONE)
    {
        if (QuadReading == QUAD_THREE) 
            actualPosition = PID[servoID].ADActual - (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
    }
    else if (PID[servoID].PreviousQuad == QUAD_THREE)
    {
        if (QuadReading == QUAD_ONE) 
            actualPosition = PID[servoID].ADActual + (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
    }
    
    Error = actualPosition - commandPosition;            
    PID[servoID].error[errIndex] = Error;
    
    // if (!PID[servoID].saturation) 
        PID[servoID].sumError = PID[servoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[servoID].error[i];
    derError = totalDerError / FILTERSIZE;    
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) ((PIDcorrection - PID[servoID].PWMoffset) / 2);
    else PID[servoID].PWMvalue = (long) ((PIDcorrection + PID[servoID].PWMoffset) / 2);
         
    if (PID[servoID].PWMvalue > PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;    
    
        displayCounter++;
        if (servoID == 0)
        {
            if (displayCounter >= 40 && displayFlag)
            {   
                if (PID[servoID].PreviousQuad == QUAD_ONE) printf("\rQI ");
                else if (PID[servoID].PreviousQuad == QUAD_TWO) printf("\rQII ");
                else printf("\rQIII ");
                printf("COM: %d, ROT: %d, ACT: %d ERR: %d P: %0.1f I: %0.1f PWM: %d ", (int)PID[servoID].ADCommand, (int)PID[servoID].ADActual, (int)actualPosition, (int)Error, (double)PCorr, (double)ICorr, (int)PID[servoID].PWMvalue);
                displayCounter = 0;
            }
        }
    return 1;
}

long ENCODERcontrol(long servoID, struct PIDtype *PID)
{
    long Error;         
    long lngCommandPos = 0;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    BYTE EncoderDirection;
    static short displayCounter = 0;
    short i;    
    float ActualVelocity = 0, AverageVelocity = 0;
    
    if (PID[servoID].Halted)
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }

#ifndef REV2
    if (servoID < 0)
    {
        PID[servoID].Halted = true;
        PID[servoID].PWMvalue = 0;
        return 0;
    }
#endif    
            
    lngCommandPos = (long) PID[servoID].CommandPos;    
        
    if (PID[servoID].Velocity < PID[servoID].TargetVelocity) 
    {
        PID[servoID].Velocity = PID[servoID].Velocity + ACCELERATION;
        if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;
    }
    else if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
    {
        PID[servoID].Velocity = PID[servoID].Velocity - ACCELERATION;
        if (PID[servoID].Velocity < PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;        
    }
           
    if (!PID[servoID].saturation) PID[servoID].CommandPos = PID[servoID].CommandPos + PID[servoID].Velocity;    
    
    if (PID[servoID].Mode == DESTINATION_MODE)
    {
        if (PID[servoID].Velocity > (float)0.0 && PID[servoID].CommandPos > PID[servoID].Destination)
                PID[servoID].CommandPos = PID[servoID].Destination;
        else if (PID[servoID].Velocity < (float)0.0 && PID[servoID].CommandPos < PID[servoID].Destination)
                PID[servoID].CommandPos = PID[servoID].Destination;
    }    
    
    
    if (servoID == 0)
    {
        EncoderDirection = !EncoderOneDir;        
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderOne;
            ActualVelocity = (float) EncoderOne;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderOne;
            ActualVelocity = (float) 0 - EncoderOne;
        }
        EncoderOne = 0;
    }    
    else if (servoID == 1)
    {
        EncoderDirection = EncoderTwoDir;
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderTwo;
            ActualVelocity = (float) EncoderTwo;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderTwo;
            ActualVelocity = (float) 0 - EncoderTwo;
        }
        EncoderTwo = 0;
    }    
    
    if (PID[servoID].Mode == DESTINATION_MODE)
    {
        if (PID[servoID].Destination >= 0) 
        {
            if (PID[servoID].ActualPos >= (long)(PID[servoID].Destination)) 
            {
                printf("\rHALTED: COM: %ld, ACT: %ld", (long)(PID[servoID].CommandPos), PID[servoID].ActualPos);
                PID[servoID].Halted = true;
                return 0;
            }
        }
        else
        {
            if (PID[servoID].ActualPos <= (long)(PID[servoID].Destination))
            {
                printf("\rHALTED: COM: %ld, ACT: %ld", (long)(PID[servoID].CommandPos), PID[servoID].ActualPos);
                PID[servoID].Halted = true;
                return 0;
            }            
        }
    }
    
    Error = PID[servoID].ActualPos - lngCommandPos;
         
    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error; 

    // derError = Error - PID[servoID].error[errIndex];
    PID[servoID].ActualVelocity[errIndex] = ActualVelocity;    
    AverageVelocity = 0;
    for (i = 0; i < FILTERSIZE; i++)
        AverageVelocity = AverageVelocity + PID[servoID].ActualVelocity[i];
    
    AverageVelocity = AverageVelocity / FILTERSIZE;        
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;        
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = (AverageVelocity - PID[servoID].Velocity) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
        
    if (PID[servoID].PWMvalue >= PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue <= -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;
    
    if (servoID == 2)
    {
        displayCounter++;
        if (displayCounter >= 20 && displayFlag)
        {
            displayCounter = 0;                     
            // printf("\r>VEL: %0.1f COM: %0.0f, ERR: %ld P: %0.1f I: %0.1f D: %0.1f PWM: %d ", PID[servoID].Velocity, PID[servoID].CommandPos, Error, PCorr, ICorr, DCorr, PID[servoID].PWMvalue);            
        }
    }       
    
    return 1;
}

void ResetPID()
{
    int i, j;
    
    EncoderOne = 0;
    EncoderTwo = 0;

    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0; // For 53:1 ratio Servo City motor
        PID[i].PWMvalue = 0;
        PID[i].ADActual = 0;
        PID[i].ADCommand = 0;        
        PID[i].saturation = false;
        PID[i].Velocity = 0;
        PID[i].ActualPos = 0;
        PID[i].CommandPos = 0;
        PID[i].Halted = false;
        PID[i].RemoteDataValid = false;
        PID[i].Mode = DEFAULT_MODE;
        PID[i].PreviousQuad = NO_QUAD;
        PID[i].Velocity = 0;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
    //PID[0].Mode = POT_MODE;
    //PID[1].Mode = POT_MODE;
    //PID[4].Mode = POT_MODE;
    errIndex = 0;
}

unsigned char ReadADC(unsigned short *ptrADResult, unsigned short *ptrADFiltered)
{
static unsigned char validData = FALSE;
static unsigned short ADbuffer[MAXPOTS][FILTERSIZE];
static short i = 0, j = 0, k = 0;
long SumADC = 0;
unsigned short ADaverage = 0;    

    if (!IFS0bits.AD1IF) return FALSE;

    // Read all AD inputs
    //IFS0bits.AD1IF = 0;
    //while(!IFS0bits.AD1IF);        
    AD1CON1bits.ASAM = 0;        // Pause sampling. 
    for (i = 0; i < MAXPOTS; i++)            
        ptrADFiltered[i] = ptrADResult[i] = ADbuffer[i][j] = (unsigned short) ADC_ResultGet(i); // read the result of channel 0 conversion from the idle buffer    
    AD1CON1bits.ASAM = 1;        // Restart sampling.          
    
    for (i = 0; i < MAXPOTS; i++)
    {
        SumADC = 0;
        for (k = 0; k < FILTERSIZE; k++)
            SumADC = SumADC + (long) ADbuffer[i][k];
        ADaverage = (unsigned short) (SumADC / FILTERSIZE);
        ptrADFiltered[i] = ADaverage;
    }
    
    j++;
    if (j >= FILTERSIZE) 
    {
        j = 0;
        validData = TRUE;
    }
    IFS0bits.AD1IF = 1;
    return validData;    
}

void InitPID()
{
    ResetPID();
    int i;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].kP = KP;
        PID[i].kI = KI;
        PID[i].kD = KD;
        PID[i].PWMoffset = PWM_OFFSET;
        PID[i].Mode = POT_MODE;
    }              
}

void controlLED(GPIO_PIN pin, uintptr_t context)
{
    if(INT0_Get() == SWITCH_PRESSED_STATE)
    {
        /* Turn ON LED */        
        //SPARE1_Set();
    }
    else
    {
        /* Turn OFF LED */        
        //SPARE1_Clear();
    }
}

void APP_WriteCallback(uintptr_t context)
{
    writeStatus = true;
}

void APP_ReadCallback(uintptr_t context)
{
    if(UART1_ErrorGet() != UART_ERROR_NONE)
    {
        /* ErrorGet clears errors, set error flag to notify console */
        errorStatus = true;
    }
    else
    {
        readStatus = true;
    }
}

void ClearUART1RxBuffer()
{
    short i;
    for (i = 0; i < MAXBUFFER+1; i++) UART1RxBuffer[MAXBUFFER+1] = '\0';
}

void DelayMs(long ms)
{
    SetDelay(ms * 10);
    while (GetDelay() != 0);    
}

void FillAtmelPage(unsigned char *ptrBuffer)
{
    static unsigned char TestByte = 0;
    int i;
    for (i = 0; i < ATMEL_PAGESIZE; i++)
    {
        ptrBuffer[i] = TestByte;
        TestByte = TestByte + 55;
    }
}
/*******************************************************************************
 End of File
*/

