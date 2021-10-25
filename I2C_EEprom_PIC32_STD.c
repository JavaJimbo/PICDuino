/* I2C_EEprom_PIC32_STD.c
 * For PIC 32MX170F256D 
 * Compiler: XC32 V3.01 IDE: MPLABX V5.45 
 * 
 * 7-4-21: 
 * 7-31-21: Modified I2C routines to work with 24LC1025 and up to four devices 
 * 8-1-21:  Reading and writing with no delays, just check status. Tested with four 24LC1025 ICs.
 */

#include "I2C_EEprom_PIC32_STD.h"
#include "definitions.h"


unsigned short I2C_Errors = 0;



void DDelayMs(long ms)
{
    SetDelay(ms * 10);
    while (GetDelay() != 0);    
}

void I2C_wait_for_idle(void)
{
    SetI2CTimeout();
    while(I2C1CON & 0x1F && !GetMemoryTimeout()); // Acknowledge sequence not in progress
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0002;
                                // Receive sequence not in progress
                                // Stop condition not in progress
                                // Repeated Start condition not in progress
                                // Start condition not in progress
    SetI2CTimeout();
    while(I2C1STATbits.TRSTAT && !GetMemoryTimeout()); // Bit = 0 ? Master transmit is not in progress
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0004;
}

// I2C_start() sends a start condition  
void I2C_start()
{
    I2C_wait_for_idle();
    I2C1CONbits.SEN = 1;   
    SetI2CTimeout();
    while (I2C1CONbits.SEN == 1 && !GetMemoryTimeout());
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0001;
}

// I2C_stop() sends a stop condition  
void I2C_stop()
{
    I2C_wait_for_idle();
    I2C1CONbits.PEN = 1;
}

// I2C_restart() sends a repeated start/restart condition
void I2C_restart()
{
    I2C_wait_for_idle();
    I2C1CONbits.RSEN = 1;
    SetI2CTimeout();
    while (I2C1CONbits.RSEN == 1 && !GetMemoryTimeout());
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0008;
}

// I2C_ack() sends an ACK condition
void I2C_ack(void)
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 0; // Clear to send ACK bit
    I2C1CONbits.ACKEN = 1; // Send ACK bit, will be automatically cleared by hardware when sent  
    SetI2CTimeout();
    while(I2C1CONbits.ACKEN && !GetMemoryTimeout()); // Wait until ACKEN bit is cleared, meaning ACK bit has been sent
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0010;
}

// I2C_nack() sends a NACK condition
void I2C_nack(void) // Acknowledge Data bit
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 1; // Set to send NACK bit
    I2C1CONbits.ACKEN = 1; // Send NACK bit, will be automatically cleared by hardware when sent  
    SetI2CTimeout();
    while(I2C1CONbits.ACKEN && !GetMemoryTimeout()); // Wait until ACKEN bit is cleared, meaning NACK bit has been sent
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0020;
}

// address is I2C slave address, set wait_ack to 1 to wait for ACK bit or anything else to skip ACK checking  
unsigned char I2C_write(unsigned char writeByte, char wait_ack)
{
    I2C1TRN = writeByte | 0;				// Send slave address with Read/Write bit cleared
    SetI2CTimeout();
    while (I2C1STATbits.TBF == 1 && !GetMemoryTimeout());		// Wait until transmit buffer is empty
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0040;
    I2C_wait_for_idle();				// Wait until I2C bus is idle        
    if (wait_ack) 
    {
        SetI2CTimeout();
        while (I2C1STATbits.ACKSTAT && !GetMemoryTimeout()); // Wait until ACK is received  
    }    
    if (GetMemoryTimeout()) return FALSE;// I2C_Errors = I2C_Errors | 0x0080;
    else return TRUE;
}

// value is the value of the data we want to send, set ack_nack to 0 to send an ACK or anything else to send a NACK  
unsigned char I2C_read(char ack)
{
unsigned char readByte = 0;
    I2C1CONbits.RCEN = 1;				// Receive enable
    SetI2CTimeout();    
    while (I2C1CONbits.RCEN && !GetMemoryTimeout());			// Wait until RCEN is cleared (automatic)  
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0100;
    SetI2CTimeout();
    while (!I2C1STATbits.RBF && !GetMemoryTimeout());    		// Wait until Receive Buffer is Full (RBF flag)  
    if (GetMemoryTimeout()) I2C_Errors = I2C_Errors | 0x0200;
    readByte = I2C1RCV;    				// Retrieve value from I2C1RCV            
    if (ack) I2C_ack();
    else I2C_nack();
    return readByte;
}

unsigned short GetI2C_Errors()
{
    return I2C_Errors;
}

// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
void I2C_init(double frequency)
{
    double BRG;
    
    I2C1CON = 0;			// Turn off I2C1 module
    I2C1CONbits.DISSLW = 1; // Disable slew rate for 100kHz
    
    // BRG = (1 / (2 * frequency)) - 0.000000104;  ????
    BRG = (1 / (frequency)) - 0.000000104;
    BRG *= (SYS_FREQ / 2) - 2;    
    
    I2C1BRG = (int)BRG;		// Set baud rate
    I2C1CONbits.ON = 1;		// Turn on I2C1 module
}

unsigned char EEpromReadBlock (unsigned long address, unsigned char *dataByte, unsigned short NumBytes)
{
    unsigned long addressStart = 0, byteIndex = 0;
    unsigned char addressHigh = 0, addressLow = 0, controlByte = 0, BlockBit = 0;
    unsigned short retryCounter = 0;
    unsigned char device = 0x00;

    
    I2C_Errors = 0x0000;
    BlockBit = 0;              
    byteIndex = 0;    
    
    device = (unsigned char) (address / MAX_ADDRESS); 
    // printf("\rREAD address: %ld, Device: %d", address, (int)device);
    
    do {
        if (device >= MAX_DEVICE) 
        {
            I2C_Errors = I2C_Errors | 0x4000;
#ifdef  ENABLE_DIAGNOSTICS            
            printf("\rAddress overrun: EE device #%d, READ address: %ld, Error: %04X", (int)device, address, I2C_Errors);           
#endif             
            break;            
        }            
        addressStart = (address + byteIndex) % MAX_ADDRESS;
        addressHigh = (unsigned char) ((addressStart & 0xFF00) >> 8);
        addressLow = (unsigned char) (addressStart & 0x00FF);

#ifdef  USE_24LC1025        
        if (addressStart & 0x010000) BlockBit = 0b00001000;
        else BlockBit = 0;      
#else        
        BlockBit = 0;
#endif        
        controlByte = (device << 1) | WR_BIT | EEPROM_ID | BlockBit;        
        
        retryCounter = 0;
        do {        
            I2C_start();
            // Send EEPROM ID Or'd with device address and WRITE bit:
        } while (!I2C_write (controlByte, WAIT_FOR_ACK && retryCounter++ < 1000));
        
#ifdef  ENABLE_DIAGNOSTICS            
            //printf("\rREAD Retry: %d, DEV: %d, CTRL: %02X", retryCounter, device, controlByte);
#endif                

        if (retryCounter >= 1000)
        {
            I2C_Errors = I2C_Errors | 0x0080;
#ifdef  ENABLE_DIAGNOSTICS            
            printf("\rREAD BLOCK Errors: %04X", I2C_Errors);
#endif            
            I2C_stop();
            return(FALSE);
        }
      
        // Send EEprom high address byte:
        I2C_write (addressHigh, WAIT_FOR_ACK);
    
        // Send EEprom low address byte:
        I2C_write (addressLow, WAIT_FOR_ACK);
        
        I2C_restart();
    
        // Send EEPROM ID Or'd with device address and READ bit:
        controlByte = (device << 1) | RD_BIT | EEPROM_ID | BlockBit; 
        I2C_write (controlByte, WAIT_FOR_ACK);
        
        // Send data to write
        while (byteIndex < NumBytes - 1) 
        {
            if (((addressStart + byteIndex + 1) % BLOCKSIZE) == 0) break;            
            dataByte[byteIndex++] = I2C_read(WAIT_FOR_ACK);
        }
        dataByte[byteIndex++] = I2C_read(NO_ACK);                
        I2C_stop();
        if ((byteIndex % MAX_ADDRESS) == 0) device++;       
    } while (byteIndex < NumBytes);
    
    if (byteIndex != NumBytes) I2C_Errors = I2C_Errors | 0x8000;
    
#ifdef ENABLE_DIAGNOSTICS
    if (I2C_Errors) printf("\rREAD BLOCK Errors: %04X", I2C_Errors);
#endif         
    if (I2C_Errors) 
    {
        DDelayMs(10);
        I2C_stop();
        return(FALSE);
    }
    else return(TRUE);
}

unsigned char EEpromWriteBlock(unsigned long address, unsigned char *ptrData, unsigned short NumBytes)
{
    unsigned long addressStart = 0, byteIndex = 0;
    unsigned char addressHigh = 0, addressLow = 0, controlByte = 0, BlockBit = 0;    
    unsigned short retryCounter = 0;
    unsigned char device = 0x00;
    
    I2C_Errors = 0x0000;
    byteIndex = 0;
    
    device = (unsigned char) (address / MAX_ADDRESS);    
    
    do {
        if (device >= MAX_DEVICE) 
        {
            I2C_Errors = I2C_Errors | 0x4000;
#ifdef  ENABLE_DIAGNOSTICS            
            printf("\rAddress overrun: EE device #%d, WRITE address: %ld, Error: %04X", (int)device, address, I2C_Errors);
#endif             
            break;            
        }                    
        addressStart = (address + byteIndex) % MAX_ADDRESS;        
        addressHigh = (unsigned char) ((addressStart & 0xFF00) >> 8);
        addressLow = (unsigned char) (addressStart & 0x00FF);
        
#ifdef  USE_24LC1025        
        if (addressStart & 0x010000) BlockBit = 0b00001000;
        else BlockBit = 0;      
#else        
        BlockBit = 0;
#endif        
        controlByte = (device << 1) | WR_BIT | EEPROM_ID | BlockBit;        
        
                   
        retryCounter = 0;
        do {        
            I2C_start();
            // Send EEPROM ID Or'd with device address and WRITE bit:
        } while (!I2C_write (controlByte, WAIT_FOR_ACK && retryCounter++ < 1000));
        
#ifdef  ENABLE_DIAGNOSTICS            
        // printf("\rWRITE Retry: %d, DEV: %d, CTRL: %02X", retryCounter, device, controlByte);
#endif         
        
        if (retryCounter >= 1000)
        {
            I2C_Errors = I2C_Errors | 0x0080;
#ifdef  ENABLE_DIAGNOSTICS            
            printf("\rWRITE BLOCK Errors: %04X", I2C_Errors);
#endif            
            I2C_stop();
            return(FALSE);
        }
      
        // Send EEprom high address byte:
        I2C_write (addressHigh, WAIT_FOR_ACK);
    
        // Send EEprom low address byte:
        I2C_write (addressLow, WAIT_FOR_ACK);
    
        // Send data to write
        while (byteIndex < NumBytes) 
        {
            if (((addressStart + byteIndex) % BLOCKSIZE) == 0 && byteIndex > 0) break;
            I2C_write (ptrData[byteIndex++], WAIT_FOR_ACK);   
        }
        I2C_stop();
        if ((byteIndex % MAX_ADDRESS) == 0) device++;
    } while (byteIndex < NumBytes);
    
    if (byteIndex != NumBytes) I2C_Errors = I2C_Errors | 0x8000;    
    
#ifdef ENABLE_DIAGNOSTICS
    if (I2C_Errors) printf("\rWRITE BLOCK Errors: %04X", I2C_Errors);
#endif         
    if (I2C_Errors) 
    {
        DDelayMs(10);
        I2C_stop();
        return(FALSE);
    }
    else return(TRUE);
}


unsigned char EEpromWriteByte(unsigned long address, unsigned char data)
{
    unsigned long addressStart = 0;
    unsigned char addressHigh = 0, addressLow = 0, controlByte = 0, BlockBit = 0, device = 0x00;    
    
    I2C_Errors = 0x0000;
    
    addressStart = address % MAX_ADDRESS;   
    
    device = (unsigned char)(addressStart / MAX_ADDRESS);
    if (device >= MAX_DEVICE) 
    {
        I2C_Errors = I2C_Errors | 0x4000;
        DDelayMs(10);
        I2C_stop();
        return(FALSE);        
    }
  
    addressHigh = (unsigned char) ((addressStart & 0xFF00) >> 8);
    addressLow = (unsigned char) (addressStart & 0x00FF);
    
    I2C_start();
    
    // Send EEPROM ID Or'd with device address with WRITE bit:
 #ifdef    USE_24LC1025        
        if (addressStart & 0x010000) BlockBit = 0b00001000;
        else BlockBit = 0;      
#else        
        BlockBit = 0;
#endif     
    controlByte = (device << 1) | WR_BIT | EEPROM_ID | BlockBit;
    I2C_write (controlByte, WAIT_FOR_ACK);
      
    // Send EEprom high address byte:
    I2C_write (addressHigh, WAIT_FOR_ACK);
    
    // Send EEprom low address byte:
    I2C_write (addressLow, WAIT_FOR_ACK);
    
    // Send data to write
    I2C_write (data, WAIT_FOR_ACK);   

    I2C_stop();
    // EEPROM_WP = 1;    
    
    if (I2C_Errors) 
    {
        DDelayMs(10);
        I2C_stop();
        return(FALSE);
    }
    else return(TRUE);
}

unsigned char EEpromReadByte (unsigned long address, unsigned char *dataByte) 
{
    unsigned long addressStart = 0;
    unsigned char addressHigh = 0, addressLow = 0, controlByte = 0, BlockBit = 0, device = 0x00;    
    
    I2C_Errors = 0x0000;
    
    addressStart = address % MAX_ADDRESS;
    device = (unsigned char)(addressStart / MAX_ADDRESS);
    if (device >= MAX_DEVICE) 
    {
        I2C_Errors = I2C_Errors | 0x4000;
        DDelayMs(10);
        I2C_stop();
        return(FALSE);        
    }    

    addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
    addressLow = (unsigned char) (address & 0x00FF);
   
    I2C_start();
    
    // Send EEPROM ID Or'd with device address with WRITE bit:
 #ifdef    USE_24LC1025        
        if (addressStart & 0x010000) BlockBit = 0b00001000;
        else BlockBit = 0;      
#else        
        BlockBit = 0;
#endif     
    controlByte = (device << 1) | WR_BIT | EEPROM_ID | BlockBit;
    I2C_write (controlByte, WAIT_FOR_ACK);    
      
    // Send EEprom high address byte:
    I2C_write (addressHigh, WAIT_FOR_ACK);
    
    // Send EEprom low address byte:
    I2C_write (addressLow, WAIT_FOR_ACK);
    
    I2C_restart();

    // Send EEPROM ID Or'd with device address with READ bit:
    controlByte = (device << 1) | RD_BIT | EEPROM_ID | BlockBit;
    I2C_write (controlByte, WAIT_FOR_ACK);
    
    *dataByte = I2C_read(NO_ACK);

    I2C_stop();
    // EEPROM_WP = 1;    
#ifdef ENABLE_DIAGNOSTICS
    if (I2C_Errors) printf("\rWRITE Errors: %04X", I2C_Errors);
#endif    
    if (I2C_Errors) return(FALSE);
    else return(TRUE);
}
