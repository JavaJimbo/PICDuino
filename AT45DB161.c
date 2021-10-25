/* AT45DB161.C -    Routines for reading and writing to ATMEL AT45DB161 memory
 *                  Tested on LED MATRIX CONTROLLER V1.0 6-14-17
 *                  This is the modified version of AT45DB642.c for the smaller '161 Atmel
 *
 * 02-11-12 JBS:    Retooled version with latest corrections
 * 05-01-13 JBS:    Tested 0-4095 page writes
 *                  Swapped in pointers in place of global arrays
 *                  Added write and read line routines.
 * 5-26-14 JBS:     Adapted for PIC32 & MPLABX.
 *                  Added initSPI(), WriteAtmelRAM(), ReadAtmelRAM()
 * 6-6-14           INVERTED return values so functions return TRUE or FALSE instead of error.
 * 12-16-14         Works with LED COntroller Board - SPI #2
 *                  initSPI() commented out below because it isn't working
 * 5-8-15           Renamed Atmel functions to RAM and FLASH
 * 5-13-15          Eliminated array length check for reading and writing bytes (lines)
 * 5-13-15          Added Buffer 1 and Buffer 2 reads/writes `A
 * 6-14-17          Added corrected version of initAtmelSPI()
 *                  Renamed ReadAtmelPage(), WriteAtmelPage(), eliminated line routines.
 *                  Simplified page addressing, renamed routines
 * 6-15-17          Minor corrections and cleanup
 * 8-28-17          Minimal modifications for Two Motor Board - use SPI #1
 * 7-27-21          Updated for Microchip compiler XC32 V3.01 and PIC170F256D
 * 8-3-21           Fixed bug - couldn't write and read full PAGESIZE
 */
#include "definitions.h"                // SYS function prototypes
#include "AT45DB161.h"

union {
    unsigned char byte[2];
    unsigned short integer;
} convert;

#define lowByte byte[0]
#define highByte byte[1]  



unsigned char SendReceiveSPI(unsigned char data)
{
    SPI1BUF = data; // Send byte to SPI2BUF.
    while (SPI1STATbits.SPIRBE); // Wait for 
    return SPI1BUF; // Return byte in SPI2BUF
}




// This function writes an entire 528 byte page to the Atmel memory buffer.
// *buffer points to input array.
int WriteAtmelPage (unsigned char bufferNum, unsigned char *buffer){
int i;

 	if (buffer==NULL) return(FALSE); // ERROR

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.
    WRITE_PROTECT_OFF();
	ATMEL_CS=0;							// select EEPROM as target
    if (bufferNum == 1) SendReceiveSPI(0x84);	// Command: Buffer #1 write 
    else SendReceiveSPI(0x87);			// Command: Buffer #2 write 
	SendReceiveSPI(0);				// Address
	SendReceiveSPI(0);
	SendReceiveSPI(0);

	for(i=0; i<ATMEL_PAGESIZE; i++)	// write the data to Atmel buffer
		SendReceiveSPI(buffer[i]);
	ATMEL_CS=1;
    WRITE_PROTECT_ON();    
    IntializeAtmelTimeout(PROGRAMMING_TIMEOUT);    
	return(TRUE); // No errors - write was successful
}

int ReadAtmelPage(unsigned char bufferNum, unsigned char *buffer){
unsigned char dataIn;
int i;

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.

	ATMEL_CS=0;
    if (bufferNum == 1) SendReceiveSPI(0xD4);    // Buffer #1 read command 
    else SendReceiveSPI(0xD6);			// Buffer #2 read command
	SendReceiveSPI(0);
	SendReceiveSPI(0);
	SendReceiveSPI(0);
	SendReceiveSPI(0);				// Additional don't care byte

	for(i=0; i<ATMEL_PAGESIZE; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;
	}
	ATMEL_CS=1;
    IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);
	return(TRUE); // No errors - read was successful
}

// This erases one sector or 256 pages on the Atmel.
// Sectors 1 to 15 can be erased using this routine.
// Sectors 0a and 0b must be erased using a separate routine.
// The routine returns FALSE if sector number is out of bounds, otherwise returns TRUE
int EraseFLASHsector(unsigned char sector){
unsigned char sectorByte;

	if (sector>MAX_SECTOR) return(FALSE);
	sectorByte = (sector<<2) & 0b00111100;	  

	AtmelBusy(1);				// Read Status register and make sure Atmel isn't busy with any previous activity.
    WRITE_PROTECT_OFF();
	ATMEL_CS=0;						
	SendReceiveSPI(0x7C);			// erase sector command
	SendReceiveSPI(sectorByte);		// sector shifted up two bits
	SendReceiveSPI(0x00);			// Don't care byte
	SendReceiveSPI(0x00);			// Don't care byte		
	ATMEL_CS=1;		
    WRITE_PROTECT_ON();    
    IntializeAtmelTimeout(SECTOR_ERASE_TIMEOUT);
    // AtmelBusy(1);				// Wait until erasing is all done

	return(TRUE);
}

// This routine erases the entire flash memory
int EraseEntireFLASH(void){

	AtmelBusy(1);	// Read Status register and make sure Atmel isn't busy with any previous activity.
    WRITE_PROTECT_OFF();
	ATMEL_CS=0;						
	SendReceiveSPI(0xC7);
	SendReceiveSPI(0x94);	
	SendReceiveSPI(0x80);	
	SendReceiveSPI(0x9A);		
	ATMEL_CS=1;					
    WRITE_PROTECT_ON();
    IntializeAtmelTimeout(CHIP_ERADE_TIMEOUT);        
	return(TRUE);
}



// This routine reads the Atmel memory status register
// to check the READY/BUSY flag.
// This flag goes high when a page programming operation 
// has completed. It returns TRUE if complete
// and FALSE otherwise.
//
// If waitFlag is a 1 then it will sit in a loop
// and keep going until the status register
// READY/BUSY flag reads high.
int AtmelBusy (unsigned char waitFlag)
{	
unsigned char status, inByte;
		
	ATMEL_CS=0;		    
	SendReceiveSPI(0xD7);	// Send read Status register command		
    inByte = (unsigned char) SendReceiveSPI(0x00);
    status = (0x80 & inByte); 

    if (waitFlag)
    {
        SetAtmelTimeout();;
    	while (!status && !GetMemoryTimeout())  // Keep looping until status bit goes high
        // while (!status)  // Keep looping until status bit goes high
        {
        	inByte = (unsigned char) SendReceiveSPI(0x00);
    		status = (0x80 & inByte); 
        } 
        if (!status) printf("\rMEMORY TIMEOUT");
    }
	ATMEL_CS=1;    
    return (int) status;
}

// This programs the buffer into a previously erased page in flash memory.
// The Atmel memory buffer is #1 or #2, specified by bufferNum
// The pages are numbered 0-4095.
// The routine always returns 0, however it can be modified to return an error code as well.
unsigned char ProgramFLASH (unsigned char bufferNum, unsigned short pageNum)
{

    convert.integer = (pageNum << 2) & 0xFFFC;

	AtmelBusy(1);					// Read Status register and make sure Atmel isn't busy with any previous activity.	
    WRITE_PROTECT_OFF();
	ATMEL_CS=0;						// select eprom as target
    if (bufferNum == 1)  SendReceiveSPI(0x88);       // Buffer 1 page program without built-in erase     
    else SendReceiveSPI(0x89);                          // Buffer 2 page program without built-in erase
    
    //if (bufferNum == 1)  SendReceiveSPI(0x83);       // Buffer 1 page program WITH built-in erase 
    //else SendReceiveSPI(0x86);                          // Buffer 2 page program WITH built-in erase
    
    SendReceiveSPI(convert.highByte);
    SendReceiveSPI(convert.lowByte);
	SendReceiveSPI(0);				// 
	ATMEL_CS=1;						// disable eprom as target
    WRITE_PROTECT_ON();    
    IntializeAtmelTimeout(PROGRAMMING_TIMEOUT);	
	return(TRUE);
}


// This copies a page into Atmel Buffer 1
// The pages are numbered 0-4095.
// The routine returns FALSE page is out of bounds, otherwise returns TRUE
unsigned char TransferFLASH (unsigned char bufferNum, unsigned short pageNum){

	if (pageNum>MAX_PAGE) return(FALSE);
        
    convert.integer = (pageNum << 2) & 0xFFFC;        

	AtmelBusy(1);					// Read Status register and make sure Atmel isn't busy with any previous activity.	

	ATMEL_CS=0;			
    if (bufferNum == 1) SendReceiveSPI(0x53);        // Command: FLASH to RAM buffer 1 transfer
    else SendReceiveSPI(0x55);                          // Command: FLASH to RAM buffer 2 transfer $$$$    
    SendReceiveSPI(convert.highByte);
    SendReceiveSPI(convert.lowByte);
	SendReceiveSPI(0);    
	ATMEL_CS=1;	
    IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);	
	return(TRUE);
}

// This erases one page in flash memory.
// The pages are numbered 0-4095.
// The routine returns ERROR of page is out of bounds, otherwise returns 0.
unsigned char EraseFLASHpage (unsigned short pageNum)
{
    if (pageNum>MAX_PAGE) return(FALSE);        
    convert.integer = (pageNum << 2) & 0xFFFC;            
	AtmelBusy(1);					// Read Status register and make sure Atmel isn't busy with any previous activity.	
    WRITE_PROTECT_OFF();    
	ATMEL_CS=0;			    
    SendReceiveSPI(0x81);		// Page erase
    SendReceiveSPI(convert.highByte);
    SendReceiveSPI(convert.lowByte);
	SendReceiveSPI(0);    
	ATMEL_CS=1;	    
    WRITE_PROTECT_ON();    
    IntializeAtmelTimeout(PAGE_ERASE_TIMEOUT);    
	return(TRUE);
}



int ReadAtmelBytes (unsigned char bufferNum, unsigned char *buffer, unsigned short bufferAddress, unsigned short numberOfBytes){
unsigned char dataIn;
int i;

	if ((bufferAddress+numberOfBytes) > ATMEL_PAGESIZE) return(FALSE); // Make sure we don't overrun the Atmel page buffer	

    convert.integer = bufferAddress & 0x03FF;    

    IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);    
	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	
		
	ATMEL_CS = 0;						
	if (bufferNum == 1) SendReceiveSPI(0xD4);			// Buffer #1 read command was 0xD4
    else SendReceiveSPI(0xD6);	// Buffer #2 read command  was 0xD6  
	SendReceiveSPI(0);
	SendReceiveSPI(convert.highByte);
	SendReceiveSPI(convert.lowByte);
	SendReceiveSPI(0);				// Additional don't care byte	
	
	for(i=0; i<numberOfBytes; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;		
	}
	ATMEL_CS = 1;
    IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);
	return (TRUE); // No errors - read was successful
}

// This function writes to either Atmel memory buffer #1 or #2, specified by bufferNum
// *buffer points to input array.	
// The "numberOfBytes" input is the number of bytes that will be 
// copied from the AtmelWriteArray[] array to the Atmel.
// If there are no errors, it returns TRUE
int WriteAtmelBytes (unsigned char bufferNum, unsigned char *buffer, unsigned short bufferAddress, unsigned short numberOfBytes)
{
int i;
 
 	if (buffer==NULL) return FALSE ; 
 
	if ((bufferAddress+numberOfBytes) > ATMEL_PAGESIZE) return(BUFFER_OVERRRUN); // Make sure we don't overrun the Atmel page buffer	

    convert.integer = bufferAddress & 0x03FF;

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	
    WRITE_PROTECT_OFF();
	ATMEL_CS=0;							// select EEPROM as target
	if (bufferNum == 1) SendReceiveSPI(0x84);			// Command: Buffer #1 write
    else SendReceiveSPI(0x87);			// Command: Buffer #2 write
	SendReceiveSPI(0);				// Address
	SendReceiveSPI(convert.highByte);
	SendReceiveSPI(convert.lowByte);    
	
	for(i=0; i<numberOfBytes; i++)	// write the data to Atmel buffer	
		SendReceiveSPI(buffer[i]);	
	ATMEL_CS=1;		
    WRITE_PROTECT_ON();    
    IntializeAtmelTimeout(PROGRAMMING_TIMEOUT);
	return(TRUE); // No errors - write was successful
}

int MainMemoryPageRead (unsigned char *buffer, unsigned short pageNum, unsigned short bufferAddress, unsigned short numberOfBytes){
unsigned char dataIn;
unsigned short lowAddress, midAddress, highAddress;
int i;

    // Make sure we don't overrun the Atmel page buffer
	if ((bufferAddress+numberOfBytes) >= ATMEL_PAGESIZE)  
    
    if (buffer==NULL) return FALSE;

    convert.integer = bufferAddress;
    lowAddress = convert.lowByte;
    midAddress = convert.highByte & 0x03;

    convert.integer = (pageNum << 2) & 0x3FFC;
    midAddress = midAddress | convert.lowByte;
    highAddress = convert.highByte;
     
    // Make sure Atmel isn't busy with any previous activity.	    
	AtmelBusy(1);						
		
	ATMEL_CS = 0;				
    // Send Main Memory Page Read command
    SendReceiveSPI(0xD2);		
    // Send page number combined with buffer address
	SendReceiveSPI((unsigned char) highAddress);
    SendReceiveSPI((unsigned char) midAddress);
    SendReceiveSPI((unsigned char) lowAddress);
    SendReceiveSPI(0);  // Send four don't care bytes
    SendReceiveSPI(0);
    SendReceiveSPI(0);
    SendReceiveSPI(0);
	
    // Now read incoming data one byte at a time
	for(i=0; i<numberOfBytes; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;		
	}
	ATMEL_CS = 1;
    IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);
	return (TRUE); // No errors - read was successful
}



unsigned char storeShortToAtmel(unsigned short pageNum, unsigned short address, unsigned short inData) {
unsigned char AtmelRAM[2];

    if (address < (ATMEL_PAGESIZE - 1)) {
        convert.integer = inData;
        AtmelRAM[0] = convert.lowByte;
        AtmelRAM[1] = convert.highByte;
        // Store flash in RAM, Erase flash, overwrite RAM, program flash:
        TransferFLASH (1, pageNum); 
        EraseFLASHpage(pageNum);
        WriteAtmelBytes (1, AtmelRAM, address, 2);
        ProgramFLASH(1, pageNum);
        IntializeAtmelTimeout(PROGRAMMING_TIMEOUT);
        return TRUE;
    } else return FALSE;
}

unsigned short fetchShortFromAtmel(unsigned short pageNum, unsigned short address) {
unsigned char AtmelRAM[2];

    if (address < (ATMEL_PAGESIZE - 1)) {
        // Read two bytes directly from flash without using RAM buffers:
        TransferFLASH (1, pageNum);
        ReadAtmelBytes (1, AtmelRAM, address, 2);
        // Convert two bytes to integer and return result;
        convert.lowByte = AtmelRAM[0];
        convert.highByte = AtmelRAM[1];     
        IntializeAtmelTimeout(DEFAULT_ATMEL_TIMEOUT);
        return (convert.integer);
    } else return (0);
}

