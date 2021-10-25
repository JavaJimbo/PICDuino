void I2C_wait_for_idle(void);
void I2C_start();
void I2C_stop();
void I2C_restart();
void I2C_ack(void);
void I2C_nack(void);
unsigned char I2C_write(unsigned char writeByte, char wait_ack);
unsigned char I2C_read(char ack_nack);
unsigned short GetI2C_Errors();
void I2C_init(double frequency);
unsigned char EEpromWriteByte(unsigned long address, unsigned char data);
unsigned char EEpromReadByte (unsigned long address, unsigned char *dataByte);
unsigned char EEpromWriteBlock(unsigned long address, unsigned char *ptrData, unsigned short NumBytes);
unsigned char EEpromReadBlock (unsigned long address, unsigned char *dataByte, unsigned short NumBytes);

#define NO_ACK 0
#define WAIT_FOR_ACK 1
#define WR_BIT 0
#define RD_BIT 1

#define EEPROM_ID 0xA0
#define I2C_ACK_ERROR 0x01
#define ENABLE_DIAGNOSTICS

#define USE_24LC1025
#ifdef USE_24LC1025
    #define BLOCKSIZE 128
#else 
    #define BLOCKSIZE 64 // TODO: FOR 24LC256 - chekc this
#endif
