#ifndef I2C_UTIL_H
#define I2C_UTIL_H

#ifdef __cplusplus 
extern "C" { 
#endif
    extern int i2c_read_reg(int fd, const unsigned char *buf, unsigned slave_address, unsigned reg_address, int len);
    extern int i2c_write_reg(int fd, const unsigned char *buf, unsigned slave_address, unsigned reg_address, int len);
    extern int i2c_read_status(int fd, const unsigned char *buf, const unsigned char * wbuf, unsigned slave_address, unsigned reg_address, int wlen);
    extern int i2c_read_Data(int fd, const unsigned char *buf, const unsigned char * wbuf, unsigned slave_address, unsigned reg_address, int wlen, int rlen);
    int I2C1_WriteData(int fd, unsigned char devI2CAddr, const unsigned char * buf, int len);
    int  I2C1_ReadData(int fd, unsigned char devI2CAddr, const unsigned char * buf,unsigned char * Addr, int len);
	int  I2C1_Read_Data(int fd, unsigned char devI2CAddr, const unsigned char * buf, int len);
#ifdef __cplusplus 
} 
#endif 

#endif