
#include "i2c-util.h"
#include <stdio.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

int i2c_read_status(int fd, const unsigned char *buf, const unsigned char * wbuf, unsigned slave_address, unsigned reg_address, int wlen)
{
	struct i2c_rdwr_ioctl_data work_queue;
	unsigned char w_buf[wlen+1];
	w_buf[0] = reg_address;
	int ret;

	work_queue.nmsgs = 2;
	work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
	if (!work_queue.msgs) {
		printf("i2c_read_status: Memory alloc error\n");
		return -1;
	}

	// ioctl(fd, I2C_TIMEOUT, 2);
	ioctl(fd, I2C_RETRIES, 1);

	(work_queue.msgs[0]).flags = 0;
	(work_queue.msgs[0]).len = wlen + 1;
	(work_queue.msgs[0]).addr = slave_address;
	(work_queue.msgs[0]).buf = w_buf;
	memcpy(w_buf + 1, wbuf, wlen);

	(work_queue.msgs[1]).len = 2;
	(work_queue.msgs[1]).flags = I2C_M_RD;
	(work_queue.msgs[1]).addr = slave_address;
	(work_queue.msgs[1]).buf = buf;

	ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
    //printf("i2c_read_status: ioctl (ret=%d, errno=%d)\n", ret, errno);
    free(work_queue.msgs);

    return ret;
}

int i2c_read_Data(int fd, const unsigned char *buf, const unsigned char * wbuf, unsigned slave_address, unsigned reg_address, int wlen, int rlen)
{
	struct i2c_rdwr_ioctl_data work_queue;
	unsigned char w_buf[wlen+1];
	w_buf[0] = reg_address;
	int ret;

	work_queue.nmsgs = 2;
	work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
	if (!work_queue.msgs) {
		printf("i2c_read_Data: Memory alloc error\n");
		return -1;
	}

	// ioctl(fd, I2C_TIMEOUT, 2);
	ioctl(fd, I2C_RETRIES, 1);

	(work_queue.msgs[0]).flags = 0;
	(work_queue.msgs[0]).len = wlen + 1;
	(work_queue.msgs[0]).addr = slave_address;
	(work_queue.msgs[0]).buf = w_buf;
	memcpy(w_buf + 1, wbuf, wlen);

	(work_queue.msgs[1]).len = rlen;
	(work_queue.msgs[1]).flags = I2C_M_RD;
	(work_queue.msgs[1]).addr = slave_address;
	(work_queue.msgs[1]).buf = buf;

	ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
    //printf("i2c_read_Data: ioctl (ret=%d, errno=%d)\n", ret, errno);
    free(work_queue.msgs);

    return ret;
}

int i2c_read_reg(int fd, const unsigned char *buf, unsigned  slave_address, unsigned  reg_address, int len)
{
	struct i2c_rdwr_ioctl_data work_queue;
	unsigned char w_val = reg_address;
	int ret;

	work_queue.nmsgs = 2;
	work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
	if (!work_queue.msgs) {
		printf("i2c_read_reg: Memory alloc error\n");
		return -1;
	}

	// ioctl(fd, I2C_TIMEOUT, 2);
	ioctl(fd, I2C_RETRIES, 1);

	(work_queue.msgs[0]).flags = 0;
	(work_queue.msgs[0]).len = 1;
	(work_queue.msgs[0]).addr = slave_address;
	(work_queue.msgs[0]).buf = &w_val;

	(work_queue.msgs[1]).len = len;
	(work_queue.msgs[1]).flags = I2C_M_RD;
	(work_queue.msgs[1]).addr = slave_address;
	(work_queue.msgs[1]).buf = buf;

	ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
	  if(ret < 0)
      printf("i2c_read_reg: ioctl (ret=%d, errno=%d)\n", ret, errno);
    free(work_queue.msgs);

    return ret;
}


int I2C1_WriteData(int fd, unsigned char devI2CAddr, const unsigned char * buf, int len)
{

	//int ret =  i2c_write_reg("/dev/i2c-0", &buf[1], devI2CAddr, buf[0],  (len-1));
	//printf(" -------------I2C1_WriteData :   start[%2x],  end[%2x],  len[%2d]\n",buf[0], buf[len-1], len );
	//int i2c_write_reg(char *dev, unsigned char *buf, unsigned slave_address, unsigned reg_address, int len)

	struct i2c_rdwr_ioctl_data work_queue;
	unsigned char w_buf[len+1];
	int ret;

	work_queue.nmsgs = 1;
	work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
	if (!work_queue.msgs) {
		printf("I2C1_WriteData: Memory alloc error\n");
		return -1;
	}

	// ioctl(fd, I2C_TIMEOUT, 2);
	ioctl(fd, I2C_RETRIES, 1);

	(work_queue.msgs[0]).flags = 0;
	(work_queue.msgs[0]).len =  len;
	(work_queue.msgs[0]).addr = devI2CAddr;
	(work_queue.msgs[0]).buf = w_buf;

	memcpy(w_buf , buf, len);

	ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
    //printf("I2C1_WriteData: ioctl (ret=%d, errno=%d)\n", ret, errno);
    free(work_queue.msgs);

    return ret;
}


int I2C1_ReadData(int fd, unsigned char devI2CAddr, const unsigned char * buf,unsigned char * Addr,int len)
{
		struct i2c_rdwr_ioctl_data work_queue;
		int ret;
		char reg_addr = 0;

		work_queue.nmsgs = 2;
		work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
		if (!work_queue.msgs) {
            printf("I2C1_ReadData: Memory alloc error\n");
			return -1;
		}

		// ioctl(fd, I2C_TIMEOUT, 2);
		ioctl(fd, I2C_RETRIES, 1);

		(work_queue.msgs[0]).flags = 0;
		(work_queue.msgs[0]).len = 3;
		(work_queue.msgs[0]).addr = devI2CAddr;
		(work_queue.msgs[0]).buf = Addr;

		(work_queue.msgs[1]).len = len;
		(work_queue.msgs[1]).flags = I2C_M_RD;
		(work_queue.msgs[1]).addr = devI2CAddr;
		(work_queue.msgs[1]).buf = buf;

		ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
        //printf("I2C1_ReadData: ioctl (ret=%d, errno=%d)\n", ret, errno);
        free(work_queue.msgs);

        return ret;
}
int I2C1_Read_Data(int fd, unsigned char devI2CAddr, const unsigned char * buf,int len)
{
		struct i2c_rdwr_ioctl_data work_queue;
		int ret;
		char reg_addr = 0;

		work_queue.nmsgs = 1;
		work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
		if (!work_queue.msgs) {
            printf("I2C1_ReadData: Memory alloc error\n");
			return -1;
		}

		// ioctl(fd, I2C_TIMEOUT, 2);
		ioctl(fd, I2C_RETRIES, 1);

		(work_queue.msgs[0]).len = len;
		(work_queue.msgs[0]).flags = I2C_M_RD;
		(work_queue.msgs[0]).addr = devI2CAddr;
		(work_queue.msgs[0]).buf = buf;

		ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
        //printf("I2C1_ReadData: ioctl (ret=%d, errno=%d)\n", ret, errno);
        free(work_queue.msgs);

        return ret;
}
int I2C1_IsErr(void)
{
	return 0;

}

int i2c_write_reg(int fd, const unsigned char *buf, unsigned slave_address, unsigned reg_address, int len)
{
	struct i2c_rdwr_ioctl_data work_queue;
	unsigned char w_val = reg_address;
	unsigned char w_buf[len+1];
	int ret;

	w_buf[0] = reg_address;

	work_queue.nmsgs = 1;
	work_queue.msgs = (struct i2c_msg*)malloc(work_queue.nmsgs *sizeof(struct i2c_msg));
	if (!work_queue.msgs) {
        printf("i2c_write_reg: Memory alloc error\n");
		return -1;
	}

	// ioctl(fd, I2C_TIMEOUT, 2);
	ioctl(fd, I2C_RETRIES, 1);

	(work_queue.msgs[0]).flags = 0;
	(work_queue.msgs[0]).len = 1 + len;
	(work_queue.msgs[0]).addr = slave_address;
	(work_queue.msgs[0]).buf = w_buf;

	memcpy(w_buf + 1, buf, len);

	ret = ioctl(fd, I2C_RDWR, (unsigned long) &work_queue);
	  if (ret < 0)
      printf("i2c_write_reg: ioctl (ret=%d, errno=%d)\n", ret, errno);
    free(work_queue.msgs);

    return ret;
}
