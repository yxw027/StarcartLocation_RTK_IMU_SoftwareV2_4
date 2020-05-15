/**
 * @file sta_i2c.h
 * @brief This file provides all the LSM6DS3 firmware definitions.
 *
 * Copyright (C) ST-Microelectronics SA 2015
 * @author: APG-MID team
 */

#ifndef _STA_LSM6DS3_H_
#define _STA_LSM6DS3_H_

#include "LSM6DS3_ACC_GYRO_driver.h"



#define I2C_LSM6DS3_ADDR		   0x6B


#define OSAL_ERROR      -1
#define OSAL_OK          0





#define FUNC_CFG_ACCESS                               (0X01)
#define SENSOR_SYNC_TIME_FRAME                        (0X04)
#define FIFO_CTRL1                                    (0X06)
#define FIFO_CTRL2                                    (0X07)
#define FIFO_CTRL3                                    (0X08)
#define FIFO_CTRL4                                    (0X09)
#define FIFO_CTRL5                                    (0X0A)
#define ORIENT_CFG_G                                  (0X0B)
#define INT1_CTRL                                     (0X0D)
#define INT2_CTRL                                     (0X0E)
#define WHO_AM_I                                      (0X0F)
#define CTRL1_XL                                      (0X10)
#define CTRL2_G                                       (0X11)
#define CTRL3_C                                       (0X12)
#define CTRL4_C                                       (0X13)
#define CTRL5_C                                       (0X14)
#define CTRL6_C                                       (0X15)
#define CTRL7_G                                       (0X16)
#define CTRL8_XL                                      (0X17)
#define CTRL9_XL                                      (0X18)
#define CTRL10_C                                      (0X19)


#define MASTER_CONFIG                                 (0X1A)
#define WAKE_UP_SRC                                   (0X1B)
#define TAP_SRC                                       (0X1C)
#define D6D_SRC                                       (0X1D)
#define STATUS_REG                                    (0X1E)
#define OUT_TEMP_L                                    (0X20)
#define OUT_TEMP_H                                    (0X21)
#define OUTX_L_G                                      (0X22)
#define OUTX_H_G                                      (0X23)
#define OUTY_L_G                                      (0X24)
#define OUTY_H_G                                      (0X25)
#define OUTZ_L_G                                      (0X26)
#define OUTZ_H_G                                      (0X27)
#define OUTX_L_XL                                     (0X28)
#define OUTX_H_XL                                     (0X29)
#define OUTY_L_XL                                     (0X2A)
#define OUTY_H_XL                                     (0X2B)
#define OUTZ_L_XL                                     (0X2C)
#define OUTZ_H_XL                                     (0X2D)



#define SENSORHUB1_REG                                (0X2E)
#define SENSORHUB2_REG                                (0X2F)
#define SENSORHUB3_REG                                (0X30)
#define SENSORHUB4_REG                                (0X31)
#define SENSORHUB5_REG                                (0X32)
#define SENSORHUB6_REG                                (0X33)
#define SENSORHUB7_REG                                (0X34)
#define SENSORHUB8_REG                                (0X35)
#define SENSORHUB9_REG                                (0X36)
#define SENSORHUB10_REG                               (0X37)
#define SENSORHUB11_REG                               (0X38)
#define SENSORHUB12_REG                               (0X39)
#define FIFO_STATUS1                                  (0X3A)
#define FIFO_STATUS2                                  (0X3B)
#define FIFO_STATUS3                                  (0X3C)
#define FIFO_STATUS4                                  (0X3D)
#define FIFO_DATA_OUT_L                               (0X3E)
#define FIFO_DATA_OUT_H                               (0X3F)
#define TIMESTAMP0_REG                                (0X40)
#define TIMESTAMP1_REG                                (0X41)
#define TIMESTAMP2_REG                                (0X42)

#define STEP_TIMESTAMP_L                              (0X49)
#define STEP_TIMESTAMP_H                              (0X4A)
#define STEP_COUNTER_L                                (0X4B)
#define STEP_COUNTER_H                                (0X4C)
#define SENSORHUB13_REG                               (0X4D)
#define SENSORHUB14_REG                               (0X4E)
#define SENSORHUB15_REG                               (0X4F)
#define SENSORHUB16_REG                               (0X50)
#define SENSORHUB17_REG                               (0X51)
#define SENSORHUB18_REG                               (0X52)
#define FUNC_SRC                                      (0X53)


#define TAP_CFG                                       (0x58)
#define TAP_THS_6D                                    (0x59)
#define INT_DUR2                                      (0x5A)
#define WAKE_UP_THS                                   (0x5B)
#define WAKE_UP_DUR                                   (0x5C)
#define FREE_FALL                                     (0x5D)
#define MD1_CFG                                       (0x5E)
#define MD2_CFG                                       (0x5F)
#define OUT_MAG_RAW_X_L                               (0x66)
#define OUT_MAG_RAW_X_H                               (0x67)
#define OUT_MAG_RAW_Y_L                               (0x68)
#define OUT_MAG_RAW_Y_H                               (0x69)
#define OUT_MAG_RAW_Z_L                               (0x6A)
#define OUT_MAG_RAW_Z_H                               (0x6B)


typedef struct
{
    float x; //mg
    float y; //mg
    float z; //mg
} Vectorf_t;

typedef struct LSM_mems_value_Tag
{
    Vectorf_t acce; //g
    Vectorf_t gyro; //dps
    float temperature;  
} LSM_mems_value_ST;


typedef struct LSM_mem_handler_Tag
{
  LSM_mems_value_ST  LSM_mems_value;
}LSM_mem_handler_ST;

typedef struct LSM_Mbox_msg_Tag
{
	unsigned char acce_x[2];
	unsigned char acce_y[2];
	unsigned char acce_z[2];
	unsigned char gyro_x[2];
	unsigned char gyro_y[2];
	unsigned char gyro_z[2];
	unsigned char temp[2];
	unsigned char lsmst;
}LSM_Mbox_msg_ST;

extern int GYRO_Module_Init(void);
extern void GYRO_Module_Deinit(void);
extern int GYRO_Module_GetInitStatus(unsigned char * state);
extern int GYRO_Device_GetAccelerate(Vectorf_t * acc_data);
extern int GYRO_Device_GetAngular(Vectorf_t * angular_data);
extern int GYRO_Device_GetTEMP(float* temp);





#endif /* _STA_I2C_H_ */
