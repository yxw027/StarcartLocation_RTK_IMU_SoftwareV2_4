/**
 * @file sta_i2c.c
 * @brief This file provides all the LSM6DS3 firmware functions.
 *
 * Copyright (C) ST-Microelectronics SA 2015
 * @author: APG-MID team
 */
#include "sta_lsm6ds3.h"
#include "LSM6DS3_ACC_GYRO_driver.h"
#include "i2c-util.h"
#include "typedef.h"
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>

#define I2C_DEV       "/dev/i2c-0"
#define SENSITIVITY_2G        (0.061)  /* mG/LSB */
#define SENSITIVITY_125DPS     (4.375) /* mdps/LSB */

LSM_Mbox_msg_ST LSM_Mbox_msg;
LSM_mem_handler_ST LSM_mem_handler_s;
int i2cfd;
uint8_t lsm_state = 0;

static int gyro_InitMems(void);
static int gyro_lsm6ds3_InitReg(void);
static int gyro_sample_temperature(float *pTemp);
static int gyro_sample_acc_gyro(Vectorf_t *pAcceValue, Vectorf_t *pGyroValue);
static int gyro_lsm6ds3_ReadReg(uint8_t addr, uint8_t *pBuf, int len);
static int gyro_lsm6ds3_WriteReg(uint8_t addr, uint8_t data);

/*************************************************************************
 *函数名称:
 *函数功能:LSM6DS3寄存器 功能任务
 *输入参数: 操作的I2C句柄
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
int GYRO_Module_Init(void) {

  int ret = -1;
  int i = 0;
  for (i = 0; i < 3; i++) {

    i2cfd = open(I2C_DEV, O_RDWR);
    if (i2cfd > 0) {

      printf("open i2c_0 ok!\r\n");
      break;
    }
  }
  if (i <= 3) {

    ret = gyro_lsm6ds3_InitReg();
    gyro_InitMems();
    lsm_state = 1;
  } else {
    return ret;
  }
  return ret;
}

/***************************************************
 * Function: GYRO_Module_Deinit
 * Description:get gyro module status
 * Parameters: none
 * Returns: bool
 * $Create & Verlog:$
 * Author:XIAOYU.SHI  Date:2019-09-06   Version:V1.0
 ****************************************************/
void GYRO_Module_Deinit(void) {
  close(i2cfd);
  lsm_state = 0;
}

/***************************************************
 * Function: GYRO_Module_GetInitStatus
 * Description:get gyro module status
 * Parameters: none
 * Returns: bool
 * $Create & Verlog:$
 * Author:XIAOYU.SHI  Date:2019-09-06   Version:V1.0
 ****************************************************/

int GYRO_Module_GetInitStatus(unsigned char *state) {
  int ret = -1;
  if ((void *) 0 == state) {
    ret = -1;
  } else {
    *state = lsm_state;
    ret = 0;
  }
  return ret;
}

/*************************************************************************
 *函数名称:dev_lsm6ds3_InitReg
 *函数功能:初始化LSM6DS3寄存器
 *输入参数:svc_i2c_com_handler 操作的I2C句柄
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_lsm6ds3_InitReg(void) {

  /**
   * FIXME: fix assert
   * 20200514 fpp
   */

  uint8_t ui8Data;
  int ret = 0;


  ret = gyro_lsm6ds3_WriteReg(FUNC_CFG_ACCESS, 0x00);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error FUNC_CFG_ACCESS\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_ReadReg(FUNC_CFG_ACCESS, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error FUNC_CFG_ACCESS\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_WriteReg(CTRL6_C, 0x00);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL6_C\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_ReadReg(CTRL6_C, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL6_C\r\n");
    goto _ERROR;
  }

  //开启加速度，转换频率为104HZ，量程为16g，50HZ滤波器
  ret = gyro_lsm6ds3_WriteReg(CTRL1_XL, 0x67);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL1_XL\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_ReadReg(CTRL1_XL, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL1_XL\r\n");
    goto _ERROR;
  }


  ret = gyro_lsm6ds3_WriteReg(INT1_CTRL, 0X40);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error INT1_CTRL\r\n");
    goto _ERROR;
  }


  ret = gyro_lsm6ds3_ReadReg(INT1_CTRL, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error INT1_CTRL\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_WriteReg(CTRL10_C, 0X38);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL10_C\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_ReadReg(CTRL10_C, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL10_C\r\n");
    goto _ERROR;
  }

  //开启陀螺仪，转换频率为104HZ，量程为2000dps
  ret = gyro_lsm6ds3_WriteReg(CTRL2_G, 0x4C);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL2_G\r\n");
    goto _ERROR;
  }

  ret = gyro_lsm6ds3_ReadReg(CTRL2_G, &ui8Data, 1);
  if (ret < 0) {
    printf(" gyro_lsm6ds3_InitReg error CTRL2_G\r\n");
    goto _ERROR;
  }


//  printf("==================\n");
//  ret = gyro_lsm6ds3_ReadReg(FUNC_CFG_ACCESS, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",FUNC_CFG_ACCESS,ui8Data);
//  ret = gyro_lsm6ds3_ReadReg(CTRL1_XL, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",CTRL1_XL,ui8Data);
//  ret = gyro_lsm6ds3_ReadReg(INT1_CTRL, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",INT1_CTRL,ui8Data);
//  ret = gyro_lsm6ds3_ReadReg(CTRL10_C, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",CTRL10_C,ui8Data);
//  ret = gyro_lsm6ds3_ReadReg(CTRL6_C, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",CTRL6_C,ui8Data);
//  ret = gyro_lsm6ds3_ReadReg(CTRL2_G, &ui8Data, 1);
//  printf("addr:0x%2x value:0x%2x \n",CTRL2_G,ui8Data);

  goto _OK;
  _OK:
  return 0;

  _ERROR:
  return -1;

  /**
   *
   * 200pcs origin code
   */
  /****
  uint8_t ui8Data;
  int ret = 0;
  ret = gyro_lsm6ds3_WriteReg(FUNC_CFG_ACCESS, 0x00);
  if (ret != 0) {
    goto _ERROR;

  }
  ret = gyro_lsm6ds3_ReadReg(FUNC_CFG_ACCESS, &ui8Data, 1);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_WriteReg(CTRL6_C, 0x00);
  if (ret != 0)
    goto _ERROR;


  //开启加速度，转换频率为104HZ，量程为16g，50HZ滤波器
  ret = gyro_lsm6ds3_WriteReg(CTRL1_XL, 0x67);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_ReadReg(CTRL1_XL, &ui8Data, 1);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_WriteReg(INT1_CTRL, 0X40);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_ReadReg(INT1_CTRL, &ui8Data, 1);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_WriteReg(CTRL10_C, 0X38);
  if (ret != 0)
    goto _ERROR;

  ret = gyro_lsm6ds3_ReadReg(CTRL10_C, &ui8Data, 1);
  if (ret != 0)
    goto _ERROR;
  //开启陀螺仪，转换频率为104HZ，量程为2000dps
  ret = gyro_lsm6ds3_WriteReg(CTRL2_G, 0x4C);
  if (ret != 0)
    goto _ERROR;
  return 0;
  _ERROR:
  return -1;

  ****/

}

/*************************************************************************
 *函数名称:lsm_InitMems
 *函数功能:LSM6DS3 mems chushihua
 *输入参数:void
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_InitMems(void) {
  uint8_t lsm_id = 0x0;
  uint16_t timout_count = 0;
  while (timout_count++ < 32) {
    LSM6DS3_ACC_GYRO_WriteReg(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);
    LSM6DS3_ACC_GYRO_R_WHO_AM_I((uint8_t *) &lsm_id);
    if (lsm_id == LSM6DS3_ACC_GYRO_WHO_AM_I)  //manage here comunication error
    {
      break;
    }
    printf("[LSM_mems] WHO_AM_I=%x\r\n", lsm_id);
  }
  printf("[LSM_mems] WHO_AM_I=%x\r\n", lsm_id);
  if (lsm_id != LSM6DS3_ACC_GYRO_WHO_AM_I)    //manage here comunication error
  {
    printf("[LSM_mems] data=%x\r\n", lsm_id);
    return -1;
  }
  /* Soft Reset the LSM6DS3 device */
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_SW_RESET(LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE))
    return -1;

  //初始化陀螺仪
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_ODR_G(LSM6DS3_ACC_GYRO_ODR_XL_104Hz))
    return -1;

  //if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_FS_G(LSM6DS3_ACC_GYRO_FS_G_2000dps))
  //return -1;
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_FS_125(LSM6DS3_ACC_GYRO_FS_125_ENABLED))
    return -1;
  //初始化加速度计
  /* Set ACC ODR  */
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_ODR_XL(LSM6DS3_ACC_GYRO_ODR_XL_104Hz))
    return -1;

  /* Set ACC full scale */
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_FS_XL(LSM6DS3_ACC_GYRO_FS_XL_2g))
    return -1;

  /* BDU Enable */
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_W_BDU(LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE))
    return -1;
  return 0;

}

/*************************************************************************
 *函数名称:lsm_sample_acc_gyro
 *函数功能:LSm get acc and gyro data
 *输入参数:pAcceValue,pGyroValue
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_sample_acc_gyro(Vectorf_t *pAcceValue, Vectorf_t *pGyroValue) {
  Type3Axis16bit_U value;
  uint8_t value_XL = 0;
  uint8_t value_G = 0;
#if 1
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_R_XLDA(&value_XL))
    return -1;

  if (LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL == value_XL) {
    LSM6DS3_ACC_GYRO_Get_GetAccData(value.u8bit);

    /* Transorm LSB into mG */
    pAcceValue->x = (float) value.i16bit[0] * SENSITIVITY_2G;
    pAcceValue->y = (float) value.i16bit[1] * SENSITIVITY_2G;
    pAcceValue->z = (float) value.i16bit[2] * SENSITIVITY_2G;
  }
#endif
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_R_GDA(&value_G))
    return -1;

  memset(&value, 0, sizeof(value));

  if (LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL == value_G) {
    LSM6DS3_ACC_GYRO_Get_GetGyroData(value.u8bit);
    /* Transorm LSB into mdps */
    pGyroValue->x = value.i16bit[0] * SENSITIVITY_125DPS / 1000;
    pGyroValue->y = value.i16bit[1] * SENSITIVITY_125DPS / 1000;
    pGyroValue->z = value.i16bit[2] * SENSITIVITY_125DPS / 1000;
  }

  return 0;
}

/*************************************************************************
 *函数名称:lsm_sample_temperature
 *函数功能:LSm get temperature data
 *输入参数:pAcceValue,pGyroValue
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_sample_temperature(float *pTemp) {
  Type1Axis16bit_U value;
  uint8_t ui8Status = 0;

  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_ReadReg(LSM6DS3_ACC_GYRO_STATUS_REG, &ui8Status))
    return -1;

  if (0 == (ui8Status & 0x04))
    return -1;

  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_Get_GetTemp(value.u8bit))
    return -1;

  *pTemp = 25 + (float) value.i16bit / 16;

  return 0;
}

/*************************************************************************
 *函数名称:dev_lsm6ds3_ReadReg
 *函数功能:LSM6DS3读寄存器
 *输入参数:svc_i2c_com_handler 操作的I2C句柄
addr 寄存器地址
pBuf 读取数据存放的地址
 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_lsm6ds3_ReadReg(uint8_t addr, uint8_t *pBuf, int len) {
  int err = 0;
  err = i2c_read_reg(i2cfd, pBuf, I2C_LSM6DS3_ADDR, addr, len);
  return err;
}

/*************************************************************************
 *函数名称:dev_lsm6ds3_WriteReg
 *函数功能:LSM6DS3写寄存器
 *输入参数:svc_i2c_com_handler 操作的I2C句柄
addr 寄存器地址
data 待写入数据

 *返回参数:操作结果
 *函数说明:
 **************************************************************************/
static int gyro_lsm6ds3_WriteReg(uint8_t addr, uint8_t data) {
  int err;
  err = i2c_write_reg(i2cfd, &data, I2C_LSM6DS3_ADDR, addr, 0x01); //gpOS_TIMEOUT_IMMEDIATE gpOS_TIMEOUT_INFINITY
  return err;
}
#if 0
/*************************************************************************
 *函数名称:gyro_Lsm6ds3_demo
 *函数功能:get data demo
 *输入参数: 
 *返回参数:
 *函数说明:
 **************************************************************************/
void gyro_Lsm6ds3_demo(void)
{
  while(1)
   {

           // gyro_sample_acc_gyro(&LSM_mem_handler_s.LSM_mems_value.acce, &LSM_mem_handler_s.LSM_mems_value.gyro);
         // gyro_sample_temperature(&LSM_mem_handler_s.LSM_mems_value.temperature);
      GYRO_Device_GetAccelerate(&LSM_mem_handler_s.LSM_mems_value.acce);
      GYRO_Device_GetAngular(&LSM_mem_handler_s.LSM_mems_value.gyro);
      GYRO_Device_GetTEMP(&LSM_mem_handler_s.LSM_mems_value.temperature);

#if 1
      printf("[lsm_mems] |acceX=%.5f|acceY=%.5f|acceZ=%.5f|gyroX=%.5f|gyroY=%.5f|gyroZ=%.5f|temperature=%.5f\r\n",
                      (LSM_mem_handler_s.LSM_mems_value.acce.x),
                      (LSM_mem_handler_s.LSM_mems_value.acce.y),
                      (LSM_mem_handler_s.LSM_mems_value.acce.z),
                      (LSM_mem_handler_s.LSM_mems_value.gyro.x),
                      (LSM_mem_handler_s.LSM_mems_value.gyro.y),
                      (LSM_mem_handler_s.LSM_mems_value.gyro.z),
                      (LSM_mem_handler_s.LSM_mems_value.temperature));
#endif
    usleep(1000*10);
   }
}

/*************************************************************************
 *函数名称:demo fun
 *函数功能:主函数
 *输入参数:
 *返回参数:
 *函数说明:
 **************************************************************************/
void main(void)
{
   GYRO_Module_Init();
   gyro_Lsm6ds3_demo();

}
#endif
/***************************************************
 * Function: GYRO_Device_GetAccelerate
 * Description: gyro device get acc
 * Parameters: none
 * Returns: bool
 * $Create & Verlog:$
 * Author:XIAOYU.SHI  Date:2019-09-06   Version:V1.0
 ****************************************************/
int GYRO_Device_GetAccelerate(Vectorf_t *acc_data) {
  Type3Axis16bit_U value;
  uint8_t value_XL = 0;
  memset(&value, 0, sizeof(value));
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_R_XLDA(&value_XL))
    return -1;

  if (LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL == value_XL) {
    LSM6DS3_ACC_GYRO_Get_GetAccData(value.u8bit);
    /* Transorm LSB into mG */
    acc_data->x = (float) value.i16bit[0] * SENSITIVITY_2G;
    acc_data->y = (float) value.i16bit[1] * SENSITIVITY_2G;
    acc_data->z = (float) value.i16bit[2] * SENSITIVITY_2G;

  } else {
    return -1;
  }
  return 0;
}

/***************************************************
 * Function: GYRO_Device_GetAngular
 * Description: gyro device get angular
 * Parameters: none
 * Returns: bool
 * $Create & Verlog:$
 * Author:XIAOYU.SHI  Date:2019-09-06   Version:V1.0
 ****************************************************/

int GYRO_Device_GetAngular(Vectorf_t *angular_data) {
  Type3Axis16bit_U value;
  uint8_t value_G = 0;
  memset(&value, 0, sizeof(value));
  if (MEMS_ERROR == LSM6DS3_ACC_GYRO_R_GDA(&value_G))
    return -1;
  if (LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL == value_G) {

    LSM6DS3_ACC_GYRO_Get_GetGyroData(value.u8bit);
    /* Transorm LSB into mdps */
    angular_data->x = value.i16bit[0] * SENSITIVITY_125DPS / 1000;
    angular_data->y = value.i16bit[1] * SENSITIVITY_125DPS / 1000;
    angular_data->z = value.i16bit[2] * SENSITIVITY_125DPS / 1000;
  } else {
    return -1;
  }
  return 0;
}

/***************************************************
 * Function: GYRO_Device_GetTEMP
 * Description: gyro device get temp
 * Parameters: none
 * Returns: bool
 * $Create & Verlog:$
 * Author:XIAOYU.SHI  Date:2019-09-06   Version:V1.0
 ****************************************************/
int GYRO_Device_GetTEMP(float *temp) {
  int ret = 0;
  if (temp == (void *) 0) {
    return -1;
  }

  if (-1 == gyro_sample_temperature(temp)) {
    return -1;
  }
  return 0;
}

