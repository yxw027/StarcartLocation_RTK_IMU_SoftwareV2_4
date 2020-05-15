#ifndef _GYRO_MODULE_H_
#define _GYRO_MODULE_H_

#define SENSITIVITY_16G    (0.488288) /* mG/LSB */
#define SENSITIVITY_2000DPS    (70.0) /* mdps/LSB */
#define DATAMAXLEN              13


#define SENSITIVITY_2G        (0.061)
#define SENSITIVITY_125DPS     (4.375)


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
} LSM_mems_value_ST;


typedef struct LSM_Mbox_msg_Tag
{   unsigned char reserve;
	unsigned char acce_x[2];
	unsigned char acce_y[2];
	unsigned char acce_z[2];
	unsigned char gyro_x[2];
	unsigned char gyro_y[2];
	unsigned char gyro_z[2];
	unsigned char temp[2];
    unsigned char lsmst;
}LSM_Mbox_msg_ST;


#if 0
typedef struct LSM_mem_handler_Tag
{

  LSM_mems_value_ST  LSM_mems_value;
  
}LSM_mem_handler_ST;

#endif


extern int GYRO_Module_GetInitStatus(unsigned char * state);
extern void GYRO_Module_DeInit(void);
extern int GYRO_Module_Init(void);
extern int GYRO_Device_GetAccelerate(Vectorf_t * acc_data);
extern int GYRO_Device_GetAngular(Vectorf_t * angular_data);
extern int GYRO_Device_GetTEMP(float* temp);



#endif 


