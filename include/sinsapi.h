/* sinsapi c++ hearder file sinsapi.h */
/*
	By     : shilei ,Starcar Lt. Guangzhou China
	Date   : 2019-08-01
*/
#ifndef _SINSAPI_H_
#define _SINSAPI_H_
#ifdef __cplusplus
extern "C" {
#endif


typedef struct{
	double time; //系统时间,开机为0，单位是s，保留三位小数。
	double Accel[3];//Accel X,Y,Z
	double Temp;//温度 待加入
	double Gyro[3];//Gyro X,Y,Z
	double Mag[3];	//Mag X,Y,Z	
}MPU_Data_value;
typedef struct
{
	bool gps_flag;
	int hour;
	int min;
	double sec;
	int lon_d;//dd
	double lon_m;//mm.mmmm
	char NS;
	int lat_d;//dd
	double lat_m;//mm.mmmm
	char EW;
	int fix;
	int num_satellite;
	double h_DOP;//0.5-99.9
	double altitude;
	double age;
}gga_t;

typedef struct
{
	double cogt;
	double cogm;
	double knots;
	double kph;
}vtg_t;

//"$CCDHV,102741.000,2.631,2.468,-0.856,0.310,2.612,,,,,M*27"
typedef struct
{
	double time;//UTC hhmmss.sss 
	double vel_norm;//  m/s 
	double vel_e, vel_n, vel_u;// m/s  $$´Ë´¦Èý¸öÁ¿ ²ÎÓëÔËËã$$ 
	double vel_ground;//¶ÔµØËÙ¶È m/s
}dhv_t;

typedef struct{
 double timestamp;
 double Frame_head;
 double OUT_cnt;
 double Gyro[3];
 double Accel[3];
 double Magn[3];//magnetic
 double mBar;//barometer
 double Att[3];
 double Vn[3];
 double Pos[5];
 double GPS_Vn[3];
 double GPS_Pos[5];
 double GPS_status;
 double GPS_delay;
 double Temp;//temprature

}Out_Data;

double GetAveStdRMS(const double *a, int n, int opt);

Out_Data* SINS_API(MPU_Data_value *mpu_Data_value, gga_t *gga, vtg_t *vtg, dhv_t *hdv, double gga_delay, int gps_ok_flag);


#ifdef __cplusplus
}
#endif

#endif
