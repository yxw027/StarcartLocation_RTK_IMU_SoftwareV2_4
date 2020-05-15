/* Author :fanchao
 *  Date :2019-09-16
 *  Department:Fusion location Lab.
*/
#ifndef _GNSS_SINS_API_H_
#define _GNSS_SINS_API_H_

#include "KFApp.h"
#include "sinsapi.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>
using namespace std;

// bool GNSS_SINS_API(MPU_Data_value *mpu_Data_value, char gga[], char vtg[], char dhv[],
// 					float gga_delay, unsigned char gps_ok_flag, Out_Data *sins_out);

bool GNSS_SINS_API(MPU_Data_value *mpu_Data_value, string &gga, string &vtg, string &dhv,
					float gga_delay, int gps_ok_flag, string &sins_gga, string &sins_vtg);

vector<string> string_split(const string &terms_str,string spliting_tag);

void save_data(MPU_Data_value *mpu_Data_value, int gps_ok_flag, gga_t *gga, vtg_t *vtg, dhv_t *dhv, 
               float gga_delay, Out_Data *sins_out, vector<double> &vec_out);
// 校验码验证
string getCheck(const string msg);

//根据开机时间戳保存文件
string getLocalTime();
#endif

