/*
 * @file   rtkImu.cpp
 * @brief  启迪项目rtk融合imu工程示例
 * @author <zhuang.sr@starcart.cn>
 * @date   2019.09.11
 */

#include <iostream>
#include <iomanip>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

using namespace std;

#include "../include/gnss_module.h"

extern "C"
{
#include "../include/gyro_module.h"
}

static bool GNSS_InitSts = FALSE;
static string result = {0};

static string gl_gga_result = {0};
static string gl_vtg_result = {0};

static Rtk gl_curr_rtk = {0};

//20200515 fpp
pthread_mutex_t sins_mutex;
pthread_mutex_t curr_mutex;

static bool alg_flag = 0;
static bool gnss_flag = FALSE;

/***************************************************
* Function: gnss_DataServe
* Description: process data pthread
* Parameters:NULL
* Returns:NULL
* $Create & Verlog:$
* Author:lihuan  Date:2019-9-7 	 Version:V1.0
****************************************************/

static void *gnss_DataServe(void *arg) {
  pthread_detach(pthread_self());

  StarUser_Info *login = (StarUser_Info *) arg;
  string username = login->username;
  string password = login->password;

  // 906A serial
  string devName = "/dev/ttyAMA2";

  // 906A config
  if (config906A(devName)) {
    cout << "config successful!" << endl;
  }

  // module init
  GYRO_Module_Init();

  /*
   * 2. 请选择您要使用的服务级别：
   * STAR_LIGHT 亚米级定位服务
   * STAR_PLUS  亚米级+定位服务
   * STAR_PRO   厘米级定位服务
   * 下面是一个厘米级的定位服务初始化示例
   */
  int service_level = login->level;
  StarLocationFactory star_pro;
  bool get_pro_result = star_pro.getStarLocationService(username, password, service_level);
  if (!get_pro_result) {
    // 可以查看当前状态得到失败原因，其他时间也可以根据需要调用查看当前连接状态
    star_pro.onState();
    return NULL;
  }
  StarSerial rtk_serial(devName);
  bool serial_correct = (rtk_serial.getFd() > 0);
  MPU_Data_value imu_data;
  Rtk curr_rtk;
  string gga_result;
  string vtg_result;

  Vectorf_t acc_data;
  Vectorf_t ang_data;
  float temperature;
  double startTime;
  double nextReadTime = getDoubleTimeStamp();
  while (serial_correct) {
    startTime = getDoubleTimeStamp();
    if (startTime < nextReadTime) {
      usleep(1000);
    } else {
      imu_data.time = nextReadTime;
      // control frequency in 50Hz
      nextReadTime = get20msLaterTime(startTime, nextReadTime);
      GYRO_Device_GetAccelerate(&acc_data);
      GYRO_Device_GetAngular(&ang_data);
      GYRO_Device_GetTEMP(&temperature);
      imu_data.Accel[0] = acc_data.x;
      imu_data.Accel[1] = acc_data.y;
      imu_data.Accel[2] = acc_data.z;
      imu_data.Gyro[0] = ang_data.x;
      imu_data.Gyro[1] = ang_data.y;
      imu_data.Gyro[2] = ang_data.z;
      imu_data.Temp = temperature;
      curr_rtk = rtk_serial.getCurrRtk();

      bool alg_ret = GNSS_SINS_API(&imu_data,
                                   curr_rtk.gngga,
                                   curr_rtk.gnvtg,
                                   curr_rtk.ccdhv,
                                   0.06,
                                   curr_rtk.gps_update,
                                   gga_result,
                                   vtg_result);

      if (alg_ret) {

//                cout << "sins gga: " << gga_result << endl;
//                cout << "sins vtg: " << vtg_result << endl;
//                gga_data analyze_gga = parseGga(gga_result);
//                vtg_data analyze_vtg = parseVtg(vtg_result);
//                cout << setprecision(16);
//                cout << "location: timestamp: " << analyze_gga.timestamp;
//                cout << "  qual: " << analyze_gga.qual;
//                cout << "  satellites: " << analyze_gga.satellites << endl;
//                cout << setprecision(10);
//                cout << "          lat: " << analyze_gga.latitude << analyze_gga.latNS;
//                cout << "  lon: " << analyze_gga.longitude << analyze_gga.lonEW << endl;
//                cout << "          hdop: " << analyze_gga.hdop;
//                cout << "  altitude: " << analyze_gga.altitude << "m";
//                cout << "  undulation: " << analyze_gga.undulation << "m" << endl;
//                cout << "          yaw: " << analyze_vtg.yaw;
//                cout << "  vector: " << analyze_vtg.vec << "km/h" << endl;

        pthread_mutex_lock(&sins_mutex);

        gl_gga_result = gga_result;
        gl_gga_result = vtg_result;

        pthread_mutex_unlock(&sins_mutex);
      }

      // cout << "end: " << fixed << getDoubleTimeStamp() << endl;
      if (0 != alg_ret) {
        //cout << "rtkUnionImu result: " << result << endl;

        pthread_mutex_lock(&curr_mutex);
        alg_flag = TRUE;

      } else {

        alg_flag = FALSE;
        // cout << "rtkUnionImu fail" << endl;
      }

      if (curr_rtk.gps_update == 1) {
        int send_ret = star_pro.sendGGA(curr_rtk.gngga);
        gnss_flag = TRUE;

        //cout << "send_ret = " << send_ret << endl;
//        cout << "curr gpgga: " << curr_rtk.gngga << endl;
//        cout << "curr gnvtg: " << curr_rtk.gnvtg << endl;
        // cout << curr_rtk.gnvtg << curr_rtk.gngga << curr_rtk.ccdhv << endl;
        VrsRtcmData get_rtcm = star_pro.requestLocRtcm();
        if (get_rtcm.rtcm_length > 0) {
          bool write_ret = rtk_serial.writeRtcmToSerial(get_rtcm);
//          cout << (write_ret ? "write rtcm true" : "write rtcm false") << endl;
          cout << (write_ret ? " " : "write rtcm false") << endl;
        }

        // cout << "end send: " << fixed << getDoubleTimeStamp() << endl;
      } else {
        gnss_flag = FALSE;
      }
    }
  }

  return 0;
}

/***************************************************
* Function: GNSS_ModuleInit
* Description: initialize GNSS
* Parameters:
		username: user name 
		password: user password
		Level: STAR_LIGHT ,STAR_PLUS ,STAR_PRO
* Returns:
 	 	   0:  successfully
		   -1； fail 
* $Create & Verlog:$
* Author:lihuan  Date:2019-9-7 	 Version:V1.0
****************************************************/
int GNSS_Module::GNSS_ModuleInit(StarUser_Info *data) {
  pthread_t gnss_id;

  pthread_mutex_init(&sins_mutex, NULL);   //0515
  pthread_mutex_init(&curr_mutex, NULL);   //0515

  if (pthread_create(&gnss_id, NULL, gnss_DataServe, (void *) data)) {
    cout << "creat pthread fail" << endl;
    return -1;
  }

  GNSS_InitSts = TRUE;

  return 0;

}

/***************************************************
* Function: GNSS_GetInitsts
* Description: get GNSS initialize status 
* Parameters:NULL
* Returns:
 	 	   0:  FALSE
		   1； TRUE 
* $Create & Verlog:$
* Author:lihuan  Date:2019-9-7 	 Version:V1.0
****************************************************/
bool GNSS_Module::GNSS_GetInitsts() {
  return GNSS_InitSts;
}

/***************************************************
* Function: GNSS_RtcmDeInit
* Description: disable initialize GNSS
* Parameters:NULL
* Returns:
 	 	   0:  successfully
		   -1； fail 
* $Create & Verlog:$
* Author:lihuan  Date:2019-9-7 	 Version:V1.0
****************************************************/
int GNSS_Module::GNSS_ModuleDeInit() {
  int ret = -1;

  if (GNSS_InitSts) {
    GNSS_InitSts = FALSE;

    pthread_mutex_destroy(&sins_mutex);
    pthread_mutex_destroy(&curr_mutex);

    ret = 0;
  } else {
    ret = -1;
  }
  return ret;
}

/***************************************************
* Function: GNSS_GetGgaData
* Description: get GGA data
* Parameters:GgaData:gga data
* Returns:
		   FALSE:  fail
		   TRUE ；  successfully
* $Create & Verlog:$
* Author:lihuan  Date:2019-9-7 	 Version:V1.0
****************************************************/
bool GNSS_Module::GNSS_GetGgaData(gga_data *GgaData) {
  bool ret = FALSE;
  gga_data gga;

  if (GNSS_InitSts) {
    if (TRUE == alg_flag) {
//		    cout << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << endl;
      *GgaData = parseGga(gga_result);
//            cout << "location: timestamp====================================== : " << GgaData->timestamp << endl;

      ret = TRUE;
    } else {
      if (gnss_flag) {
        gga = parseGga(curr_rtk.gngga);
        if (!gga.qual) {
          cout << "no star" << endl;
        } else {
          *GgaData = gga;
          ret = TRUE;
        }
      } else {
//				cout << "data not update"<< endl;
      }
    }
  } else {
    cout << "it not initialize" << endl;
    ret = FALSE;
  }
  return ret;
}

/**
 * @author fpp
 * @date 20200107
 * @param VtgData
 * @return
 */
bool GNSS_Module::GNSS_GetVtgData(vtg_data *VtgData) {
  bool ret = FALSE;
  vtg_data vtg;

  if (GNSS_InitSts) {
    if (TRUE == alg_flag) {
//            cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << endl;
      *VtgData = parseVtg(vtg_result);

//            cout << "location: timestamp +++++++++++++++++++++++++++++++++++++: " << VtgData->vec << endl;

      ret = TRUE;
    } else {
      if (gnss_flag) {
        vtg = parseVtg(curr_rtk.gnvtg);

//                    *GgaData = gga;
        *VtgData = vtg;
        ret = TRUE;

      } else {
//                cout << "data not update"<< endl;
      }
    }
  } else {
    cout << "it not initialize" << endl;
    ret = FALSE;
  }
  return ret;
}













