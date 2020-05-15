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

using namespace std;

#include "../include/StarLocationFactory.h"
#include "../include/StarSerial.h"
#include "../include/gnss_sins_api.h"
#include "../include/sinsapi.h"
extern "C"
{
#include "../include/gyro_module.h"
}

int main() {
  // 906A serial
  string devName = "/dev/ttyAMA2";

  // 906A config
  if (config906A(devName)) {
    cout << "config 906A successful!" << endl;
  }

  // module init
  GYRO_Module_Init();


  // 1. 请填入您的用户名和密码
  string username = "TMC10419031120";
  string password = "bec8b20736";

  /*
   * 2. 请选择您要使用的服务级别：
   * STAR_LIGHT 亚米级定位服务
   * STAR_PLUS  亚米级+定位服务
   * STAR_PRO   厘米级定位服务
   * 下面是一个厘米级的定位服务初始化示例
   */
  int service_level = STAR_PRO;
  StarLocationFactory star_pro;
  bool get_pro_result = star_pro.getStarLocationService(username, password, service_level);
  if (!get_pro_result) {
    // 可以查看当前状态得到失败原因，其他时间也可以根据需要调用查看当前连接状态
    star_pro.onState();
    return -1;
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
  DataSave write_sins("/run/media/sda1/sinsResult.txt");
  DataSave write_loc("/run/media/sda1/906AResult.txt");
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


      // cout << "temperature: " << temperature << endl << endl;
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
        cout << "sins gga: " << gga_result << endl;
        cout << "sins vtg: " << vtg_result << endl;
        write_sins.saveData(gga_result);
        write_sins.saveData(vtg_result);
        gga_data analyze_gga = parseGga(gga_result);
        vtg_data analyze_vtg = parseVtg(vtg_result);
        cout << setprecision(16);
        cout << "location: timestamp: " << analyze_gga.timestamp;
        cout << "  qual: " << analyze_gga.qual;
        cout << "  satellites: " << analyze_gga.satellites << endl;
        cout << setprecision(10);
        cout << "          lat: " << analyze_gga.latitude << analyze_gga.latNS;
        cout << "  lon: " << analyze_gga.longitude << analyze_gga.lonEW << endl;
        cout << "          hdop: " << analyze_gga.hdop;
        cout << "  altitude: " << analyze_gga.altitude << "m";
        cout << "  undulation: " << analyze_gga.undulation << "m" << endl;
        cout << "          yaw: " << analyze_vtg.yaw;
        cout << "  vector: " << analyze_vtg.vec << "km/h" << endl;
      }

      if (curr_rtk.gps_update == 1) {
        int send_ret = star_pro.sendGGA(curr_rtk.gngga);
        cout << "curr gga: " << curr_rtk.gngga << endl;
        cout << "curr vtg: " << curr_rtk.gnvtg << endl;
        write_loc.saveData(curr_rtk.gngga);
        write_loc.saveData(curr_rtk.gnvtg);
        VrsRtcmData get_rtcm = star_pro.requestLocRtcm();
        if (get_rtcm.rtcm_length > 0) {
          bool write_ret = rtk_serial.writeRtcmToSerial(get_rtcm);
          // cout << (write_ret ? "write rtcm true" : "write rtcm false") << endl;
        }
      }
    }
  }

  return 0;
}
