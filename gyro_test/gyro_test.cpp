//
// Created by renhono on 2020/4/20.
//

#include <iostream>

#include "unistd.h"
#include "iomanip"

extern "C"
{
#include "gyro_module.h"
}


using namespace std;
int main(int argc, char *argv[]) {

  Vectorf_t acc_data;
  Vectorf_t ang_data;
  float temperature;



  int ret = GYRO_Module_Init();

  cout << ret << endl;

  while (1) {

    GYRO_Device_GetAccelerate(&acc_data);
    GYRO_Device_GetAngular(&ang_data);

    GYRO_Device_GetTEMP(&temperature);

    cout.setf(ios::showpoint);
//    cout.setf(ios::fixed);
    cout << "[GYRO TEST] "
         << setprecision(6) << " || "
         << acc_data.x << " || "
         << acc_data.y << " || "
         << acc_data.z << " || "
         << ang_data.x << " || "
         << ang_data.y << " || "
         << ang_data.z << " || "
         << temperature << " || "
         << '\r';

    fflush(stdout);
//    sleep(1);
    usleep(500000);
  }

  GYRO_Module_DeInit();
}

