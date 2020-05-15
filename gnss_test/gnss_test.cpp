
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

using namespace std;

#include "../include/gnss_module.h"

static GNSS_Module gnss;

static void gnss_show(gga_data data) {
  cout << "   timestamp == " << data.timestamp << "    latitude == " << data.latitude <<
       "   latNS == " << data.latNS << "    longitude == " << data.longitude << "   lonEW == " << data.lonEW << endl;
  cout << "    qual == " << data.qual << "         satellites == " << data.satellites <<
       "   hdop == " << data.hdop << " altitude == " << data.altitude << "   undulation == " << data.undulation << endl
       << endl;

}

static void gnss_vtg_show(vtg_data vtg) {
  cout << "  yaw: " << vtg.yaw;
  cout << "  vector: " << vtg.vec << "km/h" << endl;
}

int main() {
  bool ret;

  //  char username[15] = "15218778943";
//  char password[15] = "QWE123456";

  char username[15] = "TMC10419031120";
  char password[15] = "bec8b20736";

  cout << "test" << endl;

  gga_data gga;
  vtg_data vtg;

  bzero(&gga, sizeof(gga));
  bzero(&vtg, sizeof(vtg));

  strcpy(gnss.gnss_login.username, username);
  strcpy(gnss.gnss_login.password, password);
  gnss.gnss_login.level = STAR_PRO;

  gnss.GNSS_ModuleInit(&gnss.gnss_login);

  while (1) {

    ret = gnss.GNSS_GetGgaData(&gga);

//     gnss_show(gga);

    if (TRUE == ret) {
      gnss_show(gga);
    } else {
//                cout << "get gga data fail " << endl;
    }
    //gnss_vtg_show(vtg);

    ret = gnss.GNSS_GetVtgData(&vtg);
    if (TRUE == ret) {
      gnss_vtg_show(vtg);
    } else {
//               cout << "get vtg data fail " << endl;
    }

    usleep(20000);

  }

  return 0;
}




