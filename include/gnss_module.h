#ifndef GNSS_RTCMMODULE_H
#define GNSS_RTCMMODULE_H

using namespace std;

#include "../include/StarLocationData.h"
#include "../include/sinsapi.h"
#include "../include/StarLocationFactory.h"
#include "../include/StarSerial.h"
#include "../include/gnss_sins_api.h"


#include <iostream>
#include <termios.h>
#include <string.h>

using namespace std;

class GNSS_Module
{
public:
    // constructor
    GNSS_Module(){};

    // destructor
    ~GNSS_Module(){};

	int GNSS_ModuleInit(StarUser_Info *data);

	int GNSS_ModuleDeInit();
	
	bool GNSS_GetInitsts();

	/* return:  TURE: get gga data successfully    FALSE: get gga data fail*/
	bool GNSS_GetGgaData(gga_data *GgaData);

	//get vtg data fpp200107
	bool GNSS_GetVtgData(vtg_data *VtgData);

	StarUser_Info gnss_login;
};


#endif

