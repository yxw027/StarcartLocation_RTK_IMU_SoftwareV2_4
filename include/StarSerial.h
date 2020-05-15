#include <iostream>
#include <termios.h>

using namespace std;

#include "StarLocationData.h"

typedef struct RtkData
{
    int gps_update;
    string gnvtg;
    string gngga;
    string ccdhv;
} Rtk;


class StarSerial
{
public:

    StarSerial(string);

    ~StarSerial();

    bool open906Device();

    int getFd();

    bool readRtkData();

    bool writeRtcmToSerial(VrsRtcmData);

    void setGpsUpdate(int);

    Rtk getCurrRtk();

private:

    int fd;
    string dev_name;
    const unsigned int dev_baud = B115200;

    pthread_mutex_t rtk_mut;
    Rtk curr_rtk;

    bool read_stop;
    static void *inner_thread(void*);
};
