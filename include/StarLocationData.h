#ifndef STARLOCATIONDATA_H
#define STARLOCATIONDATA_H

#include <iostream>
#include <fstream>
#include <queue>
#include <atomic>

using namespace std;

const int BUFFER_SIZE = 1500;

typedef struct
{
    char rtcm_buffer[BUFFER_SIZE];
    int rtcm_length;
} VrsRtcmData;

typedef struct
{
    char timestamp[16]; // 卫星时间
    double latitude;    // GGA纬度
    char latNS;         // 纬度半球，N或S(北纬或南纬)
    double longitude;   // GGA经度
    char lonEW;         // 经度半球，E或W(东经或西经)
    int qual;           // GPS状态
    int satellites;     // 卫星数量
    double hdop;        // 水平精度因子
    double altitude;    // 海拔高度
    double undulation;  // 大地水准面高度异常差值
} gga_data;

typedef struct
{
    double yaw;         // 航向角
    double vec;         // 速度(km/h)
} vtg_data;

typedef enum StarLocSeviceLevel
{
    STAR_LIGHT = 0, // 亚米级定位服务
    STAR_PLUS,      // 亚米级+定位服务
    STAR_PRO        // 厘米级定位服务
}StarLocSeviceLevel_enum;

typedef struct StarUser
{
    char username[15];
	char password[15];
	StarLocSeviceLevel_enum level;

}StarUser_Info;

enum StarLocState
{
    NETWORK_UNAVAILABLE = 4001, // 网络不可用
    PARAM_MISSING       = 4002, // 缺少参数
    SERVICE_CONNECTING  = 4005, // 服务连接中
    SERVICE_CONNECTED   = 4006, // 服务已连接
    SERVICE_DISCONNECT  = 4007, // 服务断开连接
    USER_IDENTIFY       = 4008, // 用户验证成功
    USER_UNAUTHORIZED   = 4009, // 用户验证失败
    ILLEGAL_GGA         = 4012, // 非法GGA
    GGA_SEND_SUCCESS    = 4013, // GGA上传成功
    GGA_SEND_TIMEOUT    = 4014, // GGA上传超时
    INITIAL_GGA         = 4015, // 初始化GGA
    NON_INITIAL_GGA     = 4016, // 能够获取RTCM的GGA
};

/*
 * Gga数据校验接口
 * @param gga - string类型的gga数据
 * @return 是否校验通过
 */
bool checkGga(string);

/*
 * 906A定位模组配置接口
 * @param dev_name - string类型的串口名称
 * @return 是否配置成功
 */
bool config906A(string);

/*
 * 解析Gga
 * @param gga - string类型的gga数据
 * @return gga_data
 */
gga_data parseGga(string);

/*
 * 解析Vtg
 * @param vtg - string类型的vtg数据
 * @return vtg_data
 */
vtg_data parseVtg(string);

/*
 * imu timestamp control
 */
double getDoubleTimeStamp();
double get20msLaterTime(double, double);

class DataSave
{
public:

    // constructor
    DataSave(string);

    // destructor
    ~DataSave();

    // set data need to write
    void saveData(string);

private:

    string file_name;

    ofstream write_file;

    queue<string> data_queue;

    pthread_mutex_t queue_mux;

    atomic_bool thread_run;

    static void *inner_thread(void*);

    void runInnerThread();
};

#endif // STARLOCATIONDATA_H
