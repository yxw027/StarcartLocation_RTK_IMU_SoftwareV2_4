#ifndef STARLOCATIONFACTORY_H
#define STARLOCATIONFACTORY_H

/*
 * @file   StarLocationFactory.h
 * @brief  定位服务接口。
 * @author zhuang.sr
 * @date   2019.05.19
 */

#include <iostream>

using namespace std;

#include "StarLocationData.h"

class StarLocationService;

class StarLocationFactory
{
public:

    // constructor
    StarLocationFactory();

    // destructor
    ~StarLocationFactory();

    /*
     * 接口1: 定位服务的获取和初始化
     * 定位服务的获取和初始化，一个对象当前只能使用一种服务，若使用另一种服务，前一服务将关闭，无法使用。
     * @param uname   - your user name
     * @param pwd     - your password
     * @service_level - star location service level
     *                | STAR_LIGHT - 亚米级
     *                | STAR_PLUS  - 亚米级+
     *                | STAR_PRO   - 厘米级
     * @return true - 成功
     *      | false - 失败
     */
    bool getStarLocationService(string uname, string pwd, int service_level);

    /*
     * 接口2: 发送gga数据
     * 发送string类型的gga数据
     * @param gga - string类型的gga数据
     * @return 状态码
     */
    int sendGGA(string gga);

    // 接口3: 获取定位rtcm数据
    VrsRtcmData requestLocRtcm();

    // 接口4: 关闭服务
    void close();

    // 查看状态信息
    void onState();

private:

    StarLocationService* star_location;
};

#endif // STARLOCATIONFACTORY_H
