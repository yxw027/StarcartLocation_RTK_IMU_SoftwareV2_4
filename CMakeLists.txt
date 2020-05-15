cmake_minimum_required(VERSION 3.15)
project(StarcartLocation_RTK_IMU_SoftwareV2_4)

#set(CMAKE_CXX_STANDARD 14)

set(VERSION 1)

string(TIMESTAMP vDateTime ".%m.%d.%H%M")
message(${VERSION})
message(${vDateTime})

string(CONCAT VER ${VERSION} ${vDateTime})
message(${VER})

SET(BIN_NAME rtkImu)

#set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_SYSTEM_NAME Linux)

set(MYSYSROOT /opt/poky-st/2.4.2/sysroots)
set(DIR_POKY ${MYSYSROOT}/cortexa7hf-neon-poky-linux-gnueabi)

SET(CMAKE_C_COMPILER ${MYSYSROOT}/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER ${MYSYSROOT}/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++)

set(AFLAGS "-march=armv7ve -marm -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a7")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${AFLAGS} --sysroot=${DIR_POKY}" CACHE INTERNAL "" FORCE)
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} ${AFLAGS} --sysroot=${DIR_POKY}" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AFLAGS} --sysroot=${DIR_POKY}" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} ${AFLAGS} --sysroot=${DIR_POKY}" CACHE INTERNAL "" FORCE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/output)

include_directories(${DIR_POKY}/usr/include)
link_directories(${DIR_POKY}/usr/lib/)
link_directories(${DIR_POKY}/lib/)
link_directories(${MYSYSROOT}/x86_64-pokysdk-linux/include)
link_directories(${MYSYSROOT}/x86_64-pokysdk-linux/lib/)
link_directories(${MYSYSROOT}/x86_64-pokysdk-linux/usr/lib/)


#include_directories(.)
include_directories(./include)

#link_directories(.)
link_directories(./lib)
link_directories(./gyro_test/lsm)

add_definitions(-DVER=\"${VER}\")

add_compile_options(-fPIC)

#set(LIB_FILE src/adc.cpp src/gpio.c src/usart.c)

set(LIB_FILE gnss_module/gnss_module.cpp)

set(SRC_FILE_RTKIMU src/rtkImu.cpp)

#add_executable(adctest main.cpp)
add_executable(rtkImu ${SRC_FILE_RTKIMU})
#
target_link_libraries(rtkImu
        gyro_module
        sins_gps
        starlocation-ntrip
        )

#add_library(tiot_mod_static ${LIB_FILE})
add_library(gnss_module SHARED ${LIB_FILE})

target_link_libraries(gnss_module
        gyro_module
        sins_gps
        starlocation-ntrip
        pthread
        )

add_executable(gnss_test gnss_test/gnss_test.cpp)
target_link_libraries(gnss_test
        gnss_module
        )

add_executable(gnss_test_k gnss_test/gnss_test_k.cpp)
target_link_libraries(gnss_test_k
        gnss_module
        )

add_executable(auto_test auto_test/auto_test.cpp)

target_link_libraries(auto_test
        gyro_module
        sins_gps
        starlocation-ntrip
        )


add_executable(gyro_test
        gyro_test/gyro_test.cpp
        gyro_test/lsm/i2c-util.c
        gyro_test/lsm/LSM6DS3_ACC_GYRO_driver.c
        gyro_test/lsm/sta_lsm6ds3.c

        )
#target_link_libraries(gyro_test
#        gyro_module
#        )



#set_target_properties(tiot_mod_static PROPERTIES OUTPUT_NAME "tiot_mod")
#
#
#include_directories(include)
#
#add_executable(StarcartLocation_RTK_IMU_SoftwareV2_4
#        include/gnss_sins_api.h
#        include/gyro_module.h
#        include/KFApp.h
#        include/PSINS.h
#        include/sinsapi.h
#        include/StarLocationData.h
#        include/StarLocationFactory.h
#        include/StarSerial.h
#        src/rtkImu.cpp)
