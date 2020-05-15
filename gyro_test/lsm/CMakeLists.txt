cmake_minimum_required(VERSION 3.15)
project(lsm C)

set(CMAKE_C_STANDARD 11)

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

include_directories(.)

add_compile_options(-fPIC)

add_library(gyro_module SHARED
        i2c-util.c
        LSM6DS3_ACC_GYRO_driver.c
        sta_lsm6ds3.c
        )
