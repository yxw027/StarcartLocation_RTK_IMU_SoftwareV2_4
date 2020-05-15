# makefile for sample
# author: zhuang.sr
# date: 2019.09.12

CXX = arm-poky-linux-gnueabi-g++  -march=armv7ve -marm -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a7 --sysroot=/opt/poky-st/2.4.2/sysroots/cortexa7hf-neon-poky-linux-gnueabi

# define dependence library
LDFLAGS = -L ../lib
LIBS = -lstarlocation-ntrip -lgyro_module -lsins_gps

# define objects and target name
OBJS = rtkImu.cpp
TAR = rtkImu

# complie command
$(TAR) : $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $(TAR) -Wl,-rpath,../lib

# clean command
clean:
	rm -f $(TAR)
