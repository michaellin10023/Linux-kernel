CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-


IOT_HOME = /opt/iot-devkit/1.7.2/sysroots


PATH := $(PATH):$(IOT_HOME)/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

SROOT=$(IOT_HOME)/i586-poky-linux

obj-m = rbt530_drv.o

all:
	make  ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SROOT)/usr/src/kernel M=$(shell pwd) modules
	i586-poky-linux-gcc -Wall -Wno-implicit -pthread -o  main.o main.c --sysroot=$(SROOT)	
	i586-poky-linux-gcc -Wall -Wno-implicit	 -pthread -o test main.c --sysroot=$(SROOT)
	
clean:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SROOT)/usr/src/kernel M=$(shell pwd) clean
	rm -f *.o
