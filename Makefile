obj-m += hid-asusmt.o
KDIR=/home/gavin/linux-2.6.27.7
PWD=$(shell pwd)

KVERSION = $(shell uname -r)

all:
	#cp ../linux-source-2.6.21.4-eeepc/drivers/usb/input/asusmt.c ./
	make -C $(KDIR) M=$(PWD) clean
	make -C $(KDIR) M=$(PWD) modules
	#sudo rmmod hid-asusmt
	#sudo insmod hid-asusmt.ko

clean:
	make -C $(KDIR) M=$(PWD) clean

install:
	make -C $(KDIR) M=$(PWD) modules_install
