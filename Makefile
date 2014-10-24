obj-m += hid-asusmt.o
KDIR=/lib/modules/`uname -r`/build
PWD=$(shell pwd)

KVERSION = $(shell uname -r)

all:
	make -C $(KDIR) M=$(PWD) clean
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

install:
	make -C $(KDIR) M=$(PWD) modules_install
