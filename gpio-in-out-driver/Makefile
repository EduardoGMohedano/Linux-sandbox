obj-m := gpio-inout-driver.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	sudo insmod gpio-inout-driver.ko
	sudo chmod 666 /dev/gpio-inout

uninstall:
	sudo rmmod gpio-inout

overlay:
	dtc -I dts -O dtb -o custom-gpio-inout.dtbo custom-gpio-inout.dts 	

overlay-install:
	sudo cp custom-gpio-inout.dtbo /boot/firmware/overlays/ 

.PHONY: default clean install uninstall overlay overlay-install
