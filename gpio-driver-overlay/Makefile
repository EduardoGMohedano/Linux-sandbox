obj-m := gpio-driver-overlay.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	sudo insmod gpio-driver-overlay.ko
	sudo chmod 666 /dev/simplegpio

uninstall:
	sudo rmmod gpio-driver-overlay

overlay:
	dtc -I dts -O dtb -o custom-gpio17.dtbo custom-gpio17.dts 	

overlay-install:
	sudo cp custom-gpio17.dtbo /boot/firmware/overlays/ 

.PHONY: default clean install uninstall overlay overlay-install
