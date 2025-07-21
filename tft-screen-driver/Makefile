obj-m := tft-screen-driver.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	sudo insmod tft-screen-driver.ko
	sudo chmod 666 /dev/simple-tft

uninstall:
	sudo rmmod tft-screen-driver

overlay:
	dtc -@ -I dts -O dtb -o tft-custom.dtbo tft-custom.dts 	

overlay-install:
	sudo cp -f tft-custom.dtbo /boot/overlays 

.PHONY: default clean install uninstall overlay overlay-install
