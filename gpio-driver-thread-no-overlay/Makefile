obj-m=gpio-blink.o

KDIR:=/lib/modules/$(shell uname -r)/build
PWD:=$(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	dtc -@ -I dts -O dtb -o gpio-test-overlay.dtbo gpio-test-overlay.dts
