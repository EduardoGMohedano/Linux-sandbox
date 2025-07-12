obj-m=gpio_test_driver.o

KDIR:=/lib/modules/$(shell uname -r)/build
PWD:=$(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	dtc -@ -I dts -0 dtb -o gpio-test-overlay.dtbo gpio-test-overlay.dts
