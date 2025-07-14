obj-m=gpio_driver_fs.o

KDIR:=/lib/modules/$(shell uname -r)/build
PWD:=$(shell pwd)

# base warnings
CFLAGS  := -Wall -Wextra
CXXFLAGS:= -Wall -Wextra

# append and export
CFLAGS  += \
  -Wno-incompatible-pointer-types \
  -Wno-error=incompatible-pointer-types \
  -Wno-incompatible-pointer-types-discards-qualifiers
export CFLAGS

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	dtc -@ -I dts -0 dtb -o gpio-test-overlay.dtbo gpio-test-overlay.dts
