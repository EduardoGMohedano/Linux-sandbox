obj-m := gpio-driver-char.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	sudo insmod gpio-driver-char.ko
	sudo chmod 666 /dev/simplegpio

uninstall:
	sudo rmmod gpio-driver-char

.PHONY: default clean install uninstall
