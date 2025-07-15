obj-m := gpio-driver-char.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

test: test_gpio
	./test_gpio

test_gpio: test_gpio.c
	gcc -o test_gpio test_gpio.c

install:
	sudo insmod simplegpio.ko
	sudo chmod 666 /dev/simplegpio

uninstall:
	sudo rmmod simplegpio

.PHONY: default clean test install uninstall