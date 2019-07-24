
EXTRA_CFLAGS += -Wall

TARGET = GPIO_DEVICE

obj-m += $(TARGET).o


$(TARGET)-objs := DFI_GPIO_INT.o

all:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
		echo $(shell uname -r) > version.log
clean:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
		rm version.log
