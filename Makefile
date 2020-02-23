obj-m += TLC5940.o

all:
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${KERNEL_SRC} M=$(PWD) modules

clean:
	make -C /home/nicolas/Raspberry/linux M=$(PWD) clean
