CFLAGS = -O2 -g
# CFLAGS = -O2

all: attys_fw.elf

base64.o: base64.c base64.h
	msp430-gcc -mmcu=msp430g2553 $(CFLAGS) -c base64.c

attys_fw.elf: attys_fw.c mpu9250.h base64.o
	msp430-gcc -mmcu=msp430g2553 $(CFLAGS) -o attys_fw.elf attys_fw.c base64.o

attys_fw.s: attys_fw.c mpu9250.h
	msp430-gcc -S -mmcu=msp430g2553 $(CFLAGS) attys_fw.c

obj-dump:
	objdump -s attys_fw.elf > attys_fw.dump

clean:
	rm -f *.elf *.hex *.o

prog: attys_fw.elf
	mspdebug rf2500 "prog attys_fw.elf"
