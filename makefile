CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m33 -mfpu=auto -mfloat-abi=hard -mthumb -nostdlib -W -Wall -ffunction-sections -g
CPPFLAGS= -DSTM32H533xx -Ivendor/CMSIS/Device/ST/STM32H5/Include \
	 -Ivendor/CMSIS/Core/Include \

LINKER_FILE=linker_script.ld 
LDFLAGS=-T $(LINKER_FILE)

BINARY = v67vcu.elf
OBJS = 	startup.o \
		modules/logging.o \
		modules/control.o \
		vendor/printf/printf.o \
		vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.o \
		blinky_test.o

all: $(BINARY)

$(BINARY): blinky_test.o startup.o gpio.o can.o logging.o printf.o control.o system_stm32h5xx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY) -lgcc

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) startup.c -c

blinky_test.o: blinky_test.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) blinky_test.c -c

logging.o: modules/logging.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/logging.c -c

gpio.o: modules/gpio.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/gpio.c -c

can.o: modules/can.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/can.c -c

printf.o: vendor/printf/printf.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) vendor/printf/printf.c -c

control.o: modules/control.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/control.c -c

system_stm32h5xx.o: vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c -c

.PHONY: clean
clean:
	rm -f *.o

.PHONY: cleanall
cleanall:
	rm -f *.o *.elf

.PHONY: program
program:
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "program $(BINARY) verify reset exit"

.PHONY: debug
debug:
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg  

.PHONY: reset
reset:
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "reset"
