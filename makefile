CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m33 -mfpu=auto -mfloat-abi=hard -mthumb -nostdlib -W -Wall -ffunction-sections -g
CPPFLAGS= -DSTM32H533xx -Ivendor/CMSIS/Device/ST/STM32H5/Include \
	 -Ivendor/CMSIS/CMSIS/Core/Include \

LINKER_FILE=linker_script.ld 
LDFLAGS=-T $(LINKER_FILE)

BINARY = v67vcu.elf

all: $(BINARY) clean

$(BINARY): blinky_test.o startup.o system_stm32h5xx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY) -lgcc

blinky_test.o: blinky_test.c
	$(CC) $(CFLAGS) $(CPPFLAGS) blinky_test.c -c

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup.c -c

system_stm32h5xx.o: vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c -c


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
