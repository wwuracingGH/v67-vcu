CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m0 -mthumb -nostdlib -W -Wall -ffunction-sections 
CPPFLAGS= -DSTM32F042x6 -Ivendor/CMSIS/Device/ST/STM32F0/Include \
	 -Ivendor/CMSIS/CMSIS/Core/Include \
	 -Ivendor/qfplib
	
LINKER_FILE=linker_script.ld
LDFLAGS=-T $(LINKER_FILE)

BINARY = v66vcu.elf

all: $(BINARY) clean

$(BINARY): main.o startup.o system_stm32f0xx.o vendor/qfplib/qfplib.s 
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY)

main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) main.c -c

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup.c -c

system_stm32f0xx.o: vendor/CMSIS/Device/ST/STM32F0/Source/Templates/system_stm32f0xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) vendor/CMSIS/Device/ST/STM32F0/Source/Templates/system_stm32f0xx.c -c


.PHONY: clean
clean:
	rm -f *.o

.PHONY: cleanall
cleanall:
	rm -f *.o *.elf