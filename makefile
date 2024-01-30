CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m0 -mthumb -nostdlib
CPPFLAGS= -Ivendor/CMSIS/Device/ST/STM32F0/Include \
	 -Ivendor/CMSIS/CMSIS/Core/Include
	
LINKER_FILE=linker_script.ld
LDFLAGS=-T $(LINKER_FILE)

BINARY = a.elf

PROGRAMMER = openocd
PROGRAMMER_FLAGS = -f interface/stlink.cfg -f target/stm32f0x.cfg

all: $(BINARY)

$(BINARY): main.o startup.o system_stm32f0xx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY)

main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) main.c -c

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup.c -c

system_stm32f4xx.o: vendor/CMSIS/Device/ST/STM32F0/Source/Templates/system_stm32f0xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) vendor/CMSIS/Device/ST/STM32F0/Source/Templates/system_stm32f0xx.c -c

.PHONY: clean
clean:
	rm -f *.o *.elf

flash: $(BINARY)
	$(PROGRAMMER) $(PROGRAMMER_FLAGS) -c "program $(BINARY) verify reset exit"