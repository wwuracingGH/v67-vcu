ENTRY(reset_handler)

MEMORY
{
    FLASH (rx):     ORIGIN = 0x08000000, LENGTH = 464K
    USR_FLASH(rw): ORIGIN = 0x0900C000, LENGTH = 48K
    SRAM (rwx):     ORIGIN = 0x20000000, LENGTH = 272K
}

SECTIONS
{
	.isr_vector :
	{
		KEEP(*(.isr_vector))
	} >FLASH

	.text :
	{
		. = ALIGN(4);
			
		*(.text)
		*(.rodata)
			
		. = ALIGN(4);
		_etext = .;
	} >FLASH

	_sidata = LOADADDR(.data);

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
			
		*(.data)

		. = ALIGN(4);
		_edata = .;
	} >SRAM AT> FLASH

	.bss :
	{
		. = ALIGN(4);
		_sbss = .;
			
		*(.bss)
			
		. = ALIGN(4);
		_ebss = .;
	} >SRAM

    .config : 
    {
       . = ALIGN(4);
       KEEP(*(.config))
    } > USR_FLASH
}
