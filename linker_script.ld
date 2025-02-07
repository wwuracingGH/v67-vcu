ENTRY(reset_handler)

MEMORY
{
    FLASH (rx):     ORIGIN = 0x08000000, LENGTH = 31K
    USR_FLASH(rwx): ORIGIN = 0x08007C00, LENGTH = 1K
    SRAM (rwx):     ORIGIN = 0x20000000, LENGTH = 6K
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

PROVIDE(__aeabi_fsub 	= qfp_fsub);
PROVIDE(__aeabi_fdiv 	= qfp_fdiv);
PROVIDE(__aeabi_fadd 	= qfp_fadd);
PROVIDE(__aeabi_fmul 	= qfp_fmul);
PROVIDE(__aeabi_f2iz 	= qfp_float2int);
PROVIDE(__aeabi_f2uiz 	= qfp_float2uint);
PROVIDE(__aeabi_i2f 	= qfp_int2float);
PROVIDE(__aeabi_ui2f	= qfp_uint2float);
PROVIDE(__aeabi_fcmpeq	= qfp_fcmpeq);
PROVIDE(__aeabi_fcmple	= qfp_fcmple);
PROVIDE(__aeabi_fcmplt	= qfp_fcmplt);
PROVIDE(__aeabi_fcmpge	= qfp_fcmpge);
PROVIDE(__aeabi_fcmpgt	= qfp_fcmpgt);

/* float __attribute(alias("qfp_fcos")) */
/* float __attribute(alias("qfp_fsin")) */
/* float __attribute(alias("qfp_ftan")) */
/* float __attribute(alias("qfp_fatan2")) */
/* float __attribute(alias("qfp_fexp")) */
/* float __attribute(alias("qfp_fln")) */
/* float __attribute(alias("qfp_fsqrt")) */
/* float __attribute(alias("qfp_fsqrt_fast")) */
/* float __attribute(alias("qfp_fdiv_fast")) */
