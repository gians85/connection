/*******************************************************************************
* Memory Definitions
*******************************************************************************/
/*
BlueNRG-1 memory map
+-----------------------+ 0x20005FFF
|  RAM (24K)            |
+-----------------------+ 0x20000000
|                       |
|                       |
+-----------------------+ 0x10067FFF
|                       |
|  FLASH (160K)         |
+-----------------------+ 0x10040000
|                       |
+-----------------------| 0x100007FF
|   ROM (2K)            |
+-----------------------+ 0x10000000
*/


_MEMORY_RAM_BEGIN_   =  0x20000000; 
_MEMORY_RAM_SIZE_    =  0x6000;           /* 24KB  */
_MEMORY_RAM_END_     =  0x20005FFF; 

_MEMORY_FLASH_BEGIN_ = 0x10040000;  
_MEMORY_FLASH_SIZE_  = 0x28000;          /* 160KB */ 
_MEMORY_FLASH_END_   = 0x10067FFF;  

_MEMORY_ROM_BEGIN_   = 0x10000000;    
_MEMORY_ROM_SIZE_    = 0x800;             /* 2KB */ 
_MEMORY_ROM_END_     = 0x100007FF;  


/* Reserved for BTLE stack non volatile memory */
FLASH_NVM_DATASIZE   = (4*1024);


  /* This configuration is intended for application not supporting OTA firmware upgrade */
  /*
  BlueNRG-1 standard application memory map
  +-----------------------+ 0x20005FFF
  |  RAM (24K)            |
  +-----------------------+ 0x20000000
  |                       |
  |                       |
  +-----------------------+ 0x10068000
  |                       |
  |  NVM(4K)              |
  +-----------------------+ 0x10067000
  |                       |
  |  User app (156K)      |
  +-----------------------+ 0x10040000
  |                       |
  +-----------------------| 0x100007FF
  |   ROM (2K)            |
  +-----------------------+ 0x10000000
*/

_MEMORY_FLASH_OFFSET_ = DEFINED(MEMORY_FLASH_APP_OFFSET) ? (MEMORY_FLASH_APP_OFFSET) : (0) ;
MEMORY_FLASH_APP_SIZE = ( _MEMORY_FLASH_SIZE_ - FLASH_NVM_DATASIZE - _MEMORY_FLASH_OFFSET_);
RESET_SM_MANAGER_SIZE = 0;
RESET_MANAGER_SIZE = 0;



/* 
   *****************************
   * ST_OTA_HIGHER_APPLICATION *
   *****************************
*/
   /* This configuration is intended for application supporting OTA firmware upgrade with 2-app scheme (app in the upper part of memory map) */
  /*
     BlueNRG-1 higher OTA firmware upgrade support higher application memory map
     +-----------------------+ 0x20005FFF
     |  RAM (24K-4)          |
     +-----------------------+ 0x20000004
     |                       |
     |                       |
     +-----------------------+ 0x10068000
     |                       |
     |  NVM(4K)              |
     +-----------------------+ 0x10066800
     |                       |
     |  Higher app (76K)     |
     +-----------------------+ 0x10053800
     |                       |
     |  Lower app (76K)      |
     +-----------------------| 0x10040800
     |   Reset Manager (2K)  |
     +-----------------------+ 0x10040000
     |                       |
     +-----------------------| 0x100007FF
     |   ROM (2K)            |
     +-----------------------+ 0x10000000
  */


RESET_MANAGER_SIZE = DEFINED(ST_OTA_HIGHER_APPLICATION) ? 0x800 : RESET_MANAGER_SIZE ;
MEMORY_FLASH_APP_SIZE = DEFINED(ST_OTA_HIGHER_APPLICATION) ? ((_MEMORY_FLASH_SIZE_ - RESET_MANAGER_SIZE - FLASH_NVM_DATASIZE - 2048) / 2) : MEMORY_FLASH_APP_SIZE ;
_MEMORY_FLASH_OFFSET_ = DEFINED(ST_OTA_HIGHER_APPLICATION) ? (RESET_MANAGER_SIZE + MEMORY_FLASH_APP_SIZE) : _MEMORY_FLASH_OFFSET_ ;


/* 
   *****************************
   * ST_OTA_LOWER_APPLICATION *
   *****************************
*/
  /* This configuration is intended for application supporting OTA firmware upgrade with 2-app scheme (app in the lower part of memory map) */
  /*
     BlueNRG-1 higher OTA firmware upgrade support lower application memory map
     +-----------------------+ 0x20005FFF
     |  RAM (24K-4)          |
     +-----------------------+ 0x20000004
     |                       |
     |                       |
     +-----------------------+ 0x10068000
     |                       |
     |  NVM(4K)              |
     +-----------------------+ 0x10066800
     |                       |
     |  Higher app (76K)     |
     +-----------------------+ 0x10053800
     |                       |
     |  Lower app (76K)      |
     +-----------------------| 0x10040800
     |   Reset Manager (2K)  |
     +-----------------------+ 0x10040000
     |                       |
     +-----------------------| 0x100007FF
     |   ROM (2K)            |
     +-----------------------+ 0x10000000
  */

RESET_MANAGER_SIZE = DEFINED(ST_OTA_LOWER_APPLICATION) ? 0x800 : RESET_MANAGER_SIZE ;
MEMORY_FLASH_APP_SIZE = DEFINED(ST_OTA_LOWER_APPLICATION) ? ((_MEMORY_FLASH_SIZE_ - RESET_MANAGER_SIZE - FLASH_NVM_DATASIZE - 2048) / 2) : MEMORY_FLASH_APP_SIZE ;
_MEMORY_FLASH_OFFSET_ = DEFINED(ST_OTA_LOWER_APPLICATION) ? (RESET_MANAGER_SIZE) : _MEMORY_FLASH_OFFSET_ ;


/* 
   ******************************************
   * ST_USE_OTA_SERVICE_MANAGER_APPLICATION *
   ******************************************
*/
  /* This configuration is intended for application supporting OTA firmware upgrade with independent OTA firmware upgrade service manager
  (app in the upper part of memory map) */
  /*
     BlueNRG-1 higher OTA firmware upgrade support higher application memory map
     +-----------------------+ 0x20005FFF
     |  RAM (24K-4)          |
     +-----------------------+ 0x20000004
     |                       |
     |                       |
     +-----------------------+ 0x10068000
     |                       |
     |  NVM(4K)              |
     +-----------------------+ 0x10067000
     |                       |
     |  User app (92K)       |
     +-----------------------+ 0x10050000
     | Reset Manager (2K)    |
     | + OTA S. Manager (62K)|
     +-----------------------+ 0x10040000
     |                       |
     +-----------------------| 0x100007FF
     |   ROM (2K)            |
     +-----------------------+ 0x10000000
  */

RESET_SM_MANAGER_SIZE = DEFINED(ST_USE_OTA_SERVICE_MANAGER_APPLICATION) ? 0xF000 : RESET_SM_MANAGER_SIZE ; /* Including OTA firmware upgrade service manager application */
MEMORY_FLASH_APP_SIZE = DEFINED(ST_USE_OTA_SERVICE_MANAGER_APPLICATION) ? (_MEMORY_FLASH_SIZE_ - RESET_SM_MANAGER_SIZE - FLASH_NVM_DATASIZE) : MEMORY_FLASH_APP_SIZE ;
_MEMORY_FLASH_OFFSET_ = DEFINED(ST_USE_OTA_SERVICE_MANAGER_APPLICATION) ? (RESET_SM_MANAGER_SIZE) : _MEMORY_FLASH_OFFSET_ ;
/******************************************************************************/


/* Linker script to configure memory regions. */
MEMORY
{ 
  FLASH (rx)      : ORIGIN = 0x10040000, LENGTH = 160k
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 24k
  ROM (rx)        : ORIGIN = 0x10000000, LENGTH = 0x800
}



/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 * 
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   _estack
 */

ENTRY(Reset_Handler)

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x0;      /* required amount of heap  */
_Min_Stack_Size = 0xC00; /* required amount of stack */

SECTIONS
{

	/* The startup code goes first into FLASH */
	.bootloader (ORIGIN(FLASH)) :
  	{
    	. = ALIGN(4);    
	   /* . = _MEMORY_FLASH_OFFSET_ ; */
	    KEEP(*(.bootloader)) /* Startup code */
	    . = ALIGN(4);
  	}>FLASH
  	
  	
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        KEEP(*(.eh_frame*))
        . = ALIGN(4);
     	_etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;


    //__etext = .;
    //_sidata = .;
    /* used by the startup to initialize data */
     /* used by the startup to initialize data */
    _sidata = LOADADDR(.data);
  	_sidata2 = LOADADDR(.ram_preamble);
    
    .data : //AT (__etext)
    {
        __data_start__ = .;
        _sdata = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);


        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
        _edata = .;

    } > RAM AT > FLASH


  	
 
  	/* RAM preamble */
  	.ram_preamble /*(NOLOAD)*/ :
  	{
	    . = ALIGN(4);
	    _sdata2 = .;        /* create a global symbol at data start */
	    /* This is used by the startup in order to initialize the .bss section */
	    . = 0x04 ; /* There is a waste of RAM here */
	    KEEP(*(.ota_sw_activation))
	    . = 0x08 ; /* There is a waste of RAM here */
	    KEEP(*(.savedMSP))
	    . = 0x0C ; /* There is a waste of RAM here */
	    KEEP(*(.wakeupFromSleepFlag))
	    . = 0x10 ; /* There is a waste of RAM here */
	    //KEEP(*(.app_base))
	    . = 0x30 ; /* There is a waste of RAM here */
	    KEEP(*(.__blueflag_RAM))
	      _edata2 = .;        /* create a global symbol at data start */
	    . = 0x34 ; /* There is a waste of RAM here */
	    KEEP(*(.__crash_RAM))
	    . = 0xc0 ; /* There is a waste of RAM here */
  	}  >RAM AT> FLASH
    
  
    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        _sbss = .;
        KEEP(*(.bss.__blue_RAM))
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        _ebss = .;
    } > RAM  
    
/************************************************************************************
* The last 2KB sector of FLASH is reserved for firmware to use. The BLE Host
* stores its security database in this area. The linker needs to make sure this
* 2KB sector is left empty.
*/
  	BLOCK_STACKLIB_FLASH_DATA (_MEMORY_FLASH_END_ - FLASH_NVM_DATASIZE + 1) (NOLOAD) :
  	{
	    . = ALIGN(2048);
	    
	    KEEP(*(.noinit.stacklib_flash_data))
	    KEEP(*(.noinit.stacklib_stored_device_id_data))
    
  	} >FLASH


	/* This is to emulate place at end of IAR linker */
	CSTACK (ORIGIN(RAM) + LENGTH(RAM) - _Min_Stack_Size) (NOLOAD) :
  	{
	    . = ALIGN(4);
	    _estack = . + _Min_Stack_Size;         /* define a global symbol at bss end */
	    . = ALIGN(4);
  	} > RAM
  
	.rom_info (NOLOAD) :
	{
	    . = ALIGN(4);
	    KEEP(*(.rom_info)) 
	    . = ALIGN(4);
	} >FLASH
	  

    .heap (COPY):
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > RAM

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    _estack = __StackTop;
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
    

  	.ARM.attributes 0 : { *(.ARM.attributes) }
}
