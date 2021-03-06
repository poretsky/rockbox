#include "config.h"
#include "cpu.h"

OUTPUT_FORMAT("elf32-littlemips")
OUTPUT_ARCH(MIPS)
ENTRY(_start)
STARTUP(target/mips/ingenic_x1000/crt0.o)

/* End of the audio buffer, where the codec buffer starts */
#define ENDAUDIOADDR (X1000_DRAM_END - PLUGIN_BUFFER_SIZE - CODEC_SIZE)

/* Where the codec buffer ends, and the plugin buffer starts */
#define ENDCODECADDR  (ENDAUDIOADDR + CODEC_SIZE)

MEMORY
{
    IRAM : ORIGIN = X1000_IRAM_BASE, LENGTH = X1000_IRAM_SIZE
    DRAM : ORIGIN = X1000_DRAM_BASE, LENGTH = X1000_DRAM_SIZE
}

SECTIONS
{
    .text :
    {
        loadaddress = .;
        _loadaddress = .;
        *(.init.text);
        *(.text*);
    } > DRAM

    . = ALIGN(4);
    .rodata :
    {
        *(.rodata*);
    } > DRAM

    . = ALIGN(4);
    .data :
    {
        *(.data*);
        *(.sdata*);
    } > DRAM

    .iram X1000_IRAM_BASE: AT (_bssbegin)
    {
        _iramstart = .;
        . = 0x000; /* TLB refill */
        KEEP(*(.vectors.1));
        . = 0x100; /* Cache error */
        KEEP(*(.vectors.2));
        . = 0x180; /* General exception */
        KEEP(*(.vectors.3));
        . = 0x200; /* Interrupt */
        KEEP(*(.vectors.4));
        KEEP(*(.vectors));

        *(.icode*);
        *(.irodata);
        *(.idata);
        _iramend = .;
    } > IRAM
    _iramcopy = LOADADDR(.iram);

    . = ALIGN(4);
    .stack (NOLOAD) :
    {
         *(.stack);
         stackbegin = .;
         . += X1000_STACKSIZE;
         stackend = .;
         _irqstackbegin = .;
         . += X1000_IRQSTACKSIZE;
         _irqstackend = .;
    } > IRAM

    .bss (NOLOAD) :
    {
        _bssbegin = .;
        *(.sbss*);
        *(.bss*);
        *(COMMON);
        *(.scommon*);
        _bssend = .;
        _end = .;
    } > DRAM

    .audiobuf :
    {
        . = ALIGN(4);
        audiobuffer = .;
        loadbuffer = .;
    } > DRAM

    loadbufferend = ENDAUDIOADDR;
    audiobufend = ENDAUDIOADDR;
    codecbuf = ENDAUDIOADDR;
    pluginbuf = ENDCODECADDR;

    /DISCARD/ :
    {
        *(.MIPS.abiflags);
        *(.eh_frame);
        *(.rel.dyn);
    }
}
