#include "config.h"
#include "cpu.h"

OUTPUT_FORMAT("elf32-littlemips")
OUTPUT_ARCH(MIPS)
ENTRY(_start)
STARTUP(target/mips/ingenic_x1000/crt0.o)
INPUT(target/mips/exception-mips.o)
INPUT(target/mips/system-mips.o)

#ifdef BOOTLOADER
# undef PLUGIN_BUFFER_SIZE
# undef CODEC_SIZE
# define PLUGIN_BUFFER_SIZE 0
# define CODEC_SIZE 0
# if defined(HAVE_INIT_ATTR)
/* This needs to be fixed for the bootloader */
#  error "bootloader does not support INIT_ATTR"
# endif
#endif

/* End of the audio buffer, where the codec buffer starts */
#define ENDAUDIOADDR (X1000_DRAM_END - PLUGIN_BUFFER_SIZE - CODEC_SIZE)

/* Where the codec buffer ends, and the plugin buffer starts */
#define ENDCODECADDR  (ENDAUDIOADDR + CODEC_SIZE)

/* Place init code in the codec buffer */
#define INIT_BASE   ENDAUDIOADDR
#define INIT_SIZE   CODEC_SIZE

MEMORY
{
    IRAM : ORIGIN = X1000_IRAM_BASE, LENGTH = X1000_IRAM_SIZE
    DRAM : ORIGIN = X1000_DRAM_BASE, LENGTH = X1000_DRAM_SIZE
    TCSM : ORIGIN = X1000_TCSM_BASE, LENGTH = X1000_TCSM_SIZE
    INIT : ORIGIN = INIT_BASE, LENGTH = INIT_SIZE
}

SECTIONS
{
    .startup :
    {
        loadaddress = .;
        _loadaddress = .;
        *(.startup.text);
    } > DRAM

    .text :
    {
        *(.text*);
#ifndef HAVE_INIT_ATTR
        *(.init*);
#endif
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

    /*
     * The following sections are loaded after normal DRAM sections
     * but are copied elsewhere by the startup code.
     */
    _noloaddram = .;

    .iram :
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
    } > IRAM AT> DRAM
    _iramcopy = LOADADDR(.iram);

    .tcsm :
    {
        _tcsmstart = .;
        KEEP(*(.tcsm*));
        _tcsmend = .;
    } > TCSM AT> DRAM
    _tcsmcopy = LOADADDR(.tcsm);

#ifdef HAVE_INIT_ATTR
    .init :
    {
        _initstart = .;
        *(.init*);
        _initend = .;
    } > INIT AT> DRAM
    _initcopy = LOADADDR(.init);
#endif

    /* Sections below have no data. */

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

    .bss _noloaddram (NOLOAD) :
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
    } > DRAM

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

#if defined(HAVE_NOCROSSREFS_TO) && defined(HAVE_INIT_ATTR)
/* Prevent accidental references to the .init section from other sections */
NOCROSSREFS_TO(.init .text .iram .tcsm);
#endif
