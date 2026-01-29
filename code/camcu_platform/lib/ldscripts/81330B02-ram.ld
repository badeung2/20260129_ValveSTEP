/*
 * Software Platform
 *
 * Copyright (C) 2021 Melexis N.V.
 * Linker script file
 *
 * Project: MLX81330
 *          ROM :      8 Kbytes
 *          Flash :   32 Kbytes
 *          RAM :      2 KBytes
 *          EEPROM : 512 bytes
 */

OUTPUT_FORMAT("elf32-mlx16-fx")
OUTPUT_ARCH(mlx16fx)


/*
 * Default libraries which will be used for linking
 * Any additional libraries could be provided either through command
 * line option or in linker script
 */

/* Standard library */
GROUP(-lc -lgcc)

/* Force searching symbol in libraries if not defined */
EXTERN(__fw_vectors)
EXTERN(__fw_sys_services)
EXTERN(_PLTF_VERSION_Info)
EXTERN(_INDXtbl _PARAMtbl _AUTOADDtbl)

/* Define the entry point */
ENTRY(_main)

MEMORY
{
    /* EEPROM memory map */
    eeprom_patch     (rw!x) : ORIGIN = 0x0800, LENGTH = 0x0040  /* EEPROM patch area */
    eeprom           (rw!x) : ORIGIN = 0x0840, LENGTH = 0x0170  /* EEPROM customer area */
    eeprom_cal       (r!x)  : ORIGIN = 0x09B0, LENGTH = 0x0050  /* EEPROM calibration data */
    eeprom_cs        (r!x)  : ORIGIN = 0x0A00, LENGTH = 0x0040  /* EEPROM trim data */
    eeprom_reg       (rw!x) : ORIGIN = 0x0A40, LENGTH = 0x0006  /* EEPROM registers */

    /* MLX4 RAM memory map */
    special          (rw)   : ORIGIN = 0x0E40, LENGTH = 0x0010  /* MLX16 <=> MLX4 shared RAM */
    mlx4_cst_tbl_ram (rw)   : ORIGIN = 0x0E80, LENGTH = 0x005E  /* MLX4 configuration tables */
    mlx4_par_tbl_ram (rw)   : ORIGIN = 0x0F00, LENGTH = 0x0088  /* MLX4 param tables */
    mlx4_patch_area  (rw)   : ORIGIN = 0x0F88, LENGTH = 0x0078  /* MLX4 patches area */

    /* MLX16 RAM memory map */
    ram              (rwx)  : ORIGIN = 0x1000, LENGTH = 0x07EA  /* First 256 bytes are dp: */
    bl_ram           (rw)   : ORIGIN = 0x17EA, LENGTH = 0x000E  /* Bootloader's RAM shared with Flash application */
    bist_stat        (rw)   : ORIGIN = 0x17F8, LENGTH = 0x0008  /* Use the latest RAM area for the BIST-flags */

    /* MLX16 ROM memory map */
    rom              (rx)   : ORIGIN = 0x2000, LENGTH = 0x1FFC  /* ROM */
    rom_crc          (rx)   : ORIGIN = 0x3FFC, LENGTH = 0x0004  /* CRC of the ROM */

    /* MLX16 flash memory map */
    flash_control    (rw)   : ORIGIN = 0x5668, LENGTH = 0x0198  /* FLASH shell latches, and other configuration 'RAM'/ROM data */

    /* User flash regions */
    flash            (rx)   : ORIGIN = 0x5800, LENGTH = 0x8000  /* flash area */
}


/* direct, io and extra page addresses */
__dp__ = 0x1000;
__io__ = 0x0000;
__ep__ = 0x0800;

/* far page addresses */
__fp0__ = 0x2000;   /* interrupt tables */
__fp1__ = 0x2100;   /* interrupt tables */
__fp2__ = 0x2200;   /* interrupt tables */
__fp3__ = 0x2300;
__fp4__ = 0x2400;
__fp5__ = 0x2500;
__fp6__ = 0x2600;
__fp7__ = 0x0800;   /* points to EEPROM */


/* common sections */
SECTIONS
{
    .flash_control (NOLOAD):
    {
        *(.flash_control)
    } > flash_control

    /* Global variables in extra page with no values specified. */
    .ep :
    {
        . = ALIGN (2);
        __bss_ep_start = .;
        *(.ep)
        . = ALIGN (2);
        __bss_ep_end = . ;
    } > eeprom

    .ep.data :
    {
        . = ALIGN (2);
        __data_ep_start = . ;
        KEEP(*(.ep.data))
        . = ALIGN (2);
        __data_ep_end = . ;
    } > eeprom

    .eeprom_patch (NOLOAD):
    {
        *(.eeprom_patch)
    } > eeprom_patch

    .eeprom_cal (NOLOAD):
    {
        *(.eeprom_cal)
    } > eeprom_cal

    .eeprom_cs (NOLOAD):
    {
        *(.eeprom_cs)
    } > eeprom_cs

    .eeprom_reg (NOLOAD):
    {
        *(.eeprom_reg)
    } > eeprom_reg
}

/* fw app sections */
SECTIONS
{
    /* Global LIN variables used by LIN functions located in ROM.
     * These LIN variables have to be in fixed position because
     * the LIN functions are used by Flash application as well.
     * The beginning of RAM is used for these variables.
     * Drawback is dp area overlapping.
     * For more information: https://gitlab.melexis.com/swcc-si-projects/82100/issues/62
     */
    .lin_ram_flash (NOLOAD):
    {
        ASSERT(.==ORIGIN(ram), "The .lin_ram_flash shall be placed at the beginning of the RAM");
        . = ALIGN (2);
        __lin_ram_flash_start = .;
        . = . + __lin_ram_size;
        *(.lin_ram)
        ASSERT(__lin_ram_flash_start + __lin_ram_size == . , "The .lin_ram shall be empty for the Flash FW");
        . = ALIGN (2);
        __lin_ram_flash_end = .;
    } > ram
    __lin_ram_flash_size = __lin_ram_flash_end - __lin_ram_flash_start;

    .flash_prog :
    {
        *(.rodata)          /* mostly jumptables */
        *(.rodata.*)
        *(.rodata._mlx_*)   /* Flash arrays */

        . = ALIGN (2);
        *(.text)
        *(.text.*)

        . = ALIGN (2);
        _etext = .;

        /*
         * this should be last linking to flash
         * the rest of the flash will be used to store load data
         * (see "AT > flash" below)
         */
    } > ram

    /* Initialized variables (data) with .dp attribute
     * The initial values are copied by crt from Flash to RAM
     *
     * NOTES: Output section name shoud be '.dp.data' otherwise binutils
     * complain about "Dwarf Error"
     */
    .dp.data :
    {
        . = ALIGN (2);
        __ram_start = . ;
        __data_dp_start = . ;
        *(.dp.data)
        . = ALIGN (2);
        __data_dp_end = . ;
    } > ram
    __data_dp_size = __data_dp_end - __data_dp_start;

    /* Uninitialized variables (bss) with .dp attribute
     * This memory area is zeroed by crt
     *
     * NOTES: Output section name shoud be '.dp' otherwise binutils
     * complain about "Dwarf Error"
     */
    .dp :
    {
        . = ALIGN (2);
        __bss_dp_start = .;
        *(.dp)
        . = ALIGN (2);
        __bss_dp_end = . ;
    } > ram
    __bss_dp_size = __bss_dp_end - __bss_dp_start;

    /* Global data not cleared after reset  */
    .dpnoinit (NOLOAD) :
    {
        . = ALIGN (2);
        __dpnoinit_start = .;
        *(.dp.noinit)
        . = ALIGN (2);
        __dpnoinit_end = .;
    } > ram
    __dpnoinit_size = __dpnoinit_end - __dpnoinit_start;

    /* Global initialized variables with nodp attribute
     * The initial values are copied by crt from Flash
     */
    .data :
    {
        . = ALIGN (2);
        __data_start = . ;
        *(.ramfunc)
        *(.data)
        *(.data.*)
        . = ALIGN (2);
        __data_end = . ;
    } > ram
    __data_size = __data_end - __data_start ;

    /* Global variables with nodp attribute
     * This memory area is zeroed by crt
     */
    .bss :
    {
        . = ALIGN (2);
        __bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN (2);
        __bss_end = . ;
    } > ram
    __bss_size = __bss_end - __bss_start;

    /* Global data not cleared after reset  */
    .noinit (NOLOAD) :
    {
        . = ALIGN (2);
        __noinit_start = .;
        *(.noinit)
        . = ALIGN (2);
        __noinit_end = .;
    } > ram
    __noinit_size = __noinit_end - __noinit_start;

    /* The stack starts immediately after the global variables */
    PROVIDE (_fw_stack = .);

    /* This is the starting address of the heap.
     * It is located at the end of RAM and grows downward to the end of stack.
     * In a typical embedded application the heap is not used, but this
     * symbol has to be defined since some library functions need it.
     */
    PROVIDE (__ram_end = __ram_start + 0x800);  /* for 2K RAM */

    /* Shared area between Mlx4 and Mlx16
     * The beginning and the end of this area must be defined by
     * writing in the port SHRAML and SHRAMH
     */
    .special (NOLOAD):
    {
        *(.special)
    } > special

    /* Constant tables for the MLX4: specifies the LIN version
     * and timeouts based on the running frequencies
     */
    .mlx4_cst_tables_ram :
    {
        __mlx4_cst_tables_ram_start = . ;
        KEEP(*(.mlx4_cst_tables_ram))
        __mlx4_cst_tables_ram_end = . ;
    } > mlx4_cst_tbl_ram
    __mlx4_cst_tables_ram_size = __mlx4_cst_tables_ram_end - __mlx4_cst_tables_ram_start;

    /* MLX4 frames configuration tables: _INDXtb, _PARAMtbl, _AUTOADDtbl
     */
    .mlx4_par_tables_ram :
    {
        __mlx4_flash_table_start = . ;
        KEEP(*(.mlx4_flash_table))
        __mlx4_flash_table_end = . ;
    } > mlx4_par_tbl_ram
    __mlx4_flash_table_size = __mlx4_flash_table_end - __mlx4_flash_table_start;
    __mlx4_flash_table_load_start = LOADADDR(.mlx4_par_tables_ram);

    /* MLX4 patch instructions
     */
    .mlx4_patch_area :
    {
        __mlx4_patch_area_start = . ;
        KEEP(*(.mlx4_patch_area))
        __mlx4_patch_area_end = . ;
    } > mlx4_patch_area
}

