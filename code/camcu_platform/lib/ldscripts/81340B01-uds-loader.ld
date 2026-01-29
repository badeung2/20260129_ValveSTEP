/*
 * Software Platform
 *
 * Copyright (C) 2018-2019 Melexis N.V.
 * Linker script file
 *
 * Project: MLX81340
 *          ROM :     20 Kbytes
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
ENTRY(__gdb_entry_point__)

MEMORY
{
    /* EEPROM memory map */
    eeprom_patch     (rw!x) : ORIGIN = 0x0400, LENGTH = 0x0040  /* EEPROM patch area */
    eeprom           (rw!x) : ORIGIN = 0x0440, LENGTH = 0x0170  /* EEPROM customer area */
    eeprom_cal       (r!x)  : ORIGIN = 0x05B0, LENGTH = 0x0050  /* EEPROM calibration data */
    eeprom_cs        (r!x)  : ORIGIN = 0x0600, LENGTH = 0x0040  /* EEPROM trim data */
    eeprom_reg       (rw!x) : ORIGIN = 0x0640, LENGTH = 0x0006  /* EEPROM registers */

    /* MLX4 RAM memory map */
    special          (rw)   : ORIGIN = 0x0A40, LENGTH = 0x0010  /* MLX16 <=> MLX4 shared RAM */
    mlx4_cst_tbl_ram (rw)   : ORIGIN = 0x0A80, LENGTH = 0x005E  /* MLX4 configuration tables */
    mlx4_par_tbl_ram (rw)   : ORIGIN = 0x0B00, LENGTH = 0x0088  /* MLX4 param tables */
    mlx4_patch_area  (rw)   : ORIGIN = 0x0B88, LENGTH = 0x0078  /* MLX4 patches area */

    /* MLX16 RAM memory map */
    ram              (rwx)  : ORIGIN = 0x0C00, LENGTH = 0x07EA  /* First 256 bytes are dp: */
    bl_ram           (rw)   : ORIGIN = 0x13EA, LENGTH = 0x000E  /* Bootloader's RAM shared with Flash application */
    bist_stat        (rw)   : ORIGIN = 0x13F8, LENGTH = 0x0008  /* Use the latest RAM area for the BIST-flags */

    /* MLX16 ROM memory map */
    rom              (rx)   : ORIGIN = 0x2C00, LENGTH = 0x4FFC  /* ROM */
    rom_crc          (rx)   : ORIGIN = 0x7BFC, LENGTH = 0x0004  /* CRC of the ROM */

    /* MLX16 flash memory map */
    flash_control    (rw)   : ORIGIN = 0x7E68, LENGTH = 0x0198  /* FLASH shell latches, and other configuration 'RAM'/ROM data */

    /* User flash regions */
    uds_reserved     (r)    : ORIGIN = 0x8000, LENGTH = 0x0004  /* UDS reserved field */
    uds_bist_len     (r)    : ORIGIN = 0x8004, LENGTH = 0x0002  /* UDS bist length */
    uds_key          (r)    : ORIGIN = 0x8006, LENGTH = 0x0002  /* UDS section fast key check */
    uds_app_check    (rx)   : ORIGIN = 0x8008, LENGTH = 0x0020  /* UDS application consistancy check */
    uds_config       (r)    : ORIGIN = 0x8028, LENGTH = 0x0008  /* UDS configuration */
    fw_btl_version   (r)    : ORIGIN = 0x8030, LENGTH = 0x0004  /* Bootloader version */
    fw_pltf_version  (r)    : ORIGIN = 0x8034, LENGTH = 0x0004  /* UDS Flash platform version */
    uds_services     (rx)   : ORIGIN = 0x8038, LENGTH = 0x0FC4  /* UDS configuration flash area */
    uds_crc          (r)    : ORIGIN = 0x8FFC, LENGTH = 0x0004  /* UDS CRC area */
}


/* direct, io and extra page addresses */
__dp__ = 0x0C00;
__io__ = 0x0000;
__ep__ = 0x0400;

/* far page addresses */
__fp0__ = 0x2C00;   /* interrupt tables */
__fp1__ = 0x2D00;   /* interrupt tables */
__fp2__ = 0x2E00;   /* interrupt tables */
__fp3__ = 0x2F00;
__fp4__ = 0x3000;
__fp5__ = 0x3100;
__fp6__ = 0x3200;
__fp7__ = 0x0400;   /* points to EEPROM */

/* entry point for gdb usage */
__gdb_entry_point__ = 0x2C00;

/* common sections */
SECTIONS
{
    .bist_stat (NOLOAD):
    {
        *(.bist_stat.bistHeader)
        *(.bist_stat.bistResetInfo)
        *(.bist_stat.bistError)
        *(.bist_stat.bistErrorInfo)
    } > bist_stat

    .flash_control (NOLOAD):
    {
        *(.flash_control)
    } > flash_control

    .uds_reserved :
    {
        SHORT(0x7FF);
        SHORT(0x7FF);
    } > uds_reserved

    .fw_pltf_version :
    {
        __fw_pltf_version_start = . ;
        KEEP(*(.fw_pltf_version))
    } > fw_pltf_version

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
