/* RAM region to be used for DMA3 link structures.*/
REGION_ALIAS("DMA3_RAM", ram0);

SECTIONS
{
    /* Special section for GPDMA links, it must not exceed 64kB in size.*/
    .dma3 (NOLOAD) : ALIGN(16)
    {
        __dma3_base__ = .;
        *(.dma3)
        *(.dma3.*)
        *(.bss.__dma3_*)
        . = ALIGN(4);
        __dma3_end__ = .;
    } > DMA3_RAM
}
