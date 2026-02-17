SECTIONS
{
  .psram_bss (NOLOAD) :
  {
    *(.psram_bss .psram_bss.*);
  } > PSRAM
}
