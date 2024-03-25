SECTION_DATA_PROLOGUE (.app_spiram, (NOLOAD),)
{
 	*libapp.a:(.bss .bss.*)
} GROUP_LINK_IN(ext_ram_seg)
