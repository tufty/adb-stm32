MEMORY
{
 ram (rwx) : ORIGIN = 0x20000c00, LENGTH = 17K
 rom (rx) : ORIGIN = 0x08002000, LENGTH = 120K
}
INCLUDE cortex-m-generic.ld
