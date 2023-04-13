#include <stdint.h>

#ifndef _SDCARD_H_
#define _SDCARD_H_

void sdcard_startup();
int sdcard_init();
uint32_t sdcard_block_read(uint32_t start, uint32_t blkcnt, void *dst);
void sdcard_print_mmcinfo();

#endif