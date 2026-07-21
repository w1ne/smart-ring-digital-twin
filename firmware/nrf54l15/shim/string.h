/*
 * Freestanding <string.h> shim.
 *
 * The ARM toolchain here is gcc-only (no newlib), and this firmware links
 * -nostdlib by design: a wearable does not want newlib's malloc, locale tables
 * or reentrancy structures dragged in because one file included <string.h>.
 *
 * The sensor manager core uses memcpy and memset. GCC also emits implicit
 * calls to both when copying structs or zeroing arrays, so the symbols must
 * exist regardless of what the C source names. They are implemented in
 * libc_shim.c.
 */
#ifndef SHIM_STRING_H
#define SHIM_STRING_H

#include <stddef.h>

void *memcpy(void *dst, const void *src, size_t n);
void *memmove(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);

#endif /* SHIM_STRING_H */
