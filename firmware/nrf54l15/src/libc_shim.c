/*
 * Minimal freestanding memcpy/memmove/memset.
 *
 * Byte-at-a-time and deliberately simple: correctness over speed. The hot path
 * that matters is the ring buffer's bulk copy-out, and if profiling ever shows
 * it on the critical path the fix is a word-at-a-time version guarded by an
 * alignment check — not a cleverer loop here.
 *
 * -fno-tree-loop-distribute-patterns is set in the Makefile so GCC does not
 * "optimise" these implementations into calls to themselves.
 */
#include <string.h>

void *memcpy(void *dst, const void *src, size_t n)
{
    unsigned char       *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    while (n--) {
        *d++ = *s++;
    }
    return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
    unsigned char       *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    if (d == s || n == 0) {
        return dst;
    }
    /* Overlap: copy backwards when the destination is inside the source. */
    if (d < s) {
        while (n--) {
            *d++ = *s++;
        }
    } else {
        d += n;
        s += n;
        while (n--) {
            *--d = *--s;
        }
    }
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    unsigned char *d = (unsigned char *)dst;

    while (n--) {
        *d++ = (unsigned char)c;
    }
    return dst;
}
