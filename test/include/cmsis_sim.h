/*
 * cmsis_sim.h
 *
 * Force-included into every simulation build (-include). Replaces cmsis_gcc.h, whose ARM inline
 * assembly can't be compiled for the PC, with equivalents that drive the simulated interrupt mask.
 */

#ifndef CMSIS_SIM_H
#define CMSIS_SIM_H

#define __CMSIS_GCC_H       // Stop cmsis_compiler.h pulling in the real cmsis_gcc.h

#include <stdint.h>

#ifndef __has_builtin
  #define __has_builtin(x) (0)
#endif

#define __ASM                       __asm
#define __INLINE                    inline
#define __STATIC_INLINE             static inline
#define __STATIC_FORCEINLINE        static inline
#define __NO_RETURN                 __attribute__((__noreturn__))
#define __USED                      __attribute__((used))
#define __WEAK                      __attribute__((weak))
#define __PACKED                    __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT             struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION              union __attribute__((packed, aligned(1)))
#define __ALIGNED(x)                __attribute__((aligned(x)))
#define __RESTRICT                  __restrict
#define __COMPILER_BARRIER()        __asm volatile("" ::: "memory")

struct __attribute__((packed)) T_UINT32 { uint32_t v; };
__PACKED_STRUCT T_UINT16_WRITE { uint16_t v; };
__PACKED_STRUCT T_UINT16_READ  { uint16_t v; };
__PACKED_STRUCT T_UINT32_WRITE { uint32_t v; };
__PACKED_STRUCT T_UINT32_READ  { uint32_t v; };
#define __UNALIGNED_UINT32(x)               (((struct T_UINT32 *)(x))->v)
#define __UNALIGNED_UINT16_WRITE(addr, val) (void)((((struct T_UINT16_WRITE *)(void *)(addr))->v) = (val))
#define __UNALIGNED_UINT16_READ(addr)       (((const struct T_UINT16_READ *)(const void *)(addr))->v)
#define __UNALIGNED_UINT32_WRITE(addr, val) (void)((((struct T_UINT32_WRITE *)(void *)(addr))->v) = (val))
#define __UNALIGNED_UINT32_READ(addr)       (((const struct T_UINT32_READ *)(const void *)(addr))->v)

// Interrupt mask, implemented by the simulator (sim_core.c)
void     __enable_irq(void);
void     __disable_irq(void);
uint32_t __get_PRIMASK(void);
void     __set_PRIMASK(uint32_t priMask);
uint32_t __get_IPSR(void);

#define __NOP()     ((void)0)
#define __WFI()     ((void)0)
#define __WFE()     ((void)0)
#define __SEV()     ((void)0)
#define __BKPT(v)   ((void)0)

static inline void     __ISB(void) {}
static inline void     __DSB(void) {}
static inline void     __DMB(void) {}
static inline uint32_t __REV(uint32_t v)   { return __builtin_bswap32(v); }
static inline uint32_t __REV16(uint32_t v) { return ((v & 0xFF00FF00U) >> 8) | ((v & 0x00FF00FFU) << 8); }
static inline int16_t  __REVSH(int16_t v)  { return (int16_t)__builtin_bswap16((uint16_t)v); }
static inline uint32_t __ROR(uint32_t a, uint32_t b) { b %= 32U; return b ? (a >> b) | (a << (32U - b)) : a; }
static inline uint8_t  __CLZ(uint32_t v)   { return v ? (uint8_t)__builtin_clz(v) : 32U; }
static inline uint32_t __RBIT(uint32_t v)
{
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) { r = (r << 1) | (v & 1U); v >>= 1; }
    return r;
}
static inline uint32_t __get_FPSCR(void) { return 0; }
static inline void     __set_FPSCR(uint32_t v) { (void)v; }

#endif /* CMSIS_SIM_H */
