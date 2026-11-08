/* Tiny math helpers so the firmware links without libm (Proteus VSM Studio does not pass -lm).
 * Cortex-M4F: sqrtf is a single VSQRT.F32 instruction. */
#ifndef MATHLITE_H
#define MATHLITE_H
static inline float  ml_sqrtf(float x) { float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x)); return r; }
static inline double ml_sqrt(double x) { return (double)ml_sqrtf((float)x); }
static inline float  ml_fabsf(float x) { return x < 0 ? -x : x; }
static inline double ml_fabs(double x) { return x < 0 ? -x : x; }
static inline float  ml_fmaxf(float a, float b) { return a > b ? a : b; }
#endif
