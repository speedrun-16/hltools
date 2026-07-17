#pragma once

#include <cstddef>

#include "rad.h"

// lossy bit packed storage for patch transfer data the packed bytes are part
// of the lighting math: transfers are compressed once and decompressed every
// bounce, so the exact rounding of these routines feeds the output lightmap

namespace rad
{
    // bytes at the end of a transfer block the packing never touches
    constexpr size_t transfer_unused_size = 3;

    extern const char *float_format_string[];
    extern const size_t float_format_size[];
    extern const char *vector_format_string[];
    extern const size_t vector_format_size[];

    // run once at startup: verifies the ieee bit layout assumptions below
    // hold on this platform
    void compress_compatability_test();

    inline unsigned int bitget(unsigned int i, unsigned int start, unsigned int end)
    {
        return (i & ~(~0u << end)) >> start;
    }

    inline unsigned int bitput(unsigned int i, unsigned int start, unsigned int)
    {
        return i << start;
    }

    inline unsigned int bitclr(unsigned int i, unsigned int start, unsigned int end)
    {
        return i & (~(~0u << start) | (~0u << end));
    }

    inline unsigned int float_iswrong(unsigned int i)
    {
        return i >= 0x7F800000u;
    }

    inline unsigned int float_istoobig(unsigned int i)
    {
        return i >= 0x40000000u;
    }

    inline unsigned int float_istoosmall(unsigned int i)
    {
        return i < 0x30800000u;
    }

    inline void float_compress(float_format t, void *s, const float *f)
    {
        unsigned int *m = (unsigned int *)s;
        const unsigned int *p = (const unsigned int *)f;
        switch (t)
        {
        case float_format::float32:
            m[0] = *p;
            break;
        case float_format::float16:
            m[0] = bitclr(m[0], 0, 16);
            if (float_iswrong(*p))
                ;
            else if (float_istoobig(*p))
                m[0] |= bitget(~0u, 0, 16);
            else if (float_istoosmall(*p))
                ;
            else
                m[0] |= bitget(*p, 12, 28);
            break;
        case float_format::float8:
            m[0] = bitclr(m[0], 0, 8);
            if (float_iswrong(*p))
                ;
            else if (float_istoobig(*p))
                m[0] |= bitget(~0u, 0, 8);
            else if (float_istoosmall(*p))
                ;
            else
                m[0] |= bitget(*p, 20, 28);
            break;
        }
    }

    inline void float_decompress(float_format t, const void *s, float *f)
    {
        const unsigned int *m = (const unsigned int *)s;
        unsigned int *p = (unsigned int *)f;
        switch (t)
        {
        case float_format::float32:
            *p = m[0];
            break;
        case float_format::float16:
            if (bitget(m[0], 0, 16) == 0)
                *p = 0;
            else
                *p = bitput(1, 11, 12)
                    | bitput(bitget(m[0], 0, 16), 12, 28)
                    | bitput(3, 28, 32)
                    ;
            break;
        case float_format::float8:
            if (bitget(m[0], 0, 8) == 0)
                *p = 0;
            else
                *p = bitput(1, 19, 20)
                    | bitput(bitget(m[0], 0, 8), 20, 28)
                    | bitput(3, 28, 32)
                    ;
            break;
        }
    }

    inline void vector_compress(vector_format t, void *s, const float *f1, const float *f2, const float *f3)
    {
        unsigned int *m = (unsigned int *)s;
        const unsigned int *p1 = (const unsigned int *)f1;
        const unsigned int *p2 = (const unsigned int *)f2;
        const unsigned int *p3 = (const unsigned int *)f3;
        unsigned int max, i1, i2, i3;
        switch (t)
        {
        case vector_format::vector96:
            m[0] = *p1;
            m[1] = *p2;
            m[2] = *p3;
            break;
        case vector_format::vector48:
            if (float_iswrong(*p1) || float_iswrong(*p2) || float_iswrong(*p3))
                break;
            m[0] = 0, m[1] = bitclr(m[1], 0, 16);
            if (float_istoobig(*p1))
                m[0] |= bitget(~0u, 0, 16);
            else if (float_istoosmall(*p1))
                ;
            else
                m[0] |= bitget(*p1, 12, 28);
            if (float_istoobig(*p2))
                m[0] |= bitput(bitget(~0u, 0, 16), 16, 32);
            else if (float_istoosmall(*p2))
                ;
            else
                m[0] |= bitput(bitget(*p2, 12, 28), 16, 32);
            if (float_istoobig(*p3))
                m[1] |= bitget(~0u, 0, 16);
            else if (float_istoosmall(*p3))
                ;
            else
                m[1] |= bitget(*p3, 12, 28);
            break;
        case vector_format::vector32:
        case vector_format::vector24:
            if (float_iswrong(*p1) || float_iswrong(*p2) || float_iswrong(*p3))
            {
                max = i1 = i2 = i3 = 0;
            }
            else
            {
                max = *p1 > *p2 ? (*p1 > *p3 ? *p1 : *p3) : (*p2 > *p3 ? *p2 : *p3);
                max = float_istoobig(max) ? 0x7F : float_istoosmall(max) ? 0x60 : bitget(max, 23, 31);
                i1 = float_istoobig(*p1) ? ~0u : (bitget(*p1, 0, 23) | bitput(1, 23, 24)) >> (1 + max - bitget(*p1, 23, 31));
                i2 = float_istoobig(*p2) ? ~0u : (bitget(*p2, 0, 23) | bitput(1, 23, 24)) >> (1 + max - bitget(*p2, 23, 31));
                i3 = float_istoobig(*p3) ? ~0u : (bitget(*p3, 0, 23) | bitput(1, 23, 24)) >> (1 + max - bitget(*p3, 23, 31));
            }
            if (t == vector_format::vector32)
                m[0] = 0
                    | bitput(bitget(i1, 14, 23), 0, 9)
                    | bitput(bitget(i2, 14, 23), 9, 18)
                    | bitput(bitget(i3, 14, 23), 18, 27)
                    | bitput(bitget(max, 0, 5), 27, 32)
                    ;
            else
                m[0] = bitclr(m[0], 0, 24)
                    | bitput(bitget(i1, 17, 23), 0, 6)
                    | bitput(bitget(i2, 17, 23), 6, 12)
                    | bitput(bitget(i3, 17, 23), 12, 18)
                    | bitput(bitget(max, 0, 5), 18, 23)
                    ;
            break;
        }
    }

    inline void vector_decompress(vector_format t, const void *s, float *f1, float *f2, float *f3)
    {
        const unsigned int *m = (const unsigned int *)s;
        unsigned int *p1 = (unsigned int *)f1;
        unsigned int *p2 = (unsigned int *)f2;
        unsigned int *p3 = (unsigned int *)f3;
        switch (t)
        {
        case vector_format::vector96:
            *p1 = m[0];
            *p2 = m[1];
            *p3 = m[2];
            break;
        case vector_format::vector48:
            if (bitget(m[0], 0, 16) == 0)
                *p1 = 0;
            else
                *p1 = bitput(1, 11, 12)
                    | bitput(bitget(m[0], 0, 16), 12, 28)
                    | bitput(3, 28, 32)
                    ;
            if (bitget(m[0], 16, 32) == 0)
                *p2 = 0;
            else
                *p2 = bitput(1, 11, 12)
                    | bitput(bitget(m[0], 16, 32), 12, 28)
                    | bitput(3, 28, 32)
                    ;
            if (bitget(m[1], 0, 16) == 0)
                *p3 = 0;
            else
                *p3 = bitput(1, 11, 12)
                    | bitput(bitget(m[1], 0, 16), 12, 28)
                    | bitput(3, 28, 32)
                    ;
            break;
        case vector_format::vector32:
        case vector_format::vector24:
            float f;
            if (t == vector_format::vector32)
            {
                *p1 = bitput(1, 13, 14)
                    | bitput(bitget(m[0], 0, 9), 14, 23)
                    | bitput(bitget(m[0], 27, 32), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *p2 = bitput(1, 13, 14)
                    | bitput(bitget(m[0], 9, 18), 14, 23)
                    | bitput(bitget(m[0], 27, 32), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *p3 = bitput(1, 13, 14)
                    | bitput(bitget(m[0], 18, 27), 14, 23)
                    | bitput(bitget(m[0], 27, 32), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *((unsigned int *)&f)
                    = bitput(bitget(m[0], 27, 32), 23, 28)
                    | bitput(3, 28, 32)
                    ;
            }
            else
            {
                *p1 = bitput(1, 16, 17)
                    | bitput(bitget(m[0], 0, 6), 17, 23)
                    | bitput(bitget(m[0], 18, 23), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *p2 = bitput(1, 16, 17)
                    | bitput(bitget(m[0], 6, 12), 17, 23)
                    | bitput(bitget(m[0], 18, 23), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *p3 = bitput(1, 16, 17)
                    | bitput(bitget(m[0], 12, 18), 17, 23)
                    | bitput(bitget(m[0], 18, 23), 23, 28)
                    | bitput(3, 28, 32)
                    ;
                *((unsigned int *)&f)
                    = bitput(bitget(m[0], 18, 23), 23, 28)
                    | bitput(3, 28, 32)
                    ;
            }
            *f1 = (*f1 - f) * 2.f;
            *f2 = (*f2 - f) * 2.f;
            *f3 = (*f3 - f) * 2.f;
            break;
        }
    }
}
