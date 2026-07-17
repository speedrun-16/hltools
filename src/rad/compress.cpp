#include "compress.h"

#include <cstring>

#include "../common/error.h"

namespace rad
{
    const char *float_format_string[] =
    {
        "32bit",
        "16bit",
        "8bit",
    };

    const size_t float_format_size[] =
    {
        4u,
        2u,
        1u,
    };

    const char *vector_format_string[] =
    {
        "96bit",
        "48bit",
        "32bit",
        "24bit",
    };

    const size_t vector_format_size[] =
    {
        12u,
        6u,
        4u,
        3u,
    };

    namespace
    {
        [[noreturn]] void fail()
        {
            err::fatal("compress compatability test failed: this platform does not match the ieee bit layout the transfer packing relies on");
        }
    }

    void compress_compatability_test()
    {
        unsigned char v[16];
        std::memset(v, 0, 16u);
        if (sizeof(char) != 1 || sizeof(unsigned int) != 4 || sizeof(float) != 4)
            fail();
        *(float *)(v + 1) = 0.123f;
        if (*(unsigned int *)v != 4226247936u || *(unsigned int *)(v + 1) != 1039918957u)
            fail();
        *(float *)(v + 1) = -58;
        if (*(unsigned int *)v != 1744830464u || *(unsigned int *)(v + 1) != 3261595648u)
            fail();
        float f[5] = {0.123f, 1.f, 0.f, 0.123f, 0.f};
        std::memset(v, ~0, 16u);
        vector_compress(vector_format::vector24, v, &f[0], &f[1], &f[2]);
        float_compress(float_format::float16, v + 6, &f[3]);
        float_compress(float_format::float16, v + 4, &f[4]);
        if (((unsigned int *)v)[0] != 4286318595u || ((unsigned int *)v)[1] != 3753771008u)
            fail();
        float_decompress(float_format::float16, v + 6, &f[3]);
        float_decompress(float_format::float16, v + 4, &f[4]);
        vector_decompress(vector_format::vector24, v, &f[0], &f[1], &f[2]);
        float ans[5] = {0.109375f, 1.015625f, 0.015625f, 0.123001f, 0.000000f};
        for (int i = 0; i < 5; ++i)
            if (f[i] - ans[i] > 0.00001f || f[i] - ans[i] < -0.00001f)
                fail();
    }
}
