#include <cstdint>
#include <vector>

#include "common/binary.h"
#include "support/test.h"

suite("unit.common.binary_io")
{
    test("binary_io.round trips little-endian scalar values")
    {
        std::vector<byte> data;
        binary::writer output(data);
        output.u8(0x12);
        output.u16(0x3456);
        output.u32(0x789abcde);
        output.i16(-1234);
        output.i32(-12345678);
        output.f32(12.5f);

        require(data.size() == 17);
        expect(data[1] == 0x56);
        expect(data[2] == 0x34);
        expect(data[3] == 0xde);
        expect(data[6] == 0x78);

        binary::reader input(data);
        byte u8 = 0;
        std::uint16_t u16 = 0;
        std::uint32_t u32 = 0;
        std::int16_t i16 = 0;
        std::int32_t i32 = 0;
        float f32 = 0;
        require(input.u8(u8));
        require(input.u16(u16));
        require(input.u32(u32));
        require(input.i16(i16));
        require(input.i32(i32));
        require(input.f32(f32));
        expect(u8 == 0x12);
        expect(u16 == 0x3456);
        expect(u32 == 0x789abcde);
        expect(i16 == -1234);
        expect(i32 == -12345678);
        expect(f32 == 12.5f);
        expect(input.remaining() == 0);
    }

    test("binary_io.rejects truncated reads without moving the cursor")
    {
        std::vector<byte> data = {1, 2, 3};
        binary::reader input(data);
        std::uint32_t value = 0;
        std::int32_t signed_value = 0;

        expect_false(input.u32(value));
        expect(input.position() == 0);
        expect_false(input.i32_at(0, signed_value));
        expect(input.position() == 0);
    }

    test("binary_io.patches existing fields without touching out-of-bounds data")
    {
        std::vector<byte> data;
        binary::writer output(data);
        output.u32(0);

        require(output.patch_u32(0, 0x01020304));
        expect(data == std::vector<byte>{4, 3, 2, 1});

        std::vector<byte> before = data;
        expect_false(output.patch_u32(1, 0xdeadbeef));
        expect(data == before);
    }
}
