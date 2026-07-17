#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#include "types.h"

// bounds checked little endian binary i/o over caller owned byte storage
// scalar values are encoded explicitly, so their on disk representation does
// not depend on the host byte order
namespace binary
{
    class reader
    {
    public:
        reader(const byte *data, std::size_t size)
            : data_(data), size_(size) {}

        explicit reader(const std::vector<byte> &data)
            : reader(data.data(), data.size()) {}

        std::size_t position() const {
            return position_;
        }

        std::size_t remaining() const {
            return size_ - position_;
        }

        const byte *current() const {
            return position_ ? data_ + position_ : data_;
        }

        bool seek(std::size_t offset)
        {
            if (offset > size_)
                return false;
            position_ = offset;
            return true;
        }

        bool skip(std::size_t count)
        {
            if (count > remaining())
                return false;
            position_ += count;
            return true;
        }

        bool take(void *destination, std::size_t count)
        {
            if (count > remaining())
                return false;
            if (count)
                std::memcpy(destination, data_ + position_, count);
            position_ += count;
            return true;
        }

        bool u8(byte &value) {
            return take(&value, 1);
        }

        bool u16(std::uint16_t &value)
        {
            byte bytes[2];
            if (!take(bytes, sizeof(bytes)))
                return false;
            value = static_cast<std::uint16_t>(bytes[0])
                | (static_cast<std::uint16_t>(bytes[1]) << 8);
            return true;
        }

        bool u32(std::uint32_t &value)
        {
            byte bytes[4];
            if (!take(bytes, sizeof(bytes)))
                return false;
            value = static_cast<std::uint32_t>(bytes[0])
                | (static_cast<std::uint32_t>(bytes[1]) << 8)
                | (static_cast<std::uint32_t>(bytes[2]) << 16)
                | (static_cast<std::uint32_t>(bytes[3]) << 24);
            return true;
        }

        bool i16(std::int16_t &value)
        {
            std::uint16_t raw;
            if (!u16(raw))
                return false;
            value = static_cast<std::int16_t>(raw);
            return true;
        }

        bool i32(std::int32_t &value)
        {
            std::uint32_t raw;
            if (!u32(raw))
                return false;
            value = static_cast<std::int32_t>(raw);
            return true;
        }

        bool f32(float &value)
        {
            static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit float required");
            std::uint32_t raw;
            if (!u32(raw))
                return false;
            std::memcpy(&value, &raw, sizeof(value));
            return true;
        }

        bool u16_at(std::size_t offset, std::uint16_t &value) const {
            reader cursor(data_, size_);
            return cursor.seek(offset) && cursor.u16(value);
        }

        bool u32_at(std::size_t offset, std::uint32_t &value) const {
            reader cursor(data_, size_);
            return cursor.seek(offset) && cursor.u32(value);
        }

        bool i16_at(std::size_t offset, std::int16_t &value) const {
            reader cursor(data_, size_);
            return cursor.seek(offset) && cursor.i16(value);
        }

        bool i32_at(std::size_t offset, std::int32_t &value) const {
            reader cursor(data_, size_);
            return cursor.seek(offset) && cursor.i32(value);
        }

        bool f32_at(std::size_t offset, float &value) const {
            reader cursor(data_, size_);
            return cursor.seek(offset) && cursor.f32(value);
        }

    private:
        const byte *data_;
        std::size_t size_;
        std::size_t position_ = 0;
    };

    class writer
    {
    public:
        explicit writer(std::vector<byte> &output) : output_(output) {}

        std::size_t position() const {
            return output_.size();
        }

        void u8(byte value) {
            output_.push_back(value);
        }

        void u16(std::uint16_t value)
        {
            output_.push_back(static_cast<byte>(value));
            output_.push_back(static_cast<byte>(value >> 8));
        }

        void u32(std::uint32_t value)
        {
            output_.push_back(static_cast<byte>(value));
            output_.push_back(static_cast<byte>(value >> 8));
            output_.push_back(static_cast<byte>(value >> 16));
            output_.push_back(static_cast<byte>(value >> 24));
        }

        void i16(std::int16_t value) {
            u16(static_cast<std::uint16_t>(value));
        }

        void i32(std::int32_t value) {
            u32(static_cast<std::uint32_t>(value));
        }

        void f32(float value)
        {
            static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit float required");
            std::uint32_t raw;
            std::memcpy(&raw, &value, sizeof(raw));
            u32(raw);
        }

        void raw(const byte *data, std::size_t size) {
            if (size)
                output_.insert(output_.end(), data, data + size);
        }

        void raw(const std::vector<byte> &data) {
            raw(data.data(), data.size());
        }

        bool patch_u16(std::size_t offset, std::uint16_t value) {
            return patch(offset, value, 2);
        }

        bool patch_u32(std::size_t offset, std::uint32_t value) {
            return patch(offset, value, 4);
        }

        bool patch_i16(std::size_t offset, std::int16_t value) {
            return patch_u16(offset, static_cast<std::uint16_t>(value));
        }

        bool patch_i32(std::size_t offset, std::int32_t value) {
            return patch_u32(offset, static_cast<std::uint32_t>(value));
        }

    private:
        bool patch(std::size_t offset, std::uint32_t value, std::size_t size)
        {
            if (offset > output_.size() || size > output_.size() - offset)
                return false;
            for (std::size_t i = 0; i < size; i++)
                output_[offset + i] = static_cast<byte>(value >> (i * 8));
            return true;
        }

        std::vector<byte> &output_;
    };
}
