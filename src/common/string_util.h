#pragma once

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// bounded string helpers that always null terminate, replacing the old
// safe_snprintf / safe_strncpy wrappers header only, no state

namespace str
{
    // bounded copy that always leaves dst null terminated
    inline void copy(char *dst, size_t cap, const char *src)
    {
        if (cap == 0)
            return;
        std::strncpy(dst, src, cap - 1);
        dst[cap - 1] = '\0';
    }

    // bounded printf into dst, always null terminated
    inline void format(char *dst, size_t cap, const char *fmt, ...)
    {
        if (cap == 0)
            return;
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(dst, cap, fmt, ap);
        va_end(ap);
    }

    inline bool iequals(const char *a, const char *b)
    {
        for (; *a && *b; a++, b++)
        {
            if (std::tolower((unsigned char)*a) != std::tolower((unsigned char)*b))
                return false;
        }
        return *a == *b;
    }

    inline bool starts_with(const char *s, const char *prefix)
    {
        return std::strncmp(s, prefix, std::strlen(prefix)) == 0;
    }

    inline bool istarts_with(const char *s, const char *prefix)
    {
        while (*prefix)
        {
            if (std::tolower((unsigned char)*s) != std::tolower((unsigned char)*prefix))
                return false;
            s++;
            prefix++;
        }
        return true;
    }

    // group a non negative integer with thousands separators into buf (>= 32)
    inline const char *with_commas(long long n, char *buf, size_t cap)
    {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%lld", n);
        int len = (int)std::strlen(tmp);
        int neg = tmp[0] == '-' ? 1 : 0;
        int digits = len - neg;
        int commas = (digits - 1) / 3;
        int out_len = len + commas;
        if ((size_t)out_len + 1 > cap)
        {
            copy(buf, cap, tmp);
            return buf;
        }
        buf[out_len] = '\0';
        int bi = out_len - 1;
        int count = 0;
        for (int i = len - 1; i >= neg; i--)
        {
            buf[bi--] = tmp[i];
            if (++count % 3 == 0 && i > neg)
                buf[bi--] = ',';
        }
        if (neg)
            buf[0] = '-';
        return buf;
    }

    // format a byte count as "n bytes" / "nn kb" / "nnn mb" into buf (>= 32)
    inline const char *human_bytes(long long bytes, char *buf, size_t cap)
    {
        if (bytes >= 1024 * 1024)
            std::snprintf(buf, cap, "%.2f MB", bytes / (1024.0 * 1024.0));
        else if (bytes >= 1024)
            std::snprintf(buf, cap, "%.1f KB", bytes / 1024.0);
        else
            std::snprintf(buf, cap, "%lld bytes", bytes);
        return buf;
    }
}
