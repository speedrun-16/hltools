#include "error.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "log.h"

namespace err
{
    namespace
    {
        [[noreturn]] void report_and_exit(const char *msg)
        {
            logging::info("\nerror: %s\n", msg);
            logging::close();
            std::exit(1);
        }
    }

    void fatal(const char *fmt, ...)
    {
        char buf[2048];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        report_and_exit(buf);
    }

    void require(bool condition, const char *fmt, ...)
    {
        if (condition)
            return;
        char buf[2048];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        report_and_exit(buf);
    }
}
