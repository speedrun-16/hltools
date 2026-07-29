#include "build_info.h"

#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace build_info
{
    namespace
    {
        std::string format_date(const char *build_date)
        {
            static constexpr const char *month_abbreviations[] = {
                "jan", "feb", "mar", "apr", "may", "jun",
                "jul", "aug", "sep", "oct", "nov", "dec",
            };
            static constexpr const char *month_names[] = {
                "january", "february", "march", "april", "may", "june",
                "july", "august", "september", "october", "november", "december",
            };

            char month[4]{};
            int day = 0;
            int year = 0;
            if (build_date && std::sscanf(build_date, "%3s %d %d", month, &day, &year) == 3)
            {
                for (char *p = month; *p; p++)
                    *p = (char)std::tolower((unsigned char)*p);

                for (size_t i = 0; i < 12; i++)
                {
                    if (std::strcmp(month, month_abbreviations[i]) == 0)
                    {
                        char result[32];
                        std::snprintf(result, sizeof(result), "%d %s %d",
                                      day, month_names[i], year);
                        return result;
                    }
                }
            }

            std::string result = build_date ? build_date : "";
            for (char &c : result)
                c = (char)std::tolower((unsigned char)c);
            return result;
        }

        std::string format_compiler(const char *scope)
        {
            std::string result = std::string("hltools ") + version();
            if (scope && scope[0])
                result += std::string(" - ") + scope + " ";
            result += " (" + date() + ", " + commit() + ")";
            return result;
        }

        std::string format_compiled_at()
        {
            std::time_t value = std::time(nullptr);
            // Reproducible build environments conventionally provide this
            // timestamp in seconds since the Unix epoch.
            if (const char *epoch = std::getenv("SOURCE_DATE_EPOCH"))
            {
                char *end = nullptr;
                long long parsed = std::strtoll(epoch, &end, 10);
                if (end && *end == '\0' && parsed >= 0)
                    value = (std::time_t)parsed;
            }

            std::tm utc{};
#if defined(_WIN32)
            if (gmtime_s(&utc, &value) != 0)
                return {};
#else
            if (!gmtime_r(&value, &utc))
                return {};
#endif
            char result[32]{};
            if (std::strftime(result, sizeof(result),
                              "%Y-%m-%dT%H:%M:%SZ", &utc) == 0)
                return {};
            return result;
        }
    }

    const char *version()
    {
        return HLTOOLS_VERSION;
    }

    const char *commit()
    {
        return HLTOOLS_COMMIT;
    }

    const std::string &date()
    {
        static const std::string value = format_date(__DATE__);
        return value;
    }

    const std::string &compiler()
    {
        static const std::string value = format_compiler(nullptr);
        return value;
    }

    std::string compiler(const char *scope)
    {
        return format_compiler(scope);
    }

    const std::string &compiled_at()
    {
        static const std::string value = format_compiled_at();
        return value;
    }
}
