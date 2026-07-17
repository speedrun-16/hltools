#include "build_info.h"

#include <cctype>
#include <cstdio>
#include <cstring>

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
}
