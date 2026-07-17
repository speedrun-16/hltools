#include "log.h"

#include "build_info.h"
#include "filesystem.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace logging
{
    namespace
    {
        constexpr int max_warning_len = 512;
        constexpr int max_collected_warnings = 128;

        std::mutex g_mutex;
        std::ofstream g_file;

        char g_warnings[max_collected_warnings][max_warning_len];
        unsigned g_num_warnings = 0;
        bool g_leaked = false;
        bool g_error = false;

        struct setting_row
        {
            std::string label;
            std::string value;
            std::string def;
            bool changed;
        };
        std::vector<setting_row> g_settings;

        // write to the console, the logfile, or both, from one formatted buffer
        void emit(bool to_console, bool to_file, const char *msg)
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            if (to_console)
            {
                std::cout << msg << std::flush;
            }
            if (to_file && g_file.is_open())
            {
                g_file << msg;
                g_file.flush();
            }
        }

        void vformat(char *buf, size_t cap, const char *fmt, va_list ap)
        {
            std::vsnprintf(buf, cap, fmt, ap);
        }
    }

    // ============================================================================
    // file
    // ============================================================================

    void open(const char *log_path)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_file.is_open())
            g_file.close();
        g_file.open(log_path, std::ios::out | std::ios::trunc);
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_file.is_open())
            g_file.close();
    }

    void open_stage_log(const std::string &base, const char *stage)
    {
        std::string dir = fs::directory(base);
        std::string logdir = dir.empty() ? "logs" : dir + "/logs";
        fs::make_directory(logdir);
        std::string path = logdir + "/" + fs::filename(base) + "." + stage + ".log";
        open(path.c_str());
    }

    // ============================================================================
    // channels
    // ============================================================================

    void info(const char *fmt, ...)
    {
        char buf[2048];
        va_list ap;
        va_start(ap, fmt);
        vformat(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        emit(true, true, buf);
    }

    void file(const char *fmt, ...)
    {
        char buf[2048];
        va_list ap;
        va_start(ap, fmt);
        vformat(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        emit(false, true, buf);
    }

    void console(const char *fmt, ...)
    {
        // tool help is intentionally detailed and can be several kilobytes
        // keep this larger than the file/info scratch buffers so a single
        // help page is not silently truncated halfway through
        char buf[16384];
        va_list ap;
        va_start(ap, fmt);
        vformat(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        emit(true, false, buf);
    }

    // ============================================================================
    // warnings
    // ============================================================================

    void add_warning_summary(const char *message)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_num_warnings < max_collected_warnings)
        {
            std::strncpy(g_warnings[g_num_warnings], message, max_warning_len - 1);
            g_warnings[g_num_warnings][max_warning_len - 1] = '\0';
        }
        g_num_warnings++;
    }

    void warn(const char *fmt, ...)
    {
        char buf[max_warning_len];
        va_list ap;
        va_start(ap, fmt);
        vformat(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        add_warning_summary(buf);

        char line[max_warning_len + 16];
        std::snprintf(line, sizeof(line), "warning: %s\n", buf);
        emit(true, true, line);
    }

    unsigned warning_count()
    {
        return g_num_warnings;
    }

    void print_warnings_summary()
    {
        if (g_num_warnings == 0)
            return;
        unsigned stored = g_num_warnings < max_collected_warnings ? g_num_warnings
                                                                   : max_collected_warnings;
        info("\nwarnings (%u):\n", g_num_warnings);
        for (unsigned i = 0; i < stored; i++)
            info("  ! %s\n", g_warnings[i]);
        if (g_num_warnings > stored)
            info("  ... and %u more (see above)\n", g_num_warnings - stored);
    }

    // ============================================================================
    // footer state
    // ============================================================================

    void set_leaked() {
        g_leaked = true;
    }
    void set_error() {
        g_error = true;
    }
    bool had_error() {
        return g_error;
    }
    bool had_leak() {
        return g_leaked;
    }

    // ============================================================================
    // banner and footer
    // ============================================================================

    void init_console()
    {
#ifdef _WIN32
        // render any utf 8 output (map paths etc) correctly regardless of
        // the console's default code page
        SetConsoleOutputCP(CP_UTF8);
#endif
    }

    void banner(const char *scope)
    {
        const std::string compiler = build_info::compiler(scope);
        info("\n");
        info("================================================================\n");
        info(" %s\n", compiler.c_str());
        info(" https://github.com/speedrun-16/hltools\n");
        info("================================================================\n\n");
    }

    void done(double elapsed_seconds)
    {
        print_warnings_summary();
        unsigned nwarn = g_num_warnings;
        const char *status = g_error ? " (with error)"
                           : g_leaked ? " (with leak)"
                           : nwarn ? " (with warnings)"
                           : "";
        char human[32];
        if (elapsed_seconds < 60.0)
            std::snprintf(human, sizeof(human), "%.2fs", elapsed_seconds);
        else
            std::snprintf(human, sizeof(human), "%dm %02ds", (int)elapsed_seconds / 60,
                          (int)elapsed_seconds % 60);
        if (nwarn)
            info("\ndone%s in %.1fs (%s) -- %u warning%s\n", status, elapsed_seconds, human,
                 nwarn, nwarn == 1 ? "" : "s");
        else
            info("\ndone%s in %.1fs (%s)\n", status, elapsed_seconds, human);
    }

    // ============================================================================
    // settings table
    // ============================================================================

    void setting(const char *label, const char *value, const char *default_desc, bool changed)
    {
        g_settings.push_back({label, value, default_desc ? default_desc : "", changed});
    }

    void flush_settings()
    {
        bool any_changed = false;
        for (const auto &row : g_settings)
            any_changed |= row.changed;

        // logfile always gets the full table; console gets it only if something
        // actually changed from the defaults
        file("\nsettings (changed from default):\n");
        if (any_changed)
            console("\nsettings (changed from default):\n");

        for (const auto &row : g_settings)
        {
            char line[256];
            if (row.def.empty())
                std::snprintf(line, sizeof(line), "  %-22s %s\n",
                              row.label.c_str(), row.value.c_str());
            else
                std::snprintf(line, sizeof(line), "  %-22s %-18s (default: %s)\n",
                              row.label.c_str(), row.value.c_str(), row.def.c_str());
            file("%s", line);
            if (row.changed)
                console("%s", line);
        }
        g_settings.clear();
    }
}
