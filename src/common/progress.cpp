#include "progress.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>

#include "log.h"

namespace progress
{
    namespace
    {
        constexpr int bar_width = 20;
        constexpr int name_width = 25;

        std::mutex g_mutex;
        std::string g_name;
        int g_total = 0;
        std::atomic<int> g_done{ 0 };
        int g_last_percent = -1;
        std::chrono::steady_clock::time_point g_start;
        std::string g_section; // last section header printed

        double elapsed_seconds()
        {
            auto now = std::chrono::steady_clock::now();
            return std::chrono::duration<double>(now - g_start).count();
        }

        void fill_bar(char *bar, int percent)
        {
            int filled = percent * bar_width / 100;
            for (int i = 0; i < bar_width; i++)
                bar[i] = i < filled ? '#' : ' ';
            bar[bar_width] = '\0';
        }

        // repaint the running bar in place (console only) caller holds g_mutex
        void draw(int percent)
        {
            char bar[bar_width + 1];
            fill_bar(bar, percent);
            logging::console("\r  %-*s [%s] %3d%%  %8.2fs ", name_width, g_name.c_str(),
                             bar, percent, elapsed_seconds());
        }
    }

    void section(const char *name)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_section == name)
            return;
        g_section = name;
        logging::info("\n%s\n", name);
    }

    void begin(const char *name, int total)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_name = name;
        g_total = total > 0 ? total : 1;
        g_done.store(0);
        g_last_percent = -1;
        g_start = std::chrono::steady_clock::now();
        draw(0);
        g_last_percent = 0;
    }

    void set(int done)
    {
        int pct = (int)((long long)done * 100 / g_total);
        if (pct > 100)
            pct = 100;
        // repaint only when the whole number percent advances, to keep the
        // console cheap under a fast inner loop
        if (pct == g_last_percent)
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        if (pct == g_last_percent)
            return;
        g_last_percent = pct;
        draw(pct);
    }

    void add(int n)
    {
        int done = g_done.fetch_add(n, std::memory_order_relaxed) + n;
        set(done);
    }

    void end()
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        double secs = elapsed_seconds();
        char bar[bar_width + 1];
        fill_bar(bar, 100);
        // console: overwrite the live bar, frozen at 100%, then break the line
        logging::console("\r  %-*s [%s] 100%%  %8.2fs \n", name_width, g_name.c_str(), bar, secs);
        // logfile: the same completed line, no carriage return
        logging::file("  %-*s [%s] 100%%  %8.2fs\n", name_width, g_name.c_str(), bar, secs);
    }

    void summary(const char *phase, const char *detail, double secs)
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        // the detail field is padded to the width of the bar block ("[] 100%  ")
        // so the time column lines up with the metered phases above it
        // console: overwrite any live line the caller was painting; logfile: plain
        logging::console("\r  %-*s %-28s %8.2fs \n", name_width, phase, detail, secs);
        logging::file("  %-*s %-28s %8.2fs\n", name_width, phase, detail, secs);
    }
}
