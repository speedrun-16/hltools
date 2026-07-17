#include "threads.h"

#include <atomic>
#include <thread>
#include <vector>

#include "progress.h"

namespace threads
{
    namespace
    {
        int g_count = 0; // 0 means auto
    }

    void set_count(int n) {
        g_count = n;
    }

    int count()
    {
        if (g_count > 0)
            return g_count;
        unsigned hw = std::thread::hardware_concurrency();
        return hw > 0 ? (int)hw : 1;
    }

    void run(int work_count, const std::function<void(int)> &work_fn)
    {
        if (work_count <= 0)
            return;

        int workers = count();
        if (workers > work_count)
            workers = work_count;

        std::atomic<int> next{ 0 };
        auto pump = [&]()
        {
            int i;
            while ((i = next.fetch_add(1, std::memory_order_relaxed)) < work_count)
                work_fn(i);
        };

        // spawn workers - 1 helpers and run one share on the calling thread
        std::vector<std::thread> pool;
        pool.reserve(workers - 1);
        for (int t = 1; t < workers; t++)
            pool.emplace_back(pump);
        pump();
        for (auto &th : pool)
            th.join();
    }

    void run_phase(const char *section, const char *name, int work_count,
                   const std::function<void(int)> &work_fn)
    {
        if (section)
            progress::section(section);
        progress::begin(name, work_count);
        run(work_count, [&](int i)
            {
                work_fn(i);
                progress::add(1);
            });
        progress::end();
    }
}
