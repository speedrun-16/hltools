#include "threads.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "progress.h"

namespace threads
{
    namespace
    {
        int g_count = 0; // 0 means auto
        thread_local bool tls_in_pool_worker = false;
        thread_local bool tls_priority_lowered = false;

        void restore_priority_if_lowered()
        {
            if (tls_priority_lowered)
            {
#ifdef _WIN32
                SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
#endif
                tls_priority_lowered = false;
            }
        }

        // Persistent worker pool for the primary caller of threads::run.
        // Avoids per-phase OS thread spawn/join overhead and keeps worker
        // thread_local scratch buffers warm across phases. Nested or
        // concurrent callers fall back to ad-hoc threads without blocking.
        struct worker_pool
        {
            std::mutex mtx;
            std::condition_variable cv_work;
            std::condition_variable cv_done;
            std::vector<std::thread> workers;

            bool stop = false;
            uint64_t generation = 0;
            int active_workers = 0;
            int work_count = 0;
            std::atomic<int> next_item{0};
            const std::function<void(int)> *work_fn = nullptr;
            int remaining = 0;

            std::atomic<bool> busy{false};

            ~worker_pool()
            {
                shutdown();
            }

            void shutdown()
            {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    stop = true;
                }
                cv_work.notify_all();
                for (auto &th : workers)
                {
                    if (th.joinable())
                        th.join();
                }
                workers.clear();
                stop = false;
            }

            void ensure_size(int helper_count)
            {
                if ((int)workers.size() >= helper_count)
                    return;
                shutdown();
                workers.reserve((size_t)helper_count);
                for (int id = 0; id < helper_count; id++)
                {
                    workers.emplace_back([this, id]()
                    {
                        tls_in_pool_worker = true;
                        uint64_t seen_gen = 0;
                        while (true)
                        {
                            const std::function<void(int)> *fn = nullptr;
                            int limit = 0;
                            {
                                std::unique_lock<std::mutex> lock(mtx);
                                cv_work.wait(lock, [&]() { return stop || generation != seen_gen; });
                                if (stop)
                                    return;
                                seen_gen = generation;
                                if (id >= active_workers)
                                    continue;
                                fn = work_fn;
                                limit = work_count;
                            }

                            int i;
                            while ((i = next_item.fetch_add(1, std::memory_order_relaxed)) < limit)
                                (*fn)(i);

                            restore_priority_if_lowered();

                            {
                                std::lock_guard<std::mutex> lock(mtx);
                                if (--remaining == 0)
                                    cv_done.notify_one();
                            }
                        }
                    });
                }
            }

            void dispatch(int num_workers, int count, const std::function<void(int)> &fn)
            {
                const int helpers = num_workers - 1;
                ensure_size(helpers);

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    work_count = count;
                    work_fn = &fn;
                    active_workers = helpers;
                    remaining = helpers;
                    next_item.store(0, std::memory_order_relaxed);
                    generation++;
                }
                cv_work.notify_all();

                int i;
                while ((i = next_item.fetch_add(1, std::memory_order_relaxed)) < count)
                    fn(i);
                restore_priority_if_lowered();

                std::unique_lock<std::mutex> lock(mtx);
                cv_done.wait(lock, [&]() { return remaining == 0; });
            }
        };

        worker_pool &get_pool()
        {
            static worker_pool pool;
            return pool;
        }
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

        if (workers <= 1)
        {
            for (int i = 0; i < work_count; i++)
                work_fn(i);
            return;
        }

        worker_pool &pool = get_pool();
        if (!tls_in_pool_worker && !pool.busy.exchange(true, std::memory_order_acquire))
        {
            struct release_guard
            {
                std::atomic<bool> &flag;
                ~release_guard()
                {
                    flag.store(false, std::memory_order_release);
                }
            } guard{pool.busy};
            pool.dispatch(workers, work_count, work_fn);
            return;
        }

        std::atomic<int> next{0};
        auto pump = [&]()
        {
            int i;
            while ((i = next.fetch_add(1, std::memory_order_relaxed)) < work_count)
                work_fn(i);
        };

        // Fallback for nested or concurrent background threads.
        std::vector<std::thread> adhoc;
        adhoc.reserve((size_t)(workers - 1));
        for (int t = 1; t < workers; t++)
            adhoc.emplace_back(pump);
        pump();
        for (auto &th : adhoc)
            th.join();
    }

    void lower_current_priority()
    {
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
        tls_priority_lowered = true;
#endif
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
