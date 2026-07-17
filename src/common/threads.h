#pragma once

#include <functional>

// work dispatch thread pool, one std::thread backend for every platform (the
// old code kept separate win32 and pthread implementations) a stage splits its
// work into independent indices and each worker pulls the next one, so the pool
// stays busy without per item scheduling

namespace threads
{
    // worker thread count 0 (the default) means one per hardware thread
    void set_count(int n);
    int count();

    // call work_fn(index) for every index in [0, work_count) indices are handed
    // out from a shared atomic counter, so ordering across threads is not
    // guaranteed; each index must be independent of the others
    void run(int work_count, const std::function<void(int)> &work_fn);

    // like run(), but shows a live progress bar (progressh) named `name` under
    // the group header `section` (printed once), advancing as items complete
    // pass section = nullptr to keep the current section
    void run_phase(const char *section, const char *name, int work_count,
                   const std::function<void(int)> &work_fn);
}
