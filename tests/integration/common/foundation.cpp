// compile and smoke check for the foundation layer: instantiates the templated
// math at both precisions and exercises the core operations this is scaffolding
// verification, not a lighting regression test (that is compile_diffpy)

#include <atomic>
#include <cstdio>

#include "common/cmdline.h"
#include "common/limits.h"
#include "common/log.h"
#include "common/string_util.h"
#include "common/threads.h"
#include "math/bounding_box.h"
#include "math/plane.h"
#include "math/vector.h"

template <typename T>
static int check_precision(const char *label)
{
    using v3 = math::vec3<T>;

    v3 a{ 1, 2, 2 };
    v3 b{ 4, 0, 0 };
    float raw[4] = { 1, 3, 5, 7 };

    T d = math::dot(a, b);
    T raw_dot = math::dot(a, raw);
    T raw_dot_reverse = math::dot(raw, a);
    v3 c = math::cross(a, b);
    v3 sum = a + b;
    float copied[4] = {};
    math::copy_to_float(sum, copied);

    v3 n = a;
    T len = math::normalize(n); // a has length 3 exactly

    math::basic_bounding_box<T> box;
    box.add(a);
    box.add(b);

    math::basic_plane<T> pl;
    pl.normal = v3{ 0, 0, 1 };
    pl.dist = 5;
    auto pt = math::type_for_normal(pl.normal);

    std::printf("%-7s dot=%.1f rawdot=%.1f rawrev=%.1f cross=(%.1f,%.1f,%.1f) sum.x=%.1f copy.x=%.1f len=%.4f "
                "box.max.z=%.1f plane_z=%s dist=%.1f\n",
                label, (double)d, (double)raw_dot, (double)raw_dot_reverse, (double)c.x, (double)c.y, (double)c.z,
                (double)sum.x, (double)copied[0], (double)len, (double)box.maxs.z,
                pt == math::plane_type::z ? "yes" : "no",
                (double)pl.distance_to(v3{ 0, 0, 8 }));
    return 0;
}

int main()
{
    std::printf("max_alloc_block_pages = %d, texture_step = %d\n",
                limits::max_alloc_block_pages, limits::texture_step);
    check_precision<float>("float");
    check_precision<double>("double");

    // common layer: string helpers, warning collection, footer state
    char buf[32];
    std::printf("commas: %s\n", str::with_commas(1377780, buf, sizeof(buf)));
    logging::warn("example recoverable oddity");
    logging::add_warning_summary("atlas approaching the limit");
    std::printf("warning_count = %u\n", logging::warning_count());

    // threads: sum 09999 across the pool, each index touched exactly once
    threads::set_count(4);
    std::atomic<long long> total{ 0 };
    threads::run(10000, [&](int i) { total.fetch_add(i, std::memory_order_relaxed); });
    std::printf("threads=%d parallel_sum=%lld (expect 49995000)\n",
                threads::count(), total.load());
    return 0;
}
