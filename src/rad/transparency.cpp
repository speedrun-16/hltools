#include <climits>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "internal.h"

// transparency and style arrays for the sparse and vismatrix methods: raw
// patch pair entries collected under the lock during the visibility build,
// then mirrored, sorted and searched with a per thread resume cursor

namespace rad
{
    namespace
    {
        // the value list is deduplicated by fuzzy compare, so insertion order
        // decides which representative survives; entry 0 is always full white
        unsigned add_transparency_to_data_list(rad_state &state, const vec3v &trans)
        {
            for (unsigned int i = 0; i < state.trans_list.size(); i++)
            {
                if (math::equal(trans, state.trans_list[i]))
                {
                    return i;
                }
            }

            if (state.trans_list.size() >= (unsigned int)INT_MAX)
            {
                err::fatal("add_transparency_to_data_list: array size exceeded INT_MAX");
            }
            if (state.trans_list.empty())
            {
                state.trans_list.push_back(vec3_one);
            }

            state.trans_list.push_back(trans);

            return (unsigned)(state.trans_list.size() - 1);
        }

        int sort_list(const void *a, const void *b)
        {
            const trans_pair *item1 = (const trans_pair *)a;
            const trans_pair *item2 = (const trans_pair *)b;

            if (item1->p1 == item2->p1)
            {
                return (int)(item1->p2 - item2->p2);
            }
            else
            {
                return (int)(item1->p1 - item2->p1);
            }
        }

        int sort_style_list(const void *a, const void *b)
        {
            const style_pair *item1 = (const style_pair *)a;
            const style_pair *item2 = (const style_pair *)b;

            if (item1->p1 == item2->p1)
            {
                return (int)(item1->p2 - item2->p2);
            }
            else
            {
                return (int)(item1->p1 - item2->p1);
            }
        }

        void log_array_size(const char *print_name, size_t size)
        {
            if (size > 1024 * 1024)
                logging::info("%-20s: %5.1f megs \n", print_name, (double)size / (1024.0 * 1024.0));
            else if (size > 1024)
                logging::info("%-20s: %5.1f kilos\n", print_name, (double)size / 1024.0);
            else
                logging::info("%-20s: %5.1f bytes\n", print_name, (double)size);
        }
    }

    void add_transparency_to_raw_array(rad_state &state, const unsigned p1, const unsigned p2, const vec3v &trans)
    {
        std::lock_guard<std::mutex> guard(state.lock);

        unsigned data_index = add_transparency_to_data_list(state, trans);

        if (state.trans_raw.size() >= (unsigned int)INT_MAX)
        {
            err::fatal("add_transparency_to_raw_array: array size exceeded INT_MAX");
        }

        trans_pair entry;
        entry.p1 = p1;
        entry.p2 = p2;
        entry.data_index = data_index;
        state.trans_raw.push_back(entry);
    }

    void create_final_transparency_arrays(rad_state &state, const char *print_name)
    {
        if (state.trans_raw.empty())
        {
            return;
        }

        // double sized: both orderings of each pair, so the sorted search
        // never needs to swap
        size_t raw_count = state.trans_raw.size();
        state.trans_sorted.resize(raw_count * 2);

        // first half has p1 > p2
        for (size_t i = 0; i < raw_count; i++)
        {
            state.trans_sorted[i].p1 = state.trans_raw[i].p2;
            state.trans_sorted[i].p2 = state.trans_raw[i].p1;
            state.trans_sorted[i].data_index = state.trans_raw[i].data_index;
        }
        // second half has p1 < p2
        std::memcpy(&state.trans_sorted[raw_count], state.trans_raw.data(), sizeof(trans_pair) * raw_count);

        state.trans_raw.clear();
        state.trans_raw.shrink_to_fit();

        std::qsort(state.trans_sorted.data(), state.trans_sorted.size(), sizeof(trans_pair), sort_list);

        size_t size = state.trans_sorted.size() * sizeof(trans_pair) + state.trans_list.capacity() * sizeof(vec3v);
        log_array_size(print_name, size);
    }

    void free_transparency_arrays(rad_state &state)
    {
        state.trans_sorted.clear();
        state.trans_sorted.shrink_to_fit();
        state.trans_list.clear();
        state.trans_list.shrink_to_fit();
    }

    // find transparency from the sorted list; next_index remembers the last
    // location so monotone queries resume instead of rescanning
    void get_transparency(const rad_state &state, const unsigned p1, const unsigned p2,
                          vec3v &trans, unsigned int &next_index)
    {
        trans = vec3_one;

        for (unsigned i = next_index; i < state.trans_sorted.size(); i++)
        {
            const trans_pair &entry = state.trans_sorted[i];
            if (entry.p1 < p1)
            {
                continue;
            }
            else if (entry.p1 == p1)
            {
                if (entry.p2 < p2)
                {
                    continue;
                }
                else if (entry.p2 == p2)
                {
                    math::copy(state.trans_list[entry.data_index], trans);
                    next_index = i + 1;

                    return;
                }
                else // entryp2 > p2
                {
                    next_index = i;

                    return;
                }
            }
            else // entryp1 > p1
            {
                next_index = i;

                return;
            }
        }

        next_index = (unsigned int)state.trans_sorted.size();
    }

    void add_style_to_style_array(rad_state &state, const unsigned p1, const unsigned p2, const int style)
    {
        if (style == -1)
            return;

        std::lock_guard<std::mutex> guard(state.lock);

        if (state.style_list.size() >= (unsigned int)INT_MAX)
        {
            err::fatal("add_style_to_style_array: array size exceeded INT_MAX");
        }

        style_pair entry;
        entry.p1 = p1;
        entry.p2 = p2;
        entry.style = (char)style;
        state.style_list.push_back(entry);
    }

    void create_final_style_arrays(rad_state &state, const char *print_name)
    {
        if (state.style_list.empty())
        {
            return;
        }

        std::qsort(state.style_list.data(), state.style_list.size(), sizeof(style_pair), sort_style_list);

        size_t size = state.style_list.capacity() * sizeof(style_pair);
        log_array_size(print_name, size);
    }

    void free_style_arrays(rad_state &state)
    {
        state.style_list.clear();
        state.style_list.shrink_to_fit();
    }

    void get_style(const rad_state &state, const unsigned p1, const unsigned p2,
                   int &style, unsigned int &next_index)
    {
        style = -1;

        for (unsigned i = next_index; i < state.style_list.size(); i++)
        {
            const style_pair &entry = state.style_list[i];
            if (entry.p1 < p1)
            {
                continue;
            }
            else if (entry.p1 == p1)
            {
                if (entry.p2 < p2)
                {
                    continue;
                }
                else if (entry.p2 == p2)
                {
                    style = (int)entry.style;
                    next_index = i + 1;

                    return;
                }
                else // entryp2 > p2
                {
                    next_index = i;

                    return;
                }
            }
            else // entryp1 > p1
            {
                next_index = i;

                return;
            }
        }

        next_index = (unsigned int)state.style_list.size();
    }
}
