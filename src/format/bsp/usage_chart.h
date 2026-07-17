#pragma once

namespace format
{
    struct map_data;

    // print the bsp usage chart: how full each lump is against its engine limit
    // the console shows only the rows worth watching plus a total; the logfile
    // gets the complete table full_console shows every row on the console too
    // (bspinfo) alloc_block_pages is supplied by rad when that stage has run;
    // other stages leave it unavailable this is a report, so it does not
    // affect output
    void print_usage_chart(const map_data &map, bool full_console = false,
                           int alloc_block_pages = -1);
}
