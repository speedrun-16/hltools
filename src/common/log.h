#pragma once

#include <string>

// logging and console output for a compile stage two sinks: the console (a
// compact human summary) and the log file (the full classic detail old parsers
// still read) most call sites pick one deliberately
//
// console output is not part of the byte identical contract; only the compiled
// bsp lumps are, so this layer is free to format for readability

namespace logging
{
    // open/close the stage log file console always works even before open
    void open(const char *log_path);
    void close();

    // open the per tool log inside a "logs/" folder next to the map, as
    // "<mapdir>/logs/<mapname><stage>log" (creating logs/ if needed) base is
    // the map path without extension; stage is "csg"/"bsp"/"vis"/"rad"
    void open_stage_log(const std::string &base, const char *stage);

    // both sinks: the normal channel for anything worth seeing live
    void info(const char *fmt, ...);
    // logfile only: full detail that would clutter the console
    void file(const char *fmt, ...);
    // console only: transient lines like the live progress bar
    void console(const char *fmt, ...);

    // a recoverable oddity collected for the end of run recap and counted so
    // the footer can say "(with warnings)"
    void warn(const char *fmt, ...);
    // register a recap entry without printing an inline warning line, for a
    // block (leak, atlas overflow) that prints its own detailed console output
    void add_warning_summary(const char *message);
    unsigned warning_count();
    void print_warnings_summary();

    // footer state a leak and a hard error are their own conditions, not
    // warnings, so the footer can flag "(with leak)" / "(with error)"
    void set_leaked();
    void set_error();
    bool had_error();
    // true once a leak has been reported; the bsp stage uses it to fail the
    // build so a leaking map does not silently run through vis and rad
    bool had_leak();

    // prepare the console for output (utf 8 code page on windows so any
    // non ascii text renders) safe to call once at startup
    void init_console();

    // stage banner and the closing "done " line scope is the stage name
    // ("rad") or "goldsrc map and asset toolchain" for the hltools front page:
    //   hltools 100 - rad  (11 july 2026, 9e94bb0)
    void banner(const char *scope);
    void done(double elapsed_seconds);

    // "settings (changed from default)" table buffer one row per option, then
    // flush: the logfile gets every row under the header; the console gets the
    // header and only the rows whose value differs from the default if nothing
    // changed, the console shows nothing an empty default_desc drops the
    // "(default: )" suffix, for informational rows like the gpu device
    void setting(const char *label, const char *value, const char *default_desc, bool changed);
    void flush_settings();
}
