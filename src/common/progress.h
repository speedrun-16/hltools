#pragma once

// live progress display for a compile phase the running phase paints an in place
// ascii bar to the console (repainted with a leading carriage return so it
// overwrites itself instead of scrolling) and, when it finishes, writes one plain
// summary line to the logfile (no carriage returns, so the log stays readable)
// phases can be grouped under a section header printed once when the section
// changes console output is thread safe, so worker threads report completion
// directly; it has no bearing on the byte identical bsp output all text is
// lowercase to match the house style

namespace progress
{
    // print a group header (eg "lighting") once repeated calls with the same
    // name are ignored, so several phases can share one section
    void section(const char *name);

    // start a phase with `total` work units and draw the initial 0% bar
    void begin(const char *name, int total);

    // report absolute progress, or add to it add is safe to call from many
    // worker threads at once
    void set(int done);
    void add(int n = 1);

    // freeze the console bar at 100% with the elapsed time, and write the plain
    // completed line to the logfile
    void end();

    // a phase with no meterable progress: one line
    // "  <phase>  <detail>  <secs>s" to both console and logfile (eg the
    // per bounce radiosity gather, which reports its own bounce count)
    void summary(const char *phase, const char *detail, double secs);
}
