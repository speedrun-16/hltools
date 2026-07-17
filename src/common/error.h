#pragma once

// fatal error reporting for a compile stage a stage that hits an unrecoverable
// condition reports it and stops; it must never continue and emit a corrupt bsp

namespace err
{
    // report a fatal error to the console and log, then exit the process with a
    // non zero code never returns
    [[noreturn]] void fatal(const char *fmt, ...);

    // fatal unless the condition holds use for internal invariants and input
    // preconditions that would otherwise corrupt output
    void require(bool condition, const char *fmt, ...);
}
