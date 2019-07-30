/*******************************************************************************
 * tlx/backtrace.cpp
 *
 * Part of tlx - http://panthema.net/tlx
 *
 * Copyright (C) 2008-2017 Timo Bingmann <tb@panthema.net>
 *
 * All rights reserved. Published under the Boost Software License, Version 1.0
 ******************************************************************************/

#include <tlx/backtrace.hpp>

#include <tlx/unused.hpp>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#if __linux__

#include <cxxabi.h>
#include <execinfo.h>

#endif

namespace tlx {

void print_raw_backtrace(FILE* out, unsigned int max_frames,
                         const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    vfprintf(out, fmt, args);

#if __linux__

    // storage array for stack trace address data
    void** addrlist = reinterpret_cast<void**>(
        alloca(sizeof(void*) * max_frames));

    // retrieve current stack addresses
    int addrlen = backtrace(addrlist, max_frames);

    for (int i = 1; i < addrlen; ++i) {
        if (addrlist[i] == nullptr)
            break;

        fprintf(out, " %p", addrlist[i]);
    }

    fprintf(out, "\n");

#else

    fprintf(out, "(backtrace not supported on this platform)\n");
    tlx::unused(max_frames);

#endif

    va_end(args);
}

void print_raw_backtrace(FILE* out, unsigned int max_frames) {
    return print_raw_backtrace(out, max_frames, "backtrace:");
}

void print_cxx_backtrace(FILE* out, unsigned int max_frames) {
    fprintf(out, "backtrace:\n");

#if __linux__

    // storage array for stack trace address data
    void** addrlist = reinterpret_cast<void**>(
        alloca(sizeof(void*) * max_frames));

    // retrieve current stack addresses
    int addrlen = backtrace(addrlist, max_frames);

    if (addrlen == 0) {
        fprintf(out, "  <empty, possibly corrupt>\n");
        return;
    }

    // resolve addresses into strings containing "filename(function+address)",
    // this array must be free()-ed
    char** symbollist = backtrace_symbols(addrlist, addrlen);

    // allocate string which will be filled with the demangled function name
    size_t funcnamesize = 256;
    char* funcname = (char*)malloc(funcnamesize);

    // iterate over the returned symbol lines. skip the first, it is the
    // address of this function.
    for (int i = 1; i < addrlen; i++)
    {
        char* begin_name = 0, * begin_offset = 0, * end_offset = 0;

        // find parentheses and +address offset surrounding the mangled name:
        // ./module(function+0x15c) [0x8048a6d]
        for (char* p = symbollist[i]; *p; ++p)
        {
            if (*p == '(')
                begin_name = p;
            else if (*p == '+')
                begin_offset = p;
            else if (*p == ')' && begin_offset) {
                end_offset = p;
                break;
            }
        }

        if (begin_name && begin_offset && end_offset
            && begin_name < begin_offset)
        {
            *begin_name++ = '\0';
            *begin_offset++ = '\0';
            *end_offset = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():

            int status;
            char* ret = abi::__cxa_demangle(begin_name,
                                            funcname, &funcnamesize, &status);
            if (status == 0) {
                funcname = ret; // use possibly realloc()-ed string
                fprintf(out, "  %s : %s+%s\n",
                        symbollist[i], funcname, begin_offset);
            }
            else {
                // demangling failed. Output function name as a C function with
                // no arguments.
                fprintf(out, "  %s : %s()+%s\n",
                        symbollist[i], begin_name, begin_offset);
            }
        }
        else
        {
            // couldn't parse the line? print the whole line.
            fprintf(out, "  %s\n", symbollist[i]);
        }
    }

    free(funcname);
    free(symbollist);

#else

    fprintf(out, " (not supported on this platform)\n");
    tlx::unused(max_frames);

#endif
}

} // namespace tlx

/******************************************************************************/
