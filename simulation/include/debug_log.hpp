#pragma once

#include <fstream>
#include <map>
#include <string>

// ------------------------------------------------------------------
// debug_log
//
// Lightweight file logging for debugging the D* replanner.
//
// dbg("foo.log") returns a shared std::ofstream for output/foo.log.
// The file is truncated the first time it is requested in a process,
// then appended to on every subsequent call, so a whole run's worth
// of output accumulates in one file across many replans / steps.
//
// Usage:  dbg("replan.log") << "hello\n";
// (output/ is created by sim.cpp main() before anything runs)
// ------------------------------------------------------------------
inline std::ofstream& dbg(const std::string& file) {
    static std::map<std::string, std::ofstream> streams;
    std::map<std::string, std::ofstream>::iterator it = streams.find(file);
    if (it == streams.end()) {
        std::ofstream& os = streams[file];
        os.open("output/" + file, std::ios::out | std::ios::trunc);
        return os;
    }
    return it->second;
}
