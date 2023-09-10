#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>
#include "vex.h"

/// @brief possible values for log filtering
enum LogLevel
{
    DEBUG,
    NOTICE,
    WARNING,
    ERROR,
    CRITICAL,
    TIME
};

/// @brief Class to simplify writing to files
class Logger
{
private:
    const std::string filename;
    vex::brain::sdcard sd;
    void write_level(LogLevel l);

public:
    /// @brief  maximum size for a string to be before it's written
    const int MAX_FORMAT_LEN = 512;
    /// @brief  Create a logger that will save to a file
    /// @param filename the file to save to
    explicit Logger(const std::string &filename);

    /// @brief copying not allowed
    Logger(const Logger &l) = delete;
    /// @brief copying not allowed
    Logger &operator=(const Logger &l) = delete;


    /// @brief Write a string to the log
    /// @param s the string to write
    void Log(const std::string &s);

    /// @brief Write a string to the log with a loglevel
    /// @param level the level to write. DEBUG, NOTICE, WARNING, ERROR, CRITICAL, TIME
    /// @param s the string to write
    void Log(LogLevel level, const std::string &s);

    /// @brief Write a string and newline to the log
    /// @param s the string to write
    void Logln(const std::string &s);

    /// @brief Write a string and a newline to the log with a loglevel
    /// @param level the level to write. DEBUG, NOTICE, WARNING, ERROR, CRITICAL, TIME
    /// @param s the string to write
    void Logln(LogLevel level, const std::string &s);

    /// @brief Write a formatted string to the log
    /// @param fmt the format string (like printf)
    /// @param ... the args
    void Logf(const char *fmt, ...);

    /// @brief Write a formatted string to the log with a loglevel
    /// @param level the level to write. DEBUG, NOTICE, WARNING, ERROR, CRITICAL, TIME
    /// @param fmt the format string (like printf)
    /// @param ... the args
    void Logf(LogLevel level, const char *fmt, ...);
};
