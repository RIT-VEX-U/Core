#include "../core/include/utils/logger.h"
#include <stdarg.h>

void Logger::write_level(LogLevel l)
{
    unsigned char *debug = (unsigned char *)"DEBUG: ";
    unsigned char *notice = (unsigned char *)"NOTICE: ";
    unsigned char *warning = (unsigned char *)"WARNING: ";
    unsigned char *error = (unsigned char *)"ERROR: ";
    unsigned char *critical = (unsigned char *)"CRITICAL: ";
    switch (l)
    {
    case DEBUG:
        sd.appendfile(filename.c_str(), debug, 7);
        break;
    case NOTICE:
        sd.appendfile(filename.c_str(), notice, 8);
        break;
    case WARNING:
        sd.appendfile(filename.c_str(), warning, 9);
        break;
    case ERROR:
        sd.appendfile(filename.c_str(), error, 7);
        break;
    case CRITICAL:
        sd.appendfile(filename.c_str(), critical, 10);
        break;
    case TIME:
        int32_t msec = vexSystemTimeGet();
        Logf("%d: ", msec);
        break;
    };
}

Logger::Logger(const std::string &filename) : filename(filename) {
    sd.savefile(filename.c_str(), NULL, 0);
}

void Logger::Log(const std::string &s)
{
    sd.appendfile(filename.c_str(), (unsigned char *)s.c_str(), s.size());
}
void Logger::Log(LogLevel level, const std::string &s)
{
    write_level(level);
    sd.appendfile(filename.c_str(), (unsigned char *)s.c_str(), s.size());
}

void Logger::Logln(const std::string &s)
{
    sd.appendfile(filename.c_str(), (unsigned char *)s.c_str(), s.size());
    unsigned char newline = '\n';
    sd.appendfile(filename.c_str(), &newline, 1);
}
void Logger::Logln(LogLevel level, const std::string &s)
{
    write_level(level);
    Logln(s);
}

void Logger::Logf(const char *fmt, ...)
{
    char buf[MAX_FORMAT_LEN];
    va_list args;
    va_start(args, fmt);

    int32_t written = vex_vsnprintf(buf, MAX_FORMAT_LEN, fmt, args);

    va_end(args);
    sd.appendfile(filename.c_str(), (unsigned char *)buf, written);

}
void Logger::Logf(LogLevel level, const char *fmt, ...)
{
    char buf[MAX_FORMAT_LEN];

    write_level(level);
    va_list args;
    va_start(args, fmt);
    int32_t written = vex_vsnprintf(buf, MAX_FORMAT_LEN, fmt, args);
    va_end(args);

    sd.appendfile(filename.c_str(), (unsigned char *)buf, written);
}
