#include <Arduino.h>
#include <Logger.h>
#include <stdio.h>

#ifdef _DEBUG_
extern char logBuf[100];
    #define LOGGER_VERBOSE_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_VERBOSE(logBuf)
    #define LOGGER_NOTICE_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_NOTICE(logBuf)
    #define LOGGER_WARNING_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_WARNING(logBuf)
    #define LOGGER_ERROR_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_ERROR(logBuf)
    #define LOGGER_FATAL_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_FATAL(logBuf)
    #define LOGGER_SILENT_FMT(fmt,...) sprintf(logBuf,fmt, __VA_ARGS__);LOGGER_SILENT(logBuf)
    #define LOGGER_VERBOSE(msg) Logger::verbose(__PRETTY_FUNCTION__, msg)
    #define LOGGER_NOTICE(msg) Logger::notice(__PRETTY_FUNCTION__, msg)
    #define LOGGER_WARNING(msg) Logger::warning(__PRETTY_FUNCTION__, msg)
    #define LOGGER_ERROR(msg) Logger::error(__PRETTY_FUNCTION__, msg)
    #define LOGGER_FATAL(msg) Logger::fatal(__PRETTY_FUNCTION__, msg)
    #define LOGGER_SILENT(msg) Logger::silent(__PRETTY_FUNCTION__, msg)
#else
    #define LOGGER_VERBOSE_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_NOTICE_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_WARNING_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_ERROR_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_FATAL_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_SILENT_FMT(...) asm volatile ("nop\n\t")
    #define LOGGER_VERBOSE(...) asm volatile ("nop\n\t")
    #define LOGGER_NOTICE(...) asm volatile ("nop\n\t")
    #define LOGGER_WARNING(...) asm volatile ("nop\n\t")
    #define LOGGER_ERROR(...) asm volatile ("nop\n\t")
    #define LOGGER_FATAL(...) asm volatile ("nop\n\t")
    #define LOGGER_SILENT(...) asm volatile ("nop\n\t")
#endif
void localLogger(Logger::Level level, const char* module, const char* message);
