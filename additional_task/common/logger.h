#ifndef __LOGGER_HEADER__
#define __LOGGER_HEADER__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#define LOGLEVEL_INFO

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#if defined(LOGLEVEL_DEBUG)
#define LOG_DEBUG(fmt, ...) printf("%s: [%s,%s,%d] " fmt "\n", "DEBUG", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)
#endif

#if defined(LOGLEVEL_DEBUG) || defined(LOGLEVEL_INFO)
#define LOG_INFO(fmt, ...) printf("%s: [%s,%s,%d] " fmt "\n", "INFO", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...)
#endif

#if defined(LOGLEVEL_DEBUG) || defined(LOGLEVEL_INFO) || defined(LOGLEVEL_WARN)
#define LOG_WARN(fmt, ...) printf("%s: [%s,%s,%d] " fmt "\n", "WARN", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...)
#endif

#if defined(LOGLEVEL_DEBUG) || defined(LOGLEVEL_INFO) || defined(LOGLEVEL_WARN) || defined(LOGLEVEL_ERROR)
#define LOG_ERROR(fmt, ...) printf("%s: [%s,%s,%d] " fmt "\n", "ERROR", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define LOG_ERROR(fmt, ...)
#endif

#ifdef __cplusplus
}
#endif
