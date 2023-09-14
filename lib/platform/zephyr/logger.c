/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#include <stdint.h>
#include <stdio.h>

#include <time.h>

#include <zephyr/kernel.h>

/* Full log level string  */
static char DEBUG[] = "DEBUG";
static char INFO[] = "INFO";
static char WARNING[] = "WARNING";
static char ERROR[] = "ERROR";

static inline void get_timestamp(char * timestamp)
{
	int64_t up_time_ms = k_uptime_get();

	int64_t millis  = up_time_ms%1000;
	int64_t seconds = (up_time_ms/1000)%60;
	int64_t minutes = ((up_time_ms/1000)/60)%60;
	int64_t hours   = ((up_time_ms/1000)/60)/60;

	sprintf(timestamp,
			"%02d:%02d:%02d.%03d",
			(int32_t)hours,
			(int32_t)minutes,
			(int32_t)seconds,
			(int32_t)millis
	);
}

static inline void print_prefix(char level, char * module)
{
    // Timestamp should always feat to 23 char, but some margins
    char timestamp[50];
    char * full_level;

    switch (level)
    {
        case ('D'):
            full_level = DEBUG;
            break;
        case ('I'):
            full_level = INFO;
            break;
        case ('W'):
            full_level = WARNING;
            break;
        case ('E'):
            full_level = ERROR;
            break;
        default:
            full_level = &level;
    }
    get_timestamp(timestamp);
    printk("%s | [%s] %s:", timestamp, full_level, module);
}

void Platform_LOG(char level, char * module, char * format, va_list args)
{
    print_prefix(level, module);
    vprintf(format, args);
}

void Platform_print_buffer(uint8_t * buffer, int size)
{
    int i;
    for (i = 0; i < size; i++)
    {
        if ((i & 0xF) == 0xF)
            printk("\n");
        printk("%02x ", buffer[i]);
    }
    printk("\n");
}
