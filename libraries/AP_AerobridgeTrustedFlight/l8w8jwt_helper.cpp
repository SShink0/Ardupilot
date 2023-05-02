#include "l8w8jwt_helper.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <mbedtls/x509.h>
#include "LogStructure.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <sys/time.h>
#else
#include <AP_GPS/AP_GPS.h>
const AP_GPS& gps = AP::gps();
#endif

extern const AP_HAL::HAL& hal;

void log_message(const char *message);  // remove

void log_message(const char *message)
{
    struct log_Message pkt{
        LOG_PACKET_HEADER_INIT(LOG_TRUSTED_FLIGHT_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };

    strncpy_noterm(pkt.msg, message, sizeof(pkt.msg));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void *l8w8jwt_realloc_impl(void *ptr, size_t new_size) {
    return hal.util->std_realloc(ptr, new_size);
}

time_t l8w8jwt_time_impl(time_t *time) {
    if (time != NULL) {
        // only support to fetch current time
        return -1;
    }
    uint64_t ret_time;
    char msg[100];

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    ret_time = current_time.tv_sec;
    hal.util->snprintf(msg, sizeof(msg), "Current time of day for SITL. sec: %ld, usec:%ld", current_time.tv_sec, current_time.tv_usec);
#else
    ret_time = gps.time_epoch_usec() * 1.0e-6;
    hal.util->snprintf(msg, sizeof(msg), "Current GPS time week ms: %lu, unix epoch usec: %lu", (unsigned long int)gps.time_week_ms(), (unsigned long int)ret_time);
#endif

    log_message(msg);

    mbedtls_x509_time now;
    struct tm *lt, tm_buf;
    lt = mbedtls_platform_gmtime_r( (mbedtls_time_t *)&ret_time, &tm_buf );
    if( lt == NULL )
        log_message("lt is NULL");
    else
    {
        now.year = lt->tm_year + 1900;
        now.mon  = lt->tm_mon  + 1;
        now.day  = lt->tm_mday;
        now.hour = lt->tm_hour;
        now.min  = lt->tm_min;
        now.sec  = lt->tm_sec;

        hal.util->snprintf(msg, sizeof(msg), "Current time: %d/%d/%d %d:%d:%d UTC", now.day, now.mon, now.year, now.hour, now.min, now.sec);
        log_message(msg);
    }

    return ret_time;
}
