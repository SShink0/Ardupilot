#pragma once

#include <mbedtls/platform_time.h>
#include <l8w8jwt/timehelper.h>
#include <utils/definitions.h>


#ifdef __cplusplus
extern "C" {
#endif

void *l8w8jwt_realloc_impl(void *ptr, size_t new_size);
time_t l8w8jwt_time_impl(time_t *time);

#ifdef __cplusplus
}
#endif

// defined in trusted_flight_mbedtls_config.h
#if defined(MBEDTLS_PLATFORM_TIME_ALT)
mbedtls_time_t (*mbedtls_time)(mbedtls_time_t *time) = l8w8jwt_time_impl;
#endif

// defined in boards.py
#if L8W8JWT_PLATFORM_TIME_ALT
l8w8jwt_time_t (*l8w8jwt_time)(l8w8jwt_time_t *time) = l8w8jwt_time_impl;
#endif

// defined in boards.py
#if CHILLBUFF_PLATFORM_REALLOC_ALT
void *(*chillbuff_realloc) (void *ptr, size_t size) = l8w8jwt_realloc_impl;
#endif
