/**
 * @file common.h
 * @author Zmmfly
 * @brief Shared definitions of the ezctrl control library
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026, License under GPLv3
 *
 * Every ezctrl module includes this header, either directly or through the
 * module header that declares the API in use.
 */
#ifndef __EZCTRL_COMMON_H__
#define __EZCTRL_COMMON_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Status returned by ezctrl entry points that can reject arguments.
 *
 * The value is negative on failure, so callers may test either for
 * @ref EZC_OK or for a negative result.
 */
typedef enum ezc_status_e {
    EZC_OK = 0,               /**< The requested operation completed. */
    EZC_ERR_INVALID_ARG = -1, /**< A pointer was NULL or a value was out of range. */
} ezc_status_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __EZCTRL_COMMON_H__ */
