/**
 ******************************************************************************
 * @file    logger.h
 * @brief   VCI Gateway — Non-blocking trace logger
 ******************************************************************************
 *  Features:
 *   - Zero-copy in IRQ context (ring buffer + background drain task)
 *   - Levels: TRACE, DEBUG, INFO, WARN, ERROR
 *   - Optional SWO/ITM output (ST-Link real-time trace)
 *   - Bus frame hex-dump with timestamp
 */

#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"

typedef enum {
    LOG_TRACE = 0,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
} LogLevel_t;

#define LOG_MAX_LEN    128U

/* ── Macros ───────────────────────────────────────────────────────────── */
#if VCI_LOG_ENABLE
    #define LOG_T(fmt, ...) Logger_Write(LOG_TRACE, __func__, fmt, ##__VA_ARGS__)
    #define LOG_D(fmt, ...) Logger_Write(LOG_DEBUG, __func__, fmt, ##__VA_ARGS__)
    #define LOG_I(fmt, ...) Logger_Write(LOG_INFO,  __func__, fmt, ##__VA_ARGS__)
    #define LOG_W(fmt, ...) Logger_Write(LOG_WARN,  __func__, fmt, ##__VA_ARGS__)
    #define LOG_E(fmt, ...) Logger_Write(LOG_ERROR, __func__, fmt, ##__VA_ARGS__)
    #define LOG_FRAME(bus, fr) Logger_DumpFrame((bus), (fr))
#else
    #define LOG_T(fmt, ...)
    #define LOG_D(fmt, ...)
    #define LOG_I(fmt, ...)
    #define LOG_W(fmt, ...)
    #define LOG_E(fmt, ...)
    #define LOG_FRAME(bus, fr)
#endif

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t Logger_Init        (LogLevel_t min_level);
void         Logger_Write       (LogLevel_t level, const char *func,
                                  const char *fmt, ...);
void         Logger_DumpFrame   (VCI_BusID_t bus, const VCI_Message_t *msg);
void         Logger_SetLevel    (LogLevel_t level);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════════════════════
 *  Implementation
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef LOGGER_IMPL

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "rtos_tasks.h"

static LogLevel_t s_min_level = LOG_DEBUG;
static const char *const s_level_str[] = { "TRC","DBG","INF","WRN","ERR" };

VCI_Status_t Logger_Init(LogLevel_t min_level)
{
    s_min_level = min_level;
    return VCI_OK;
}

void Logger_SetLevel(LogLevel_t level)
{
    s_min_level = level;
}

void Logger_Write(LogLevel_t level, const char *func,
                   const char *fmt, ...)
{
    if (level < s_min_level) return;

    char buf[LOG_MAX_LEN];
    uint32_t ts = osKernelGetTickCount();
    int n = snprintf(buf, sizeof(buf), "[%6lu][%s][%s] ",
                     ts, s_level_str[level], func ? func : "?");
    if (n < 0 || n >= (int)sizeof(buf)) return;

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, args);
    va_end(args);
    buf[LOG_MAX_LEN - 1] = '\0';

    /* Non-blocking — drop if queue full (logger should never block callers) */
    osMessageQueuePut(g_q_log, buf, 0, 0);

    /* Also output via ITM/SWO */
#ifdef VCI_ITM_ENABLE
    for (char *p = buf; *p; p++) ITM_SendChar(*p);
    ITM_SendChar('\n');
#endif
}

void Logger_DumpFrame(VCI_BusID_t bus, const VCI_Message_t *msg)
{
    if (s_min_level > LOG_TRACE) return;
    char buf[LOG_MAX_LEN];
    int n = 0;

    if (msg->type == MSG_TYPE_CAN) {
        const VCI_CanFrame_t *fr = &msg->payload.can;
        n = snprintf(buf, sizeof(buf),
                     "%s %s %08lX [%2u] ",
                     BusMgr_GetName(bus),
                     (fr->flags & CAN_FLAG_FD) ? "FD" : "CL",
                     fr->id, fr->dlc);
        for (uint8_t i = 0; i < fr->dlc && n < (int)sizeof(buf) - 3; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%02X ", fr->data[i]);
    } else {
        n = snprintf(buf, sizeof(buf), "%s RAW [%u] ",
                     BusMgr_GetName(bus), msg->payload_len);
        for (uint16_t i = 0; i < msg->payload_len && n < (int)sizeof(buf)-3; i++)
            n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                          "%02X ", msg->payload.raw[i]);
    }
    osMessageQueuePut(g_q_log, buf, 0, 0);
}

#endif /* LOGGER_IMPL */
#endif /* LOGGER_H */
