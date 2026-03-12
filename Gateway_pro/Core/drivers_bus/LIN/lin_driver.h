/**
 ******************************************************************************
 * @file    lin_driver.h / lin_driver.c
 * @brief   VCI Gateway — LIN bus driver (UART + TJA1021)
 ******************************************************************************
 *  STM32H723 LIN config:
 *   - USART2 (or USART3) in LIN mode (HAL_LIN_*)
 *   - BREAK detection via HAL_LIN_SendBreak()
 *   - TJA1021 controlled via GPIO (EN pin)
 *   - Standard LIN 1.3 / 2.0 / SAE J2602 compatible
 *
 *  Master operation:
 *    1. Send BREAK (13 dominant bits minimum)
 *    2. Send SYNC  (0x55)
 *    3. Send PID   (6-bit ID + 2 parity bits)
 *    4. Send/receive DATA + checksum
 */

#ifndef LIN_DRIVER_H
#define LIN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "bus_manager.h"
#include "stm32h7xx_hal.h"

/* ── LIN checksum type ────────────────────────────────────────────────── */
typedef enum {
    LIN_CHECKSUM_CLASSIC  = 0,   /* LIN 1.x — data bytes only           */
    LIN_CHECKSUM_ENHANCED = 1    /* LIN 2.x — PID + data bytes          */
} LIN_ChecksumType_t;

/* ── LIN frame direction ──────────────────────────────────────────────── */
typedef enum {
    LIN_DIR_TX = 0,
    LIN_DIR_RX
} LIN_Direction_t;

/* ── Schedule table entry ─────────────────────────────────────────────── */
typedef struct {
    uint8_t            id;       /* 6-bit frame ID (0x00..0x3F)         */
    LIN_Direction_t    dir;
    uint8_t            dlc;      /* 1..8 bytes                          */
    LIN_ChecksumType_t cksum;
    uint32_t           period_ms;
    uint8_t            tx_data[LIN_MAX_DATA]; /* Static TX data          */
    void (*rx_cb)(const VCI_LinFrame_t *fr);  /* Called on master RX     */
} LIN_ScheduleEntry_t;

/* ── Init config ──────────────────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef    *huart;
    uint32_t               baudrate;       /* Default 19200 bps          */
    LIN_ScheduleEntry_t   *schedule;       /* Master schedule table      */
    uint8_t                sched_count;
    osMessageQueueId_t    *rx_queue;
    GPIO_TypeDef          *en_gpio_port;   /* TJA1021 EN pin             */
    uint16_t               en_gpio_pin;
} LIN_DriverCfg_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t     LIN_DriverInit (const LIN_DriverCfg_t *cfg);
VCI_BusDriver_t *LIN_GetDriver  (void);
uint8_t          LIN_CalcPID    (uint8_t id);
uint8_t          LIN_CalcChecksum(uint8_t pid, const uint8_t *data,
                                   uint8_t len, LIN_ChecksumType_t type);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════════════════════
 *  Implementation (single-file for simplicity)
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef LIN_DRIVER_IMPL

#include <string.h>

static LIN_DriverCfg_t s_lin_cfg;
static VCI_BusState_t  s_lin_state = BUS_STATE_UNINIT;
static VCI_BusStats_t  s_lin_stats;

/* ── PID parity bits (ISO 17987-2) ───────────────────────────────────── */
uint8_t LIN_CalcPID(uint8_t id)
{
    id &= 0x3FU;
    uint8_t p0 = ((id>>0)^(id>>1)^(id>>2)^(id>>4)) & 1U;
    uint8_t p1 = (~((id>>1)^(id>>3)^(id>>4)^(id>>5))) & 1U;
    return (uint8_t)(id | (p0 << 6) | (p1 << 7));
}

/* ── Checksum ─────────────────────────────────────────────────────────── */
uint8_t LIN_CalcChecksum(uint8_t pid, const uint8_t *data,
                          uint8_t len, LIN_ChecksumType_t type)
{
    uint16_t sum = (type == LIN_CHECKSUM_ENHANCED) ? pid : 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }
    return (uint8_t)(~sum & 0xFF);
}

/* ── Master frame TX ──────────────────────────────────────────────────── */
static VCI_Status_t _lin_send_frame(uint8_t id, const uint8_t *data,
                                     uint8_t len, LIN_ChecksumType_t cksum_type)
{
    uint8_t pid = LIN_CalcPID(id);
    uint8_t cksum = LIN_CalcChecksum(pid, data, len, cksum_type);

    /* Break field */
    if (HAL_LIN_SendBreak(s_lin_cfg.huart) != HAL_OK) return VCI_ERR_HW;
    osDelay(1); /* guard time */

    /* Sync + PID */
    uint8_t header[2] = { 0x55U, pid };
    if (HAL_UART_Transmit(s_lin_cfg.huart, header, 2, 10U) != HAL_OK)
        return VCI_ERR_HW;

    /* Data + checksum */
    uint8_t frame[LIN_MAX_DATA + 1];
    memcpy(frame, data, len);
    frame[len] = cksum;
    if (HAL_UART_Transmit(s_lin_cfg.huart, frame,
                           (uint16_t)(len + 1), 20U) != HAL_OK)
        return VCI_ERR_HW;

    s_lin_stats.tx_frames++;
    return VCI_OK;
}

/* ── vtable ──────────────────────────────────────────────────────────── */
static VCI_Status_t _lin_init(void *hw) {
    (void)hw;
    /* Enable TJA1021 */
    HAL_GPIO_WritePin(s_lin_cfg.en_gpio_port,
                      s_lin_cfg.en_gpio_pin, GPIO_PIN_SET);
    s_lin_state = BUS_STATE_ACTIVE;
    return VCI_OK;
}
static VCI_Status_t _lin_start(void)  { s_lin_state = BUS_STATE_ACTIVE; return VCI_OK; }
static VCI_Status_t _lin_stop(void)   {
    HAL_GPIO_WritePin(s_lin_cfg.en_gpio_port, s_lin_cfg.en_gpio_pin, GPIO_PIN_RESET);
    s_lin_state = BUS_STATE_IDLE; return VCI_OK;
}
static VCI_Status_t _lin_send(const VCI_Message_t *msg, uint32_t timeout_ms) {
    (void)timeout_ms;
    const VCI_LinFrame_t *fr = &msg->payload.lin;
    return _lin_send_frame(fr->pid & 0x3F, fr->data, fr->len,
        fr->enhanced_checksum ? LIN_CHECKSUM_ENHANCED : LIN_CHECKSUM_CLASSIC);
}
static VCI_Status_t _lin_recv(VCI_Message_t *msg, uint32_t timeout_ms) {
    (void)msg; (void)timeout_ms; return VCI_ERR_UNINIT;
}
static VCI_BusState_t _lin_state_fn(void) { return s_lin_state; }
static VCI_Status_t   _lin_stats(VCI_BusStats_t *o) { if(o) *o = s_lin_stats; return VCI_OK; }
static VCI_Status_t   _lin_clr(void) { memset(&s_lin_stats,0,sizeof(s_lin_stats)); return VCI_OK; }

static VCI_BusDriver_t s_lin_drv = {
    .init=_lin_init, .start=_lin_start, .stop=_lin_stop,
    .send=_lin_send, .recv=_lin_recv,
    .get_state=_lin_state_fn, .get_stats=_lin_stats, .clear_stats=_lin_clr
};

VCI_Status_t LIN_DriverInit(const LIN_DriverCfg_t *cfg) {
    if (!cfg) return VCI_ERR_PARAM;
    s_lin_cfg = *cfg;
    return _lin_init(cfg->huart);
}
VCI_BusDriver_t *LIN_GetDriver(void) { return &s_lin_drv; }

#endif /* LIN_DRIVER_IMPL */
#endif /* LIN_DRIVER_H */
