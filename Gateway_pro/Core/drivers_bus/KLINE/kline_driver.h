/**
 ******************************************************************************
 * @file    kline_driver.h
 * @brief   VCI Gateway — K-Line ISO 9141-2 / KWP2000 driver (UART + L9637)
 ******************************************************************************
 *  K-Line physical layer:
 *   - Single wire, half-duplex UART (USART3 or UART4 on H723)
 *   - L9637 transceiver — UART TX/RX → K-Line signal
 *   - 5-baud slow init (ISO 9141) OR fast init (KWP2000 / ISO 14230)
 *   - Default baud after init: 10400 bps
 *
 *  Echo handling:
 *   - L9637 echoes every transmitted byte back → must be discarded
 */

#ifndef KLINE_DRIVER_H
#define KLINE_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "bus_manager.h"
#include "stm32h7xx_hal.h"

/* ── Init types ───────────────────────────────────────────────────────── */
typedef enum {
    KLINE_INIT_SLOW  = 0,   /* ISO 9141 5-baud init (25ms address byte) */
    KLINE_INIT_FAST  = 1    /* KWP2000 fast init (25ms LOW + 25ms HIGH) */
} KLine_InitType_t;

/* ── Config ───────────────────────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *k_gpio_port;  /* K-Line TX as GPIO for bit-bang  */
    uint16_t            k_gpio_pin;
    KLine_InitType_t    init_type;
    uint8_t             ecu_address;  /* 0x33 for ISO9141, 0x00 for KWP  */
    osMessageQueueId_t *rx_queue;
} KLine_DriverCfg_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t     KLine_DriverInit  (const KLine_DriverCfg_t *cfg);
VCI_Status_t     KLine_SlowInit    (void);
VCI_Status_t     KLine_FastInit    (void);
VCI_Status_t     KLine_SendRequest (const uint8_t *data, uint16_t len);
VCI_Status_t     KLine_RecvResponse(uint8_t *buf, uint16_t *len,
                                     uint32_t timeout_ms);
VCI_BusDriver_t *KLine_GetDriver   (void);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════════════════════
 *  Implementation
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef KLINE_DRIVER_IMPL

#include <string.h>

static KLine_DriverCfg_t s_kline_cfg;
static VCI_BusState_t    s_kline_state = BUS_STATE_UNINIT;
static VCI_BusStats_t    s_kline_stats;

/* ── 5-baud bit-bang (K-line LOW/HIGH pulses at 5 bps = 200ms/bit) ──── */
VCI_Status_t KLine_SlowInit(void)
{
    uint8_t addr = s_kline_cfg.ecu_address;
    /* Send address byte at 5 bps using GPIO bit-bang */
    HAL_GPIO_WritePin(s_kline_cfg.k_gpio_port,
                      s_kline_cfg.k_gpio_pin, GPIO_PIN_RESET); /* Start bit */
    osDelay(200);
    for (int bit = 0; bit < 8; bit++) {
        GPIO_PinState level = ((addr >> bit) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(s_kline_cfg.k_gpio_port,
                          s_kline_cfg.k_gpio_pin, level);
        osDelay(200);
    }
    HAL_GPIO_WritePin(s_kline_cfg.k_gpio_port,
                      s_kline_cfg.k_gpio_pin, GPIO_PIN_SET); /* Stop bit */
    osDelay(200);

    /* Reconfigure pin as UART after bit-bang */
    /* (Pin alternate function must be switched — HAL_GPIO_Init needed) */

    /* Wait for ECU synchronisation response (0x55) at 10400 bps */
    uint8_t sync = 0;
    if (HAL_UART_Receive(s_kline_cfg.huart, &sync, 1, 300U) != HAL_OK)
        return VCI_ERR_TIMEOUT;
    if (sync != 0x55U) return VCI_ERR_PROTO;

    /* Read KB1, KB2 */
    uint8_t kb[2];
    if (HAL_UART_Receive(s_kline_cfg.huart, kb, 2, 100U) != HAL_OK)
        return VCI_ERR_TIMEOUT;

    /* Acknowledge with inverted KB2 */
    uint8_t ack = ~kb[1];
    osDelay(40);
    if (HAL_UART_Transmit(s_kline_cfg.huart, &ack, 1, 10U) != HAL_OK)
        return VCI_ERR_HW;

    /* Discard echo */
    uint8_t echo;
    HAL_UART_Receive(s_kline_cfg.huart, &echo, 1, 20U);

    /* Read inverted address byte from ECU */
    uint8_t inv_addr;
    if (HAL_UART_Receive(s_kline_cfg.huart, &inv_addr, 1, 200U) != HAL_OK)
        return VCI_ERR_TIMEOUT;
    if (inv_addr != (uint8_t)(~addr)) return VCI_ERR_PROTO;

    s_kline_state = BUS_STATE_ACTIVE;
    return VCI_OK;
}

/* ── Fast init (KWP2000 ISO 14230-2) ─────────────────────────────────── */
VCI_Status_t KLine_FastInit(void)
{
    /* 25ms LOW + 25ms HIGH on K-line */
    HAL_GPIO_WritePin(s_kline_cfg.k_gpio_port,
                      s_kline_cfg.k_gpio_pin, GPIO_PIN_RESET);
    osDelay(25);
    HAL_GPIO_WritePin(s_kline_cfg.k_gpio_port,
                      s_kline_cfg.k_gpio_pin, GPIO_PIN_SET);
    osDelay(25);

    /* Send StartCommunication request: 0xC1 0x33 0xF1 0x81 0x66       */
    uint8_t start_req[] = { 0xC1, 0x33, 0xF1, 0x81, 0x66 };
    if (HAL_UART_Transmit(s_kline_cfg.huart,
                          start_req, sizeof(start_req), 50U) != HAL_OK)
        return VCI_ERR_HW;

    /* Discard echo */
    uint8_t echo[sizeof(start_req)];
    HAL_UART_Receive(s_kline_cfg.huart, echo, sizeof(echo), 50U);

    /* Read StartCommunication positive response */
    uint8_t resp[10];
    if (HAL_UART_Receive(s_kline_cfg.huart, resp, 6, 300U) != HAL_OK)
        return VCI_ERR_TIMEOUT;

    s_kline_state = BUS_STATE_ACTIVE;
    return VCI_OK;
}

/* ── Send / Receive ───────────────────────────────────────────────────── */
VCI_Status_t KLine_SendRequest(const uint8_t *data, uint16_t len)
{
    if (HAL_UART_Transmit(s_kline_cfg.huart,
                          (uint8_t *)data, len, 200U) != HAL_OK)
        return VCI_ERR_HW;

    /* Discard echo bytes (L9637 loopback) */
    uint8_t echo[KLINE_MAX_DATA];
    HAL_UART_Receive(s_kline_cfg.huart, echo, len, 100U);
    s_kline_stats.tx_frames++;
    return VCI_OK;
}

VCI_Status_t KLine_RecvResponse(uint8_t *buf, uint16_t *len,
                                 uint32_t timeout_ms)
{
    /* Read first byte to get length */
    if (HAL_UART_Receive(s_kline_cfg.huart, buf, 1,
                         (uint16_t)timeout_ms) != HAL_OK)
        return VCI_ERR_TIMEOUT;

    uint8_t msg_len = buf[0] & 0x3FU;   /* ISO 14230 format byte        */
    if (msg_len == 0) {
        /* Length in third byte */
        if (HAL_UART_Receive(s_kline_cfg.huart,
                             &buf[1], 2, 50U) != HAL_OK) return VCI_ERR_HW;
        msg_len = buf[2];
    }

    if (HAL_UART_Receive(s_kline_cfg.huart,
                         &buf[1], msg_len + 1, /* +1 for checksum */
                         (uint16_t)(msg_len * 2 + 20)) != HAL_OK)
        return VCI_ERR_TIMEOUT;

    *len = msg_len + 2;
    s_kline_stats.rx_frames++;
    return VCI_OK;
}

/* ── vtable ──────────────────────────────────────────────────────────── */
static VCI_Status_t _kl_init(void *hw) {
    (void)hw; return KLine_FastInit();
}
static VCI_Status_t _kl_start(void) { s_kline_state = BUS_STATE_ACTIVE; return VCI_OK; }
static VCI_Status_t _kl_stop(void)  { s_kline_state = BUS_STATE_IDLE;   return VCI_OK; }
static VCI_Status_t _kl_send(const VCI_Message_t *m, uint32_t t) {
    (void)t;
    return KLine_SendRequest(m->payload.raw, m->payload_len);
}
static VCI_Status_t _kl_recv(VCI_Message_t *m, uint32_t t) {
    uint16_t l = 0;
    VCI_Status_t st = KLine_RecvResponse(m->payload.raw, &l, t);
    m->payload_len = l;
    return st;
}
static VCI_BusState_t _kl_state(void) { return s_kline_state; }
static VCI_Status_t   _kl_stats(VCI_BusStats_t *o) { if(o) *o=s_kline_stats; return VCI_OK; }
static VCI_Status_t   _kl_clr(void)   { memset(&s_kline_stats,0,sizeof(s_kline_stats)); return VCI_OK; }

static VCI_BusDriver_t s_kline_drv = {
    .init=_kl_init, .start=_kl_start, .stop=_kl_stop,
    .send=_kl_send, .recv=_kl_recv,
    .get_state=_kl_state, .get_stats=_kl_stats, .clear_stats=_kl_clr
};

VCI_Status_t     KLine_DriverInit(const KLine_DriverCfg_t *cfg) {
    if (!cfg) return VCI_ERR_PARAM;
    s_kline_cfg = *cfg;
    return VCI_OK;
}
VCI_BusDriver_t *KLine_GetDriver(void) { return &s_kline_drv; }

#endif /* KLINE_DRIVER_IMPL */
#endif /* KLINE_DRIVER_H */
