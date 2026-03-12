/**
 ******************************************************************************
 * @file    fdcan_driver.h / fdcan_driver.c
 * @brief   VCI Gateway — STM32H7 FDCAN driver (FDCAN1 HS + FDCAN2 FD)
 ******************************************************************************
 *  STM32H723 FDCAN peripheral specifics:
 *   - Message RAM shared between FDCAN1 and FDCAN2
 *   - FDCAN clock from PLL2Q → must be configured in CubeMX
 *   - FDCAN1: Classical CAN  up to 1 Mbit/s  (TJA1044)
 *   - FDCAN2: CAN-FD         up to 8 Mbit/s  (TCAN4550)
 *   - Uses HAL_FDCAN_* API from STM32H7 HAL
 *
 *  IRQ → ISR → puts frame into RX queue (never blocks in IRQ context)
 */

#ifndef FDCAN_DRIVER_H
#define FDCAN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "bus_manager.h"
#include "stm32h7xx_hal.h"

/* ── FDCAN bitrate presets ────────────────────────────────────────────── */
typedef enum {
    FDCAN_BR_125K  = 0,
    FDCAN_BR_250K,
    FDCAN_BR_500K,
    FDCAN_BR_1M,
    /* FD data phase */
    FDCAN_BR_FD_2M,
    FDCAN_BR_FD_4M,
    FDCAN_BR_FD_5M,
    FDCAN_BR_FD_8M
} FDCAN_BitRatePreset_t;

/* ── Init config ──────────────────────────────────────────────────────── */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;        /* HAL handle (from CubeMX)       */
    VCI_BusID_t          bus_id;
    uint32_t             nominal_bps;   /* Arbitration phase              */
    uint32_t             data_bps;      /* Data phase (FD only)           */
    bool                 fd_mode;       /* Enable CAN-FD                  */
    bool                 brs_enable;    /* Bit rate switching             */
    bool                 tx_pause;      /* ISO 11898-1 TX pause           */
    osMessageQueueId_t  *rx_queue;      /* Shared RX queue reference      */
} FDCAN_DriverCfg_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t    FDCAN_DriverInit      (const FDCAN_DriverCfg_t *cfg);
VCI_BusDriver_t *FDCAN1_GetDriver     (void);
VCI_BusDriver_t *FDCAN2_GetDriver     (void);

/**
 * @brief  Fill HAL nominal timing fields for a given bitrate.
 *         Used by bus_probe.c to reconfigure in listen-only mode.
 *         FDCAN clock assumed 80 MHz (PLL2Q). Adjust table if different.
 */
VCI_Status_t FDCAN_FillNominalTiming (uint32_t bps, FDCAN_InitTypeDef *init);

/**
 * @brief  Fill HAL data-phase timing fields (CAN-FD only).
 */
VCI_Status_t FDCAN_FillDataTiming    (uint32_t bps, FDCAN_InitTypeDef *init);

/* Called from HAL_FDCAN_RxFifo0Callback / HAL_FDCAN_RxFifo1Callback     */
void FDCAN_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo,
                      uint32_t fill_level);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_DRIVER_H */
