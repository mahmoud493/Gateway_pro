/**
 ******************************************************************************
 * @file    fdcan_tasks.c
 * @brief   VCI Gateway — Task_FDCAN1 / Task_FDCAN2 RTOS task bodies
 ******************************************************************************
 *  Ce fichier est séparé de fdcan_driver.c pour casser la dépendance
 *  circulaire :
 *
 *    fdcan_driver.h  ←  bus_probe.h  (probe a besoin des timings FDCAN)
 *    bus_probe.h     →  fdcan_driver.h  ← déjà inclus → BusProbeResult_t
 *                                          non encore visible si inclus
 *                                          depuis fdcan_driver.c
 *
 *  Ici on inclut d'abord bus_probe.h (qui tire fdcan_driver.h dedans),
 *  puis on utilise les deux APIs sans cycle.
 *
 *  Graphe d'inclusion propre :
 *    fdcan_tasks.c
 *      ├── bus_probe.h      → fdcan_driver.h → vci_config.h
 *      ├── bus_manager.h
 *      └── rtos_tasks.h
 ******************************************************************************
 */

/* bus_probe.h inclut fdcan_driver.h — donc il doit être PREMIER           */
#include "bus_probe.h"
#include "bus_manager.h"
#include "rtos_tasks.h"
#include <string.h>

/* ── Résultats de probe — partagés entre les deux tâches ─────────────── */
static BusProbeResult_t s_result_fdcan1;
static BusProbeResult_t s_result_fdcan2;

/* ── Helper : appliquer un résultat de probe à un handle FDCAN ────────── */
static void _apply_probe_and_start(FDCAN_HandleTypeDef  *hfdcan,
                                    VCI_BusID_t           bus_id,
                                    BusProbeResult_t     *res,
                                    osMutexId_t          *mtx,
                                    osMessageQueueId_t   *rxq,
                                    VCI_BusDriver_t *(*get_drv)(void))
{
    /* Fallback si non détecté */
    if (!res->detected) {
        res->baudrate_nominal = (bus_id == BUS_ID_FDCAN1) ? 500000UL : 500000UL;
        res->baudrate_data    = (bus_id == BUS_ID_FDCAN1) ? 500000UL : 2000000UL;
        res->is_fd            = (bus_id == BUS_ID_FDCAN2);
    }

    FDCAN_DriverCfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.hfdcan      = hfdcan;
    cfg.bus_id      = bus_id;
    cfg.nominal_bps = res->baudrate_nominal;
    cfg.data_bps    = res->baudrate_data;
    cfg.fd_mode     = res->is_fd;
    cfg.brs_enable  = res->is_fd;
    cfg.tx_pause    = true;
    cfg.rx_queue    = rxq;

    FDCAN_DriverInit(&cfg);
    BusMgr_RegisterDriver(bus_id, get_drv(), mtx, rxq);
    BusMgr_StartBus(bus_id);
    HAL_FDCAN_Start(hfdcan);
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_FDCAN1 — HS-CAN (TJA1044)
 * ════════════════════════════════════════════════════════════════════════ */
void Task_FDCAN1(void *arg)
{
    (void)arg;
    extern FDCAN_HandleTypeDef hfdcan1;

    /* Attendre fin de BusProbe_Task (one-shot) */
    osEventFlagsWait(g_evf_system,
                     EVF_PROBE_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    /* Démarrer avec le baudrate détecté */
    _apply_probe_and_start(&hfdcan1, BUS_ID_FDCAN1, &s_result_fdcan1,
                            &g_mtx_fdcan1, &g_q_fdcan1_rx,
                            FDCAN1_GetDriver);

    /* ── Surveillance bus-off — bloque sans consommer CPU ────────────── */
    for (;;) {
        osEventFlagsWait(g_evf_system,
                         EVF_BUS_OFF_FDCAN1,
                         osFlagsWaitAny,
                         osWaitForever);

        osDelay(500U);  /* stabilisation */

        HAL_FDCAN_Stop(&hfdcan1);
        BusProbe_Run(BUS_ID_FDCAN1, &s_result_fdcan1);
        _apply_probe_and_start(&hfdcan1, BUS_ID_FDCAN1, &s_result_fdcan1,
                                &g_mtx_fdcan1, &g_q_fdcan1_rx,
                                FDCAN1_GetDriver);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_FDCAN2 — FD-CAN (TCAN4550)
 * ════════════════════════════════════════════════════════════════════════ */
void Task_FDCAN2(void *arg)
{
    (void)arg;
    extern FDCAN_HandleTypeDef hfdcan2;

    osEventFlagsWait(g_evf_system,
                     EVF_PROBE_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    _apply_probe_and_start(&hfdcan2, BUS_ID_FDCAN2, &s_result_fdcan2,
                            &g_mtx_fdcan2, &g_q_fdcan2_rx,
                            FDCAN2_GetDriver);

    for (;;) {
        osEventFlagsWait(g_evf_system,
                         EVF_BUS_OFF_FDCAN2,
                         osFlagsWaitAny,
                         osWaitForever);

        osDelay(500U);

        HAL_FDCAN_Stop(&hfdcan2);
        BusProbe_Run(BUS_ID_FDCAN2, &s_result_fdcan2);
        _apply_probe_and_start(&hfdcan2, BUS_ID_FDCAN2, &s_result_fdcan2,
                                &g_mtx_fdcan2, &g_q_fdcan2_rx,
                                FDCAN2_GetDriver);
    }
}
