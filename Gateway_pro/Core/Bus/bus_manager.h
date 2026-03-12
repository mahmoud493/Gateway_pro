/**
 ******************************************************************************
 * @file    bus_manager.h
 * @brief   VCI Gateway — Bus abstraction layer
 ******************************************************************************
 *  The Bus Manager provides a unified API over all physical buses.
 *  Upper layers (Gateway Core, ISO-TP, UDS) use only this API and are
 *  completely independent of the underlying hardware.
 *
 *  Each bus driver registers a VCI_BusDriver_t vtable at init time.
 */

#ifndef BUS_MANAGER_H
#define BUS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "cmsis_os2.h"

/* ── Per-bus statistics ───────────────────────────────────────────────── */
typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_errors;
    uint32_t tx_errors;
    uint32_t bus_off_count;
    uint32_t overflow_count;
} VCI_BusStats_t;

/* ── Bus driver vtable ────────────────────────────────────────────────── */
typedef struct {
    /* Lifecycle */
    VCI_Status_t (*init)  (void *hw_handle);
    VCI_Status_t (*start) (void);
    VCI_Status_t (*stop)  (void);
    VCI_Status_t (*sleep) (void);
    VCI_Status_t (*wake)  (void);

    /* Data path */
    VCI_Status_t (*send)  (const VCI_Message_t *msg, uint32_t timeout_ms);
    VCI_Status_t (*recv)  (VCI_Message_t *msg,       uint32_t timeout_ms);

    /* Configuration */
    VCI_Status_t (*set_bitrate) (uint32_t nominal_bps, uint32_t data_bps);
    VCI_Status_t (*set_filter)  (uint32_t id, uint32_t mask, bool ide);

    /* Diagnostics */
    VCI_BusState_t (*get_state) (void);
    VCI_Status_t   (*get_stats) (VCI_BusStats_t *out);
    VCI_Status_t   (*clear_stats)(void);
} VCI_BusDriver_t;

/* ── Bus descriptor ───────────────────────────────────────────────────── */
typedef struct {
    VCI_BusID_t         id;
    const char         *name;
    VCI_BusState_t      state;
    VCI_BusStats_t      stats;
    VCI_BusDriver_t    *drv;
    osMutexId_t        *tx_mutex;   /* pointer to global mutex           */
    osMessageQueueId_t *rx_queue;   /* pointer to global RX queue        */
} VCI_BusDesc_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t    BusMgr_Init           (void);
VCI_Status_t    BusMgr_RegisterDriver (VCI_BusID_t bus, VCI_BusDriver_t *drv,
                                        osMutexId_t *mtx,
                                        osMessageQueueId_t *rxq);
VCI_Status_t    BusMgr_StartBus       (VCI_BusID_t bus);
VCI_Status_t    BusMgr_StopBus        (VCI_BusID_t bus);
VCI_Status_t    BusMgr_SleepBus       (VCI_BusID_t bus);
VCI_Status_t    BusMgr_WakeBus        (VCI_BusID_t bus);

VCI_Status_t    BusMgr_Send           (VCI_BusID_t bus, const VCI_Message_t *msg,
                                        uint32_t timeout_ms);
VCI_Status_t    BusMgr_Recv           (VCI_BusID_t bus, VCI_Message_t *msg,
                                        uint32_t timeout_ms);

VCI_Status_t    BusMgr_SetBitrate     (VCI_BusID_t bus,
                                        uint32_t nominal_bps, uint32_t data_bps);
VCI_Status_t    BusMgr_SetFilter      (VCI_BusID_t bus,
                                        uint32_t id, uint32_t mask, bool ide);

VCI_BusState_t  BusMgr_GetState       (VCI_BusID_t bus);
VCI_Status_t    BusMgr_GetStats       (VCI_BusID_t bus, VCI_BusStats_t *out);
VCI_Status_t    BusMgr_ClearStats     (VCI_BusID_t bus);
const char     *BusMgr_GetName        (VCI_BusID_t bus);

#ifdef __cplusplus
}
#endif

#endif /* BUS_MANAGER_H */
