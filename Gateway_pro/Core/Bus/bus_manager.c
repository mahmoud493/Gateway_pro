/**
 ******************************************************************************
 * @file    bus_manager.c
 * @brief   VCI Gateway — Bus abstraction layer implementation
 ******************************************************************************
 */

#include "bus_manager.h"
#include <string.h>

/* ── Internal bus table ───────────────────────────────────────────────── */
static VCI_BusDesc_t s_buses[BUS_ID_MAX];
static bool          s_initialized = false;

static const char *const s_bus_names[BUS_ID_MAX] = {
    [BUS_ID_FDCAN1] = "FDCAN1(HS-CAN)",
    [BUS_ID_FDCAN2] = "FDCAN2(FD-CAN)",
    [BUS_ID_LIN1]   = "LIN1",
    [BUS_ID_KLINE]  = "K-Line",
    [BUS_ID_ETH]    = "ETH(DoIP)",
    [BUS_ID_USB]    = "USB-CDC"
};

/* ═════════════════════════════════════════════════════════════════════════
 *  BusMgr_Init
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusMgr_Init(void)
{
    memset(s_buses, 0, sizeof(s_buses));
    for (uint8_t i = 0; i < BUS_ID_MAX; i++) {
        s_buses[i].id    = (VCI_BusID_t)i;
        s_buses[i].name  = s_bus_names[i];
        s_buses[i].state = BUS_STATE_UNINIT;
    }
    s_initialized = true;
    return VCI_OK;
}

/* ═════════════════════════════════════════════════════════════════════════
 *  BusMgr_RegisterDriver
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusMgr_RegisterDriver(VCI_BusID_t bus,
                                    VCI_BusDriver_t    *drv,
                                    osMutexId_t        *mtx,
                                    osMessageQueueId_t *rxq)
{
    if (!s_initialized || bus >= BUS_ID_MAX || drv == NULL) {
        return VCI_ERR_PARAM;
    }
    s_buses[bus].drv      = drv;
    s_buses[bus].tx_mutex = mtx;
    s_buses[bus].rx_queue = rxq;
    s_buses[bus].state    = BUS_STATE_IDLE;
    return VCI_OK;
}

/* ── Internal guard ───────────────────────────────────────────────────── */
static inline VCI_Status_t _check(VCI_BusID_t bus)
{
    if (bus >= BUS_ID_MAX)              return VCI_ERR_PARAM;
    if (s_buses[bus].drv == NULL)       return VCI_ERR_UNINIT;
    return VCI_OK;
}

/* ═════════════════════════════════════════════════════════════════════════
 *  Lifecycle wrappers
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusMgr_StartBus(VCI_BusID_t bus)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->start == NULL) return VCI_OK;
    st = s_buses[bus].drv->start();
    if (st == VCI_OK) s_buses[bus].state = BUS_STATE_ACTIVE;
    return st;
}

VCI_Status_t BusMgr_StopBus(VCI_BusID_t bus)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->stop == NULL) return VCI_OK;
    st = s_buses[bus].drv->stop();
    if (st == VCI_OK) s_buses[bus].state = BUS_STATE_IDLE;
    return st;
}

VCI_Status_t BusMgr_SleepBus(VCI_BusID_t bus)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->sleep == NULL) return VCI_OK;
    st = s_buses[bus].drv->sleep();
    if (st == VCI_OK) s_buses[bus].state = BUS_STATE_SLEEP;
    return st;
}

VCI_Status_t BusMgr_WakeBus(VCI_BusID_t bus)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->wake == NULL) return VCI_OK;
    st = s_buses[bus].drv->wake();
    if (st == VCI_OK) s_buses[bus].state = BUS_STATE_ACTIVE;
    return st;
}

/* ═════════════════════════════════════════════════════════════════════════
 *  Data path
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusMgr_Send(VCI_BusID_t bus, const VCI_Message_t *msg,
                          uint32_t timeout_ms)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK || msg == NULL) return VCI_ERR_PARAM;

    if (s_buses[bus].state != BUS_STATE_ACTIVE) return VCI_ERR_BUSY;

    /* Mutex-protect TX path */
    if (s_buses[bus].tx_mutex != NULL) {
        if (osMutexAcquire(*s_buses[bus].tx_mutex,
                           timeout_ms) != osOK) {
            return VCI_ERR_TIMEOUT;
        }
    }

    st = s_buses[bus].drv->send(msg, timeout_ms);

    if (st == VCI_OK) {
        s_buses[bus].stats.tx_frames++;
    } else {
        s_buses[bus].stats.tx_errors++;
    }

    if (s_buses[bus].tx_mutex != NULL) {
        osMutexRelease(*s_buses[bus].tx_mutex);
    }
    return st;
}

VCI_Status_t BusMgr_Recv(VCI_BusID_t bus, VCI_Message_t *msg,
                          uint32_t timeout_ms)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK || msg == NULL) return VCI_ERR_PARAM;

    /* Try the dedicated RX queue first (populated by ISR/DMA callback)  */
    if (s_buses[bus].rx_queue != NULL) {
        osStatus_t os_st = osMessageQueueGet(*s_buses[bus].rx_queue,
                                              msg,
                                              NULL,
                                              timeout_ms);
        if (os_st == osOK) {
            s_buses[bus].stats.rx_frames++;
            return VCI_OK;
        }
        return (os_st == osErrorTimeout) ? VCI_ERR_TIMEOUT : VCI_ERR_HW;
    }

    /* Fallback: blocking driver recv (LIN / K-Line) */
    st = s_buses[bus].drv->recv(msg, timeout_ms);
    if (st == VCI_OK) s_buses[bus].stats.rx_frames++;
    return st;
}

/* ═════════════════════════════════════════════════════════════════════════
 *  Configuration
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusMgr_SetBitrate(VCI_BusID_t bus,
                                uint32_t nominal_bps, uint32_t data_bps)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->set_bitrate == NULL) return VCI_OK;
    return s_buses[bus].drv->set_bitrate(nominal_bps, data_bps);
}

VCI_Status_t BusMgr_SetFilter(VCI_BusID_t bus,
                               uint32_t id, uint32_t mask, bool ide)
{
    VCI_Status_t st = _check(bus);
    if (st != VCI_OK) return st;
    if (s_buses[bus].drv->set_filter == NULL) return VCI_OK;
    return s_buses[bus].drv->set_filter(id, mask, ide);
}

/* ═════════════════════════════════════════════════════════════════════════
 *  Diagnostics
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_BusState_t BusMgr_GetState(VCI_BusID_t bus)
{
    if (bus >= BUS_ID_MAX) return BUS_STATE_ERROR;
    if (s_buses[bus].drv && s_buses[bus].drv->get_state)
        return s_buses[bus].drv->get_state();
    return s_buses[bus].state;
}

VCI_Status_t BusMgr_GetStats(VCI_BusID_t bus, VCI_BusStats_t *out)
{
    if (_check(bus) != VCI_OK || out == NULL) return VCI_ERR_PARAM;
    *out = s_buses[bus].stats;
    return VCI_OK;
}

VCI_Status_t BusMgr_ClearStats(VCI_BusID_t bus)
{
    if (_check(bus) != VCI_OK) return VCI_ERR_PARAM;
    memset(&s_buses[bus].stats, 0, sizeof(VCI_BusStats_t));
    return VCI_OK;
}

const char *BusMgr_GetName(VCI_BusID_t bus)
{
    if (bus >= BUS_ID_MAX) return "UNKNOWN";
    return s_buses[bus].name;
}
