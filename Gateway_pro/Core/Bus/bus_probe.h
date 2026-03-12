/*
 * bus_probe.h
 *
 *  Created on: Mar 11, 2026
 *      Author: IT DOCTOR
 */

/* Core/Bus/bus_probe.h */
#ifndef BUS_PROBE_H
#define BUS_PROBE_H

#include "vci_config.h"
#include "fdcan_driver.h"
/* ── Résultat de détection ────────────────────────────────────────────── */
typedef struct {
    bool     detected;
    uint32_t baudrate_nominal;   /* CAN/LIN/KLINE                        */
    uint32_t baudrate_data;      /* CAN-FD data phase (0 si classic)     */
    bool     is_fd;              /* CAN-FD détecté                       */
    uint32_t frame_count;        /* Trames captées pendant la probe      */
    uint32_t error_count;
} BusProbeResult_t;

typedef enum {
    PROBE_STATE_IDLE = 0,
    PROBE_STATE_LISTEN,          /* Mode écoute passive                  */
    PROBE_STATE_ANALYZING,
    PROBE_STATE_DONE,
    PROBE_STATE_NO_ACTIVITY
} BusProbeState_t;

/* ── Config de la sonde ───────────────────────────────────────────────── */
typedef struct {
    VCI_BusID_t      bus;
    uint32_t         listen_ms;      /* Durée écoute par baudrate (défaut 200ms) */
    bool             try_fd;         /* Tenter la détection FD après classic     */
    bool             try_lin;        /* Activer si bus physiquement LIN          */
    bool             try_kline;      /* Activer si bus physiquement K-Line       */
} BusProbeCfg_t;

/* ── Baudrates testés dans l'ordre ───────────────────────────────────── */
static const uint32_t CAN_BAUDRATES[] = {
    1000000, 500000, 250000, 125000, 100000, 83333, 50000, 33333, 20000, 10000
};
static const uint32_t CANFD_DATA_RATES[] = {
    8000000, 5000000, 4000000, 2000000, 1000000
};

/* ── API ──────────────────────────────────────────────────────────────── */
VCI_Status_t    BusProbe_Init       (const BusProbeCfg_t *cfg);
VCI_Status_t    BusProbe_Run        (VCI_BusID_t bus, BusProbeResult_t *result);
VCI_Status_t    BusProbe_RunAll     (BusProbeResult_t results[BUS_ID_MAX]);
void            BusProbe_Task       (void *arg);   /* RTOS task dédiée   */

/* Callback — appelé quand un bus est détecté */
void            BusProbe_OnDetected (VCI_BusID_t bus,
                                     const BusProbeResult_t *result);

#endif /* BUS_PROBE_H */
