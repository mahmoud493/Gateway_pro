/**
 ******************************************************************************
 * @file    gateway_core.h / gateway_core.c
 * @brief   VCI Gateway — Routing engine & message dispatch
 ******************************************************************************
 *  The Gateway Core:
 *   1. Reads messages from all bus RX queues (via Bus Manager)
 *   2. Applies configurable routing rules
 *   3. Forwards frames to destination buses and/or to the USB host
 *   4. Feeds ISO-TP / UDS demultiplexer
 *
 *  Routing rule format:
 *    { src_bus, src_id, src_id_mask, dst_bus, transform_fn }
 *
 *  Default rule: ALL frames from all buses → forwarded to USB host (PC)
 */

#ifndef GATEWAY_CORE_H
#define GATEWAY_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "bus_manager.h"

/* ── Routing rule ─────────────────────────────────────────────────────── */
typedef struct {
    VCI_BusID_t  src_bus;
    uint32_t     src_id;
    uint32_t     src_id_mask;   /* 0xFFFFFFFF = exact match, 0 = wildcard*/
    VCI_BusID_t  dst_bus;
    bool         forward_to_usb;
    bool         feed_isotp;
    /* Optional transform: modify frame in-place before forwarding       */
    VCI_Status_t (*transform)(VCI_Message_t *msg);
} GW_RoutingRule_t;

#define GW_MAX_RULES   64U
#define GW_RULE_END    { BUS_ID_MAX, 0, 0, BUS_ID_MAX, false, false, NULL }

/* ── Gateway context ──────────────────────────────────────────────────── */
typedef struct {
    GW_RoutingRule_t  rules[GW_MAX_RULES];
    uint8_t           rule_count;
    uint32_t          routed_total;
    uint32_t          dropped_total;
} GW_Context_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t GW_Init           (GW_Context_t *ctx);
VCI_Status_t GW_AddRule        (GW_Context_t *ctx, const GW_RoutingRule_t *rule);
VCI_Status_t GW_ClearRules     (GW_Context_t *ctx);
VCI_Status_t GW_LoadDefaultRules(GW_Context_t *ctx);
VCI_Status_t GW_ProcessMessage (GW_Context_t *ctx, VCI_Message_t *msg);

/* Task body */
void Task_Gateway(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_CORE_H */
