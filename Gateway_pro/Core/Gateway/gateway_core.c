/**
 ******************************************************************************
 * @file    gateway_core.c
 * @brief   Gateway Core routing engine + all RTOS task bodies
 ******************************************************************************
 *  This file also contains stub task bodies for:
 *   Task_FDCAN1, Task_FDCAN2, Task_LIN, Task_KLine,
 *   Task_ISOTP, Task_Diag, Task_USBHost, Task_ETH_DoIP,
 *   Task_Logger, Task_Power
 *  Each task has a clear structure showing startup → steady-state loop.
 */

#include "gateway_core.h"
#include "rtos_tasks.h"
#include "bus_manager.h"
#include "fdcan_driver.h"
#include "isotp.h"
#include "uds.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* ════════════════════════════════════════════════════════════════════════
 *  Gateway Core Implementation
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t GW_Init(GW_Context_t *ctx)
{
    if (!ctx) return VCI_ERR_PARAM;
    memset(ctx, 0, sizeof(*ctx));
    return VCI_OK;
}

VCI_Status_t GW_AddRule(GW_Context_t *ctx, const GW_RoutingRule_t *rule)
{
    if (!ctx || !rule || ctx->rule_count >= GW_MAX_RULES)
        return VCI_ERR_PARAM;
    ctx->rules[ctx->rule_count++] = *rule;
    return VCI_OK;
}

VCI_Status_t GW_ClearRules(GW_Context_t *ctx)
{
    if (!ctx) return VCI_ERR_PARAM;
    ctx->rule_count = 0;
    return VCI_OK;
}

/* Default rules:
 *  1. All FDCAN1 frames → USB host (passthrough sniffer mode)
 *  2. All FDCAN2 frames → USB host
 *  3. Functional UDS address (7DF) on FDCAN1 → ISO-TP handler
 *  4. Physical UDS responses (7E0-7E7) on FDCAN1 → ISO-TP handler
 *  5. FDCAN1 ↔ FDCAN2 gateway (bridged by default) — can be toggled
 */
VCI_Status_t GW_LoadDefaultRules(GW_Context_t *ctx)
{
    GW_ClearRules(ctx);

    /* Rule 0: FDCAN1 all → USB passthrough */
    GW_AddRule(ctx, &(GW_RoutingRule_t){
        .src_bus       = BUS_ID_FDCAN1,
        .src_id        = 0,
        .src_id_mask   = 0,        /* wildcard */
        .dst_bus       = BUS_ID_USB,
        .forward_to_usb= true,
        .feed_isotp    = false
    });

    /* Rule 1: FDCAN2 all → USB passthrough */
    GW_AddRule(ctx, &(GW_RoutingRule_t){
        .src_bus       = BUS_ID_FDCAN2,
        .src_id        = 0,
        .src_id_mask   = 0,
        .dst_bus       = BUS_ID_USB,
        .forward_to_usb= true,
        .feed_isotp    = false
    });

    /* Rule 2: FDCAN1 UDS functional request → ISO-TP */
    GW_AddRule(ctx, &(GW_RoutingRule_t){
        .src_bus       = BUS_ID_FDCAN1,
        .src_id        = UDS_REQ_ID,
        .src_id_mask   = 0xFFFFFFFF,
        .dst_bus       = BUS_ID_FDCAN1,
        .forward_to_usb= true,
        .feed_isotp    = true
    });

    /* Rule 3: FDCAN1 UDS physical responses → ISO-TP */
    GW_AddRule(ctx, &(GW_RoutingRule_t){
        .src_bus       = BUS_ID_FDCAN1,
        .src_id        = UDS_RESP_ID_BASE,
        .src_id_mask   = 0xFFFFFFF8,   /* 7E0..7E7                      */
        .dst_bus       = BUS_ID_USB,
        .forward_to_usb= true,
        .feed_isotp    = true
    });

    return VCI_OK;
}

/* ── Message dispatch ─────────────────────────────────────────────────── */
VCI_Status_t GW_ProcessMessage(GW_Context_t *ctx, VCI_Message_t *msg)
{
    if (!ctx || !msg) return VCI_ERR_PARAM;

    bool dispatched = false;

    for (uint8_t i = 0; i < ctx->rule_count; i++) {
        const GW_RoutingRule_t *r = &ctx->rules[i];

        /* Bus match */
        if (r->src_bus != BUS_ID_MAX &&
            r->src_bus != msg->src_bus) continue;

        /* ID match (CAN only) */
        if (r->src_id_mask != 0 && msg->type == MSG_TYPE_CAN) {
            if ((msg->payload.can.id & r->src_id_mask) != r->src_id)
                continue;
        }

        /* Optional transform */
        if (r->transform) r->transform(msg);

        /* Forward to destination bus (not USB) */
        if (r->dst_bus < BUS_ID_USB) {
            msg->dst_bus = r->dst_bus;
            BusMgr_Send(r->dst_bus, msg, 10U);
        }

        /* Forward to USB host */
        if (r->forward_to_usb) {
            osMessageQueuePut(g_q_usb_rx, msg, 0, 0);
        }

        /* Feed ISO-TP demultiplexer */
        if (r->feed_isotp) {
            osMessageQueuePut(g_q_isotp_rx, msg, 0, 0);
        }

        dispatched = true;
    }

    if (dispatched) ctx->routed_total++;
    else            ctx->dropped_total++;

    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_Gateway — main routing task
 * ════════════════════════════════════════════════════════════════════════ */
void Task_Gateway(void *arg)
{
    (void)arg;
    static GW_Context_t gw_ctx;
    GW_Init(&gw_ctx);
    GW_LoadDefaultRules(&gw_ctx);

    VCI_Message_t msg;

    /* All incoming queues polled in round-robin */
    osMessageQueueId_t *queues[] = {
        &g_q_fdcan1_rx,
        &g_q_fdcan2_rx,
        &g_q_lin_rx,
        &g_q_kline_rx,
        &g_q_eth_rx
    };
    const uint8_t num_queues = sizeof(queues) / sizeof(queues[0]);

    for (;;) {
        bool any = false;
        for (uint8_t q = 0; q < num_queues; q++) {
            if (osMessageQueueGet(*queues[q], &msg, NULL, 0) == osOK) {
                GW_ProcessMessage(&gw_ctx, &msg);
                any = true;
            }
        }
        if (!any) osDelay(1);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_ISOTP — reassembly pump
 * ════════════════════════════════════════════════════════════════════════ */
void Task_ISOTP(void *arg)
{
    (void)arg;
    static ISOTP_Channel_t ch_fdcan1; /* One channel per ECU in practice */

    /* Example: physical channel FDCAN1, ECU at 0x7E0               */
    ISOTP_ChannelCfg_t cfg = {
        .bus        = BUS_ID_FDCAN1,
        .tx_id      = 0x7DF,
        .rx_id      = 0x7E8,
        .ext_id     = false,
        .fd_mode    = false,
        .stmin_ms   = ISOTP_STMIN_MS,
        .blocksize  = ISOTP_BS
    };
    ISOTP_ChannelInit(&ch_fdcan1, &cfg);

    VCI_Message_t msg;
    for (;;) {
        if (osMessageQueueGet(g_q_isotp_rx, &msg, NULL, 5U) == osOK) {
            if (msg.type == MSG_TYPE_CAN)
                ISOTP_ProcessRxFrame(&ch_fdcan1, &msg.payload.can);
        }
        ISOTP_Tick(&ch_fdcan1, 5U);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_Diag — UDS request processor
 * ════════════════════════════════════════════════════════════════════════ */
void Task_Diag(void *arg)
{
    (void)arg;
    static UDS_Client_t uds_client;

    UDS_ClientInit(&uds_client, BUS_ID_FDCAN1,
                   0x7DF, 0x7E8, false, false);
    uds_client.tp_running = true;

    /* Wait for USB host to be ready before accepting commands           */
    osSemaphoreAcquire(g_sem_usb_ready, osWaitForever);

    VCI_Message_t req_msg;
    for (;;) {
        if (osMessageQueueGet(g_q_diag_req, &req_msg, NULL, 10U) == osOK) {
            /* Decode PC request and execute UDS service */
            UDS_Request_t  uds_req  = {0};
            UDS_Response_t uds_resp = {0};

            /* Parse raw UDS PDU from USB host */
            const uint8_t *pdu = req_msg.payload.raw;
            uds_req.sid      = (UDS_ServiceID_t)pdu[0];
            uds_req.sub_func = (req_msg.payload_len > 1) ? pdu[1] : 0xFF;
            if (req_msg.payload_len > 3) {
                uds_req.data_id  = (uint16_t)((pdu[2] << 8) | pdu[3]);
                uds_req.data_len = req_msg.payload_len - 4;
                memcpy(uds_req.data, &pdu[4], uds_req.data_len);
            }

            UDS_Request(&uds_client, &uds_req, &uds_resp);

            /* Send response back to USB host */
            VCI_Message_t resp_msg = {0};
            resp_msg.type = MSG_TYPE_UDS;
            resp_msg.src_bus = BUS_ID_FDCAN1;
            resp_msg.dst_bus = BUS_ID_USB;
            resp_msg.payload_len = (uint16_t)(uds_resp.data_len + 1);
            resp_msg.payload.raw[0] = uds_resp.positive ?
                                      (uint8_t)(uds_resp.sid + 0x40U) :
                                      UDS_SID_NRC;
            memcpy(&resp_msg.payload.raw[1], uds_resp.data, uds_resp.data_len);
            osMessageQueuePut(g_q_diag_resp, &resp_msg, 0, 10U);
        }
        UDS_Tick(&uds_client, 10U);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_LIN — LIN bus handler (UART + TJA1021)
 * ════════════════════════════════════════════════════════════════════════ */
void Task_LIN(void *arg)
{
    (void)arg;
    /* LIN master schedule table — add your LIN frames here              */
    for (;;) {
        /* TODO: LIN master schedule — send break + PID, receive data   */
        osDelay(10);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_KLine — K-Line ISO 9141 / KWP2000 (UART + L9637)
 * ════════════════════════════════════════════════════════════════════════ */
void Task_KLine(void *arg)
{
    (void)arg;
    /* 5-baud init or fast init sequence, then KWP2000 session           */
    for (;;) {
        osDelay(20);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_USBHost — USB CDC bridge to PC (USB_OTG_HS)
 *  Protocol: simple framing  [SYNC=0xAA][LEN_H][LEN_L][TYPE][PAYLOAD][CRC8]
 * ════════════════════════════════════════════════════════════════════════ */
#define USB_FRAME_SYNC  0xAAU

void Task_USBHost(void *arg)
{
    (void)arg;

    /* Wait for USB stack to enumerate */
    osSemaphoreAcquire(g_sem_usb_ready, osWaitForever);

    VCI_Message_t msg;
    uint8_t       frame_buf[sizeof(VCI_Message_t) + 4];

    for (;;) {
        /* TX: dequeue messages and send to PC */
        while (osMessageQueueGet(g_q_usb_rx, &msg, NULL, 0) == osOK) {
            uint16_t payload_len = msg.payload_len;
            frame_buf[0] = USB_FRAME_SYNC;
            frame_buf[1] = (uint8_t)(payload_len >> 8);
            frame_buf[2] = (uint8_t)(payload_len & 0xFF);
            frame_buf[3] = (uint8_t)msg.type;
            memcpy(&frame_buf[4], msg.payload.raw, payload_len);
            /* CRC8 appended by USB CDC driver */

            /* TODO: call CDC_Transmit_HS(frame_buf, payload_len + 4)    */
        }

        /* RX: read from PC and push to diag request queue              */
        /* TODO: CDC_Receive_HS callback → parse frame → g_q_diag_req   */

        osDelay(1);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_ETH_DoIP — Ethernet DoIP (ISO 13400)
 * ════════════════════════════════════════════════════════════════════════ */
void Task_ETH_DoIP(void *arg)
{
    (void)arg;
    /* Wait for Ethernet link */
    osSemaphoreAcquire(g_sem_eth_link, osWaitForever);

    /* TODO: lwIP socket — bind UDP 13400 (vehicle announce) +
             TCP 13400 (diagnostic)                                       */
    for (;;) {
        osDelay(5);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_Logger — non-blocking frame trace logger
 * ════════════════════════════════════════════════════════════════════════ */
void Task_Logger(void *arg)
{
    (void)arg;
    char log_entry[256];

    for (;;) {
        if (osMessageQueueGet(g_q_log, log_entry, NULL, 50U) == osOK) {
            /* Output over SWO / ITM (ST-Link) or store to SD card      */
            /* printf/ITM_SendChar implementation here                   */
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Task_Power — voltage monitor, sleep/wake management
 * ════════════════════════════════════════════════════════════════════════ */
void Task_Power(void *arg)
{
    (void)arg;
    for (;;) {
        /* Read ADC for VCC_BUS / VBAT monitoring                        */
        /* Trigger BUS_SLEEP if no activity for configurable timeout      */
        uint32_t flags = osEventFlagsWait(g_evf_system,
                                           EVF_POWER_LOW | EVF_POWER_WAKE,
                                           osFlagsWaitAny, 1000U);
        if (flags & EVF_POWER_LOW) {
            /* Put transceivers into low-power mode                      */
            BusMgr_SleepBus(BUS_ID_FDCAN1);
            BusMgr_SleepBus(BUS_ID_FDCAN2);
            BusMgr_SleepBus(BUS_ID_LIN1);
        }
        if (flags & EVF_POWER_WAKE) {
            BusMgr_WakeBus(BUS_ID_FDCAN1);
            BusMgr_WakeBus(BUS_ID_FDCAN2);
            BusMgr_WakeBus(BUS_ID_LIN1);
        }
    }
}
