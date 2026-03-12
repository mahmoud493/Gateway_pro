/**
 ******************************************************************************
 * @file    uds.c
 * @brief   UDS (ISO 14229) tester/client implementation
 ******************************************************************************
 */

#include "uds.h"
#include "cmsis_os2.h"
#include <string.h>

/* ── ISO-TP RX callback ───────────────────────────────────────────────── */
static void _isotp_rx_cb(uint8_t *data, uint32_t len, void *ctx)
{
    UDS_Client_t *client = (UDS_Client_t *)ctx;
    if (len > ISOTP_BUFFER_SIZE) len = ISOTP_BUFFER_SIZE;
    memcpy(client->resp_buf, data, len);
    client->resp_len   = len;
    client->resp_ready = true;
}

/* ════════════════════════════════════════════════════════════════════════
 *  UDS_ClientInit
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t UDS_ClientInit(UDS_Client_t *client,
                             VCI_BusID_t bus,
                             uint32_t tx_id, uint32_t rx_id,
                             bool ext_id, bool fd_mode)
{
    if (!client) return VCI_ERR_PARAM;
    memset(client, 0, sizeof(*client));

    ISOTP_ChannelCfg_t cfg = {
        .bus         = bus,
        .tx_id       = tx_id,
        .rx_id       = rx_id,
        .ext_id      = ext_id,
        .addr_mode   = ISOTP_ADDR_NORMAL,
        .fd_mode     = fd_mode,
        .stmin_ms    = ISOTP_STMIN_MS,
        .blocksize   = ISOTP_BS
    };
    VCI_Status_t st = ISOTP_ChannelInit(&client->isotp, &cfg);
    if (st != VCI_OK) return st;

    client->isotp.on_rx_complete  = _isotp_rx_cb;
    client->isotp.user_ctx        = client;
    client->p2_timeout_ms         = UDS_P2_TIMEOUT_MS;
    client->p2ext_timeout_ms      = UDS_P2EXT_TIMEOUT_MS;
    client->session               = UDS_SESSION_DEFAULT;
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  _wait_response  — wait up to timeout_ms for ISOTP reassembly
 * ════════════════════════════════════════════════════════════════════════ */
static VCI_Status_t _wait_response(UDS_Client_t *client,
                                    uint32_t timeout_ms,
                                    UDS_Response_t *resp)
{
    uint32_t deadline = osKernelGetTickCount() + timeout_ms;
    client->resp_ready = false;

    while (!client->resp_ready) {
        if (osKernelGetTickCount() >= deadline)
            return VCI_ERR_TIMEOUT;
        ISOTP_Tick(&client->isotp, 1);
        osDelay(1);
    }

    /* Parse response */
    const uint8_t *buf = client->resp_buf;
    uint32_t       len = client->resp_len;

    if (len < 1) return VCI_ERR_PROTO;

    if (buf[0] == UDS_SID_NRC) {
        /* Negative response: [7F] [SID] [NRC] */
        if (len < 3) return VCI_ERR_PROTO;

        /* Handle RCRRP — response pending */
        if (buf[2] == UDS_NRC_RCRRP) {
            return _wait_response(client, client->p2ext_timeout_ms, resp);
        }
        resp->positive  = false;
        resp->sid       = (UDS_ServiceID_t)buf[1];
        resp->nrc       = (UDS_NRC_t)buf[2];
        resp->data_len  = 0;
        return VCI_OK;
    }

    /* Positive response */
    resp->positive = true;
    resp->sid      = (UDS_ServiceID_t)(buf[0] - 0x40U);
    if (len > 1) {
        resp->data_len = len - 1;
        memcpy(resp->data, &buf[1], resp->data_len);
    }
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  UDS_Request — generic request/response
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t UDS_Request(UDS_Client_t *client,
                          const UDS_Request_t  *req,
                          UDS_Response_t       *resp)
{
    if (!client || !req || !resp) return VCI_ERR_PARAM;

    /* Build request PDU */
    uint8_t  pdu[ISOTP_BUFFER_SIZE];
    uint32_t pdu_len = 0;

    pdu[pdu_len++] = (uint8_t)req->sid;

    /* Sub-function (bit7 = suppress positive response) */
    if (req->sub_func != 0xFF) {
        pdu[pdu_len++] = req->sub_func;
    }

    /* DID (for services that use it) */
    if (req->data_id != 0x0000 &&
        (req->sid == UDS_SID_RDBI || req->sid == UDS_SID_WDBI ||
         req->sid == UDS_SID_IOCBI)) {
        pdu[pdu_len++] = (uint8_t)(req->data_id >> 8);
        pdu[pdu_len++] = (uint8_t)(req->data_id & 0xFF);
    }

    /* Append data payload */
    if (req->data_len > 0 && req->data_len < ISOTP_BUFFER_SIZE - pdu_len) {
        memcpy(&pdu[pdu_len], req->data, req->data_len);
        pdu_len += req->data_len;
    }

    uint32_t t0 = osKernelGetTickCount();
    VCI_Status_t st = ISOTP_Send(&client->isotp, pdu, pdu_len);
    if (st != VCI_OK) return st;

    /* Suppress positive response requested */
    if (req->sub_func != 0xFF && (req->sub_func & 0x80U)) {
        resp->positive = true;
        resp->data_len = 0;
        return VCI_OK;
    }

    st = _wait_response(client, client->p2_timeout_ms, resp);
    resp->elapsed_ms = osKernelGetTickCount() - t0;
    return st;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Convenience wrappers
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t UDS_DiagSession(UDS_Client_t *c, UDS_Session_t sess,
                               UDS_Response_t *resp)
{
    UDS_Request_t req = {
        .sid      = UDS_SID_DSC,
        .sub_func = (uint8_t)sess,
        .data_len = 0
    };
    VCI_Status_t st = UDS_Request(c, &req, resp);
    if (st == VCI_OK && resp->positive)
        c->session = sess;
    return st;
}

VCI_Status_t UDS_ECUReset(UDS_Client_t *c, uint8_t reset_type,
                            UDS_Response_t *resp)
{
    UDS_Request_t req = {
        .sid      = UDS_SID_ER,
        .sub_func = reset_type,
        .data_len = 0
    };
    return UDS_Request(c, &req, resp);
}

VCI_Status_t UDS_ReadDID(UDS_Client_t *c, uint16_t did,
                          uint8_t *out, uint32_t *out_len)
{
    UDS_Response_t resp = {0};
    UDS_Request_t  req  = {
        .sid      = UDS_SID_RDBI,
        .sub_func = 0xFF,         /* No sub-function for RDBI          */
        .data_id  = did,
        .data_len = 0
    };
    VCI_Status_t st = UDS_Request(c, &req, &resp);
    if (st == VCI_OK && resp.positive && out) {
        /* Skip the echoed DID (2 bytes) */
        uint32_t skip = (resp.data_len >= 2) ? 2 : 0;
        *out_len = resp.data_len - skip;
        memcpy(out, &resp.data[skip], *out_len);
    }
    return st;
}

VCI_Status_t UDS_WriteDID(UDS_Client_t *c, uint16_t did,
                           const uint8_t *data, uint32_t len)
{
    UDS_Response_t resp = {0};
    UDS_Request_t  req  = {
        .sid      = UDS_SID_WDBI,
        .sub_func = 0xFF,
        .data_id  = did,
        .data_len = len
    };
    memcpy(req.data, data, len);
    return UDS_Request(c, &req, &resp);
}

VCI_Status_t UDS_TesterPresent(UDS_Client_t *c)
{
    UDS_Response_t resp = {0};
    UDS_Request_t  req  = {
        .sid      = UDS_SID_TP,
        .sub_func = 0x80U,        /* Suppress positive response         */
        .data_len = 0
    };
    return UDS_Request(c, &req, &resp);
}

VCI_Status_t UDS_ClearDTC(UDS_Client_t *c, uint32_t group,
                            UDS_Response_t *resp)
{
    UDS_Request_t req = {
        .sid      = UDS_SID_CDTCI,
        .sub_func = 0xFF,
        .data_len = 3,
        .data     = {
            (uint8_t)((group >> 16) & 0xFF),
            (uint8_t)((group >>  8) & 0xFF),
            (uint8_t)( group        & 0xFF)
        }
    };
    return UDS_Request(c, &req, resp);
}

/* ════════════════════════════════════════════════════════════════════════
 *  UDS_Tick — TesterPresent keep-alive + ISO-TP timer
 * ════════════════════════════════════════════════════════════════════════ */
static uint32_t s_tp_timer = 0;
#define TP_INTERVAL_MS  2000U

void UDS_Tick(UDS_Client_t *c, uint32_t elapsed_ms)
{
    ISOTP_Tick(&c->isotp, elapsed_ms);

    if (c->session != UDS_SESSION_DEFAULT && c->tp_running) {
        s_tp_timer += elapsed_ms;
        if (s_tp_timer >= TP_INTERVAL_MS) {
            s_tp_timer = 0;
            UDS_TesterPresent(c);
        }
    }
}
