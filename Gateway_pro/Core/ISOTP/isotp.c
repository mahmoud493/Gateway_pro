/**
 ******************************************************************************
 * @file    isotp.c
 * @brief   ISO 15765-2 Transport Protocol — full state-machine implementation
 ******************************************************************************
 */

#include "isotp.h"
#include "bus_manager.h"
#include <string.h>

/* ── Helpers ──────────────────────────────────────────────────────────── */
static uint8_t _pci_type(const uint8_t *data)
{
    return data[0] & 0xF0U;
}

/* Maximum payload bytes per SF depending on mode */
static uint8_t _sf_max_payload(const ISOTP_Channel_t *ch)
{
    return ch->cfg.fd_mode ? 62U : 7U;
}

/* Build and transmit a CAN frame through BusMgr */
static VCI_Status_t _send_can(ISOTP_Channel_t *ch,
                               const uint8_t *data, uint8_t len)
{
    VCI_Message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type    = MSG_TYPE_CAN;
    msg.src_bus = ch->cfg.bus;
    msg.dst_bus = ch->cfg.bus;

    VCI_CanFrame_t *fr = &msg.payload.can;
    fr->id  = ch->cfg.tx_id;
    fr->bus = ch->cfg.bus;
    if (ch->cfg.ext_id) fr->flags |= CAN_FLAG_IDE;
    if (ch->cfg.fd_mode) {
        fr->flags |= CAN_FLAG_FD | CAN_FLAG_BRS;
        fr->dlc = 64U;
        memset(fr->data, 0xCC, 64); /* ISO 15765-2 padding byte = 0xCC   */
    } else {
        fr->dlc = 8U;
    }
    if (len > fr->dlc) len = fr->dlc;
    memcpy(fr->data, data, len);
    msg.payload_len = fr->dlc;

    return BusMgr_Send(ch->cfg.bus, &msg, ISOTP_N_As_MS);
}

/* ════════════════════════════════════════════════════════════════════════
 *  ISOTP_ChannelInit
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t ISOTP_ChannelInit(ISOTP_Channel_t *ch,
                                const ISOTP_ChannelCfg_t *cfg)
{
    if (!ch || !cfg) return VCI_ERR_PARAM;
    memset(ch, 0, sizeof(*ch));
    ch->cfg   = *cfg;
    ch->state = ISOTP_STATE_IDLE;
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  ISOTP_Send
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t ISOTP_Send(ISOTP_Channel_t *ch,
                         const uint8_t *data, uint32_t len)
{
    if (!ch || !data || len == 0 || len > ISOTP_BUFFER_SIZE)
        return VCI_ERR_PARAM;
    if (ch->state != ISOTP_STATE_IDLE)
        return VCI_ERR_BUSY;

    uint8_t sf_max = _sf_max_payload(ch);
    uint8_t frame[64];

    /* ── Single Frame ───────────────────────────────────────────────── */
    if (len <= sf_max) {
        if (ch->cfg.fd_mode && len > 7U) {
            /* FD SF: escape sequence */
            frame[0] = ISOTP_PCI_SF;        /* PCI = 0x00               */
            frame[1] = (uint8_t)len;         /* Actual length            */
            memcpy(&frame[2], data, len);
            return _send_can(ch, frame, (uint8_t)(2 + len));
        } else {
            frame[0] = (uint8_t)(ISOTP_PCI_SF | (len & 0x0FU));
            memcpy(&frame[1], data, len);
            return _send_can(ch, frame, (uint8_t)(1 + len));
        }
    }

    /* ── Multi-frame: First Frame ───────────────────────────────────── */
    uint8_t ff_payload;
    uint8_t ff_hdr_len;

    if (len <= 0xFFF) {
        /* Normal FF */
        frame[0] = (uint8_t)(ISOTP_PCI_FF | ((len >> 8) & 0x0FU));
        frame[1] = (uint8_t)(len & 0xFFU);
        ff_hdr_len  = 2U;
    } else {
        /* Long FF (> 4095 bytes) */
        frame[0] = ISOTP_PCI_FF;
        frame[1] = 0x00U;
        frame[2] = (uint8_t)((len >> 24) & 0xFFU);
        frame[3] = (uint8_t)((len >> 16) & 0xFFU);
        frame[4] = (uint8_t)((len >>  8) & 0xFFU);
        frame[5] = (uint8_t)( len        & 0xFFU);
        ff_hdr_len = 6U;
    }
    ff_payload = (ch->cfg.fd_mode ? 62U : 8U) - ff_hdr_len;
    memcpy(&frame[ff_hdr_len], data, ff_payload);

    VCI_Status_t st = _send_can(ch, frame,
                                 (uint8_t)(ff_hdr_len + ff_payload));
    if (st != VCI_OK) return st;

    /* Prepare for segmented TX */
    ch->tx_buf       = data + ff_payload;
    ch->tx_total_len = len;
    ch->tx_sent      = ff_payload;
    ch->tx_sn        = 1U;
    ch->tx_bs_cnt    = 0U;
    ch->tx_stmin_ms  = 0U;
    ch->tx_timer_ms  = ISOTP_N_Bs_MS;
    ch->state        = ISOTP_STATE_TX_WAIT_FC;
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  _process_rx_sf — Single Frame received
 * ════════════════════════════════════════════════════════════════════════ */
static VCI_Status_t _process_sf(ISOTP_Channel_t *ch,
                                 const uint8_t *data, uint8_t dlc)
{
    (void)dlc;
    uint8_t len;
    uint8_t offset;

    if ((data[0] & 0x0FU) == 0U && data[1] != 0U) {
        /* FD escape: length in byte 1 */
        len    = data[1];
        offset = 2U;
    } else {
        len    = data[0] & 0x0FU;
        offset = 1U;
    }
    if (len == 0 || len > ISOTP_BUFFER_SIZE) return VCI_ERR_PROTO;

    memcpy(ch->rx_buf, &data[offset], len);
    ch->rx_total_len = len;
    ch->state        = ISOTP_STATE_IDLE;

    if (ch->on_rx_complete)
        ch->on_rx_complete(ch->rx_buf, len, ch->user_ctx);
    return VCI_OK;
}

/* ── First Frame received ─────────────────────────────────────────────── */
static VCI_Status_t _process_ff(ISOTP_Channel_t *ch,
                                  const uint8_t *data, uint8_t dlc)
{
    uint32_t total;
    uint8_t  payload_start;

    if ((data[0] & 0x0FU) == 0U && data[1] == 0U) {
        /* Long FF */
        total         = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
                        ((uint32_t)data[4] <<  8) |  (uint32_t)data[5];
        payload_start = 6U;
    } else {
        total = (((uint32_t)(data[0] & 0x0FU)) << 8) | data[1];
        payload_start = 2U;
    }
    if (total > ISOTP_BUFFER_SIZE) {
        /* Send overflow FC */
        uint8_t fc[3] = { ISOTP_PCI_FC | ISOTP_FC_OVFLW, 0, 0 };
        _send_can(ch, fc, 3);
        return VCI_ERR_OVFLOW;
    }

    uint8_t payload_len = dlc - payload_start;
    memcpy(ch->rx_buf, &data[payload_start], payload_len);
    ch->rx_total_len = total;
    ch->rx_received  = payload_len;
    ch->rx_sn        = 1U;
    ch->rx_timer_ms  = ISOTP_N_Cr_MS;
    ch->state        = ISOTP_STATE_RX_WAIT_CF;

    /* Send CTS Flow Control */
    uint8_t fc[3] = {
        (uint8_t)(ISOTP_PCI_FC | ISOTP_FC_CTS),
        ch->cfg.blocksize,
        ch->cfg.stmin_ms
    };
    return _send_can(ch, fc, 3);
}

/* ── Consecutive Frame received ──────────────────────────────────────── */
static VCI_Status_t _process_cf(ISOTP_Channel_t *ch,
                                  const uint8_t *data, uint8_t dlc)
{
    if (ch->state != ISOTP_STATE_RX_WAIT_CF) return VCI_ERR_PROTO;

    uint8_t sn = data[0] & 0x0FU;
    if (sn != (ch->rx_sn & 0x0FU)) {
        ch->state = ISOTP_STATE_ERROR;
        return VCI_ERR_PROTO;
    }
    ch->rx_sn++;

    uint8_t  payload_len = dlc - 1U;
    uint32_t remaining   = ch->rx_total_len - ch->rx_received;
    if (payload_len > remaining) payload_len = (uint8_t)remaining;

    memcpy(&ch->rx_buf[ch->rx_received], &data[1], payload_len);
    ch->rx_received += payload_len;
    ch->rx_timer_ms  = ISOTP_N_Cr_MS;  /* Restart timer                  */

    if (ch->rx_received >= ch->rx_total_len) {
        ch->state = ISOTP_STATE_IDLE;
        if (ch->on_rx_complete)
            ch->on_rx_complete(ch->rx_buf, ch->rx_total_len, ch->user_ctx);
    }
    return VCI_OK;
}

/* ── Flow Control received ────────────────────────────────────────────── */
static VCI_Status_t _process_fc(ISOTP_Channel_t *ch,
                                  const uint8_t *data)
{
    if (ch->state != ISOTP_STATE_TX_WAIT_FC) return VCI_ERR_PROTO;

    uint8_t fs = data[0] & 0x0FU;
    if (fs == ISOTP_FC_OVFLW) { ch->state = ISOTP_STATE_ERROR; return VCI_ERR_OVFLOW; }
    if (fs == ISOTP_FC_WAIT)  { ch->tx_timer_ms = ISOTP_N_Bs_MS; return VCI_OK; }

    /* CTS */
    ch->tx_bs_cnt   = data[1];   /* 0 = unlimited */
    ch->tx_stmin_ms = data[2];
    ch->state       = ISOTP_STATE_TX_SENDING_CF;
    ch->tx_timer_ms = ch->tx_stmin_ms;

    /* Send first batch of CF frames */
    uint8_t frame[64];
    uint8_t cf_payload = ch->cfg.fd_mode ? 63U : 7U;

    while (ch->tx_sent < ch->tx_total_len &&
           (ch->tx_bs_cnt == 0 || ch->tx_bs_cnt > 0)) {
        uint32_t remaining = ch->tx_total_len - ch->tx_sent;
        uint8_t  chunk     = (remaining > cf_payload) ? cf_payload :
                             (uint8_t)remaining;

        frame[0] = (uint8_t)(ISOTP_PCI_CF | (ch->tx_sn & 0x0FU));
        memcpy(&frame[1], ch->tx_buf + ch->tx_sent, chunk);
        ch->tx_sn++;

        VCI_Status_t st = _send_can(ch, frame, (uint8_t)(1 + chunk));
        if (st != VCI_OK) { ch->state = ISOTP_STATE_ERROR; return st; }

        ch->tx_sent += chunk;
        if (ch->tx_bs_cnt > 0) ch->tx_bs_cnt--;

        /* Wait STmin between frames */
        if (ch->tx_stmin_ms > 0) osDelay(ch->tx_stmin_ms);

        if (ch->tx_bs_cnt == 0 && data[1] != 0) {
            /* Block exhausted — wait for next FC */
            ch->state = ISOTP_STATE_TX_WAIT_FC;
            ch->tx_timer_ms = ISOTP_N_Bs_MS;
            return VCI_OK;
        }
    }

    /* All sent */
    ch->state = ISOTP_STATE_IDLE;
    if (ch->on_tx_complete) ch->on_tx_complete(VCI_OK, ch->user_ctx);
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  ISOTP_ProcessRxFrame
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t ISOTP_ProcessRxFrame(ISOTP_Channel_t *ch,
                                   const VCI_CanFrame_t *fr)
{
    if (!ch || !fr) return VCI_ERR_PARAM;
    if (fr->id != ch->cfg.rx_id) return VCI_OK; /* Not for this channel  */

    const uint8_t *data = fr->data;
    uint8_t        dlc  = fr->dlc;
    uint8_t        pci  = _pci_type(data);

    switch (pci) {
        case ISOTP_PCI_SF: return _process_sf(ch, data, dlc);
        case ISOTP_PCI_FF: return _process_ff(ch, data, dlc);
        case ISOTP_PCI_CF: return _process_cf(ch, data, dlc);
        case ISOTP_PCI_FC: return _process_fc(ch, data);
        default:           return VCI_ERR_PROTO;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  ISOTP_Tick  — call every 1 ms from ISOTP task
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t ISOTP_Tick(ISOTP_Channel_t *ch, uint32_t elapsed_ms)
{
    if (!ch || ch->state == ISOTP_STATE_IDLE) return VCI_OK;
    if (ch->tx_timer_ms > elapsed_ms)
        ch->tx_timer_ms -= elapsed_ms;
    else
        ch->tx_timer_ms = 0;

    if (ch->rx_timer_ms > elapsed_ms)
        ch->rx_timer_ms -= elapsed_ms;
    else
        ch->rx_timer_ms = 0;

    /* Timeout detection */
    if ((ch->state == ISOTP_STATE_RX_WAIT_CF    && ch->rx_timer_ms == 0) ||
        (ch->state == ISOTP_STATE_TX_WAIT_FC     && ch->tx_timer_ms == 0) ||
        (ch->state == ISOTP_STATE_TX_SENDING_CF  && ch->tx_timer_ms == 0)) {
        ch->state = ISOTP_STATE_ERROR;
        if (ch->on_tx_complete)
            ch->on_tx_complete(VCI_ERR_TIMEOUT, ch->user_ctx);
        ch->state = ISOTP_STATE_IDLE;
        return VCI_ERR_TIMEOUT;
    }
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  ISOTP_Abort
 * ════════════════════════════════════════════════════════════════════════ */
void ISOTP_Abort(ISOTP_Channel_t *ch)
{
    if (!ch) return;
    ch->state = ISOTP_STATE_IDLE;
    memset(ch->rx_buf, 0, sizeof(ch->rx_buf));
}
