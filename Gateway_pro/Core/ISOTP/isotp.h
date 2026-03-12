/**
 ******************************************************************************
 * @file    isotp.h
 * @brief   VCI Gateway — ISO 15765-2 Transport Protocol (ISO-TP)
 ******************************************************************************
 *  Implements the full ISO-TP frame segmentation / reassembly state machine:
 *
 *    Single Frame     (SF)  — payload ≤ 7 bytes (classic) / 62 bytes (FD)
 *    First Frame      (FF)  — first segment of multi-frame message
 *    Consecutive Frame(CF)  — subsequent segments
 *    Flow Control     (FC)  — receiver pacing
 *
 *  Supports:
 *   • Classical CAN  (8-byte DLC, 7-byte effective payload per SF)
 *   • CAN-FD padding (up to 62 bytes per SF with FD DLC=64)
 *   • Functional (broadcast) and physical (point-to-point) addressing
 *   • N_Bs, N_Cs, N_Cr timers per ISO 15765-2:2016
 */

#ifndef ISOTP_H
#define ISOTP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "bus_manager.h"

/* ── Timer values (ms) ────────────────────────────────────────────────── */
#define ISOTP_N_As_MS   25U    /* TX frame transmit timeout              */
#define ISOTP_N_Bs_MS   75U    /* Wait for FC after FF                   */
#define ISOTP_N_Cs_MS   25U    /* TX consecutive frame timeout           */
#define ISOTP_N_Cr_MS   150U   /* Wait for CF after FC or prev CF        */

/* ── Frame type nibble ────────────────────────────────────────────────── */
#define ISOTP_PCI_SF    0x00U
#define ISOTP_PCI_FF    0x10U
#define ISOTP_PCI_CF    0x20U
#define ISOTP_PCI_FC    0x30U

/* ── Flow status ──────────────────────────────────────────────────────── */
#define ISOTP_FC_CTS    0x00U  /* Continue to send                       */
#define ISOTP_FC_WAIT   0x01U  /* Wait                                   */
#define ISOTP_FC_OVFLW  0x02U  /* Overflow / abort                       */

/* ── Addressing mode ──────────────────────────────────────────────────── */
typedef enum {
    ISOTP_ADDR_NORMAL     = 0,   /* Normal fixed (11-bit ID)             */
    ISOTP_ADDR_EXTENDED   = 1,   /* Extended (target address in byte 0)  */
    ISOTP_ADDR_MIXED_11   = 2,   /* Mixed 11-bit (AE in byte 0)          */
    ISOTP_ADDR_MIXED_29   = 3    /* Mixed 29-bit                         */
} ISOTP_AddrMode_t;

/* ── Channel configuration ────────────────────────────────────────────── */
typedef struct {
    VCI_BusID_t      bus;
    uint32_t         tx_id;       /* CAN ID for outgoing frames          */
    uint32_t         rx_id;       /* CAN ID to listen on                 */
    bool             ext_id;      /* 29-bit addressing                   */
    ISOTP_AddrMode_t addr_mode;
    uint8_t          addr_ext;    /* Extended address byte (if applicable) */
    bool             fd_mode;     /* Use CAN-FD padding                  */
    uint8_t          stmin_ms;    /* Min separation time (TX side)       */
    uint8_t          blocksize;   /* FC block size (0 = no limit)        */
} ISOTP_ChannelCfg_t;

/* ── Channel state machine ────────────────────────────────────────────── */
typedef enum {
    ISOTP_STATE_IDLE = 0,
    ISOTP_STATE_RX_WAIT_CF,     /* Waiting for consecutive frames       */
    ISOTP_STATE_TX_WAIT_FC,     /* Waiting for flow control             */
    ISOTP_STATE_TX_SENDING_CF,  /* Sending consecutive frames           */
    ISOTP_STATE_ERROR
} ISOTP_State_t;

/* ── Channel handle ───────────────────────────────────────────────────── */
typedef struct {
    ISOTP_ChannelCfg_t  cfg;
    ISOTP_State_t       state;

    /* RX reassembly */
    uint8_t             rx_buf[ISOTP_BUFFER_SIZE];
    uint32_t            rx_total_len;
    uint32_t            rx_received;
    uint8_t             rx_sn;          /* Expected sequence number       */
    uint32_t            rx_timer_ms;    /* N_Cr deadline                  */

    /* TX segmentation */
    const uint8_t      *tx_buf;
    uint32_t            tx_total_len;
    uint32_t            tx_sent;
    uint8_t             tx_sn;          /* Next SN to send                */
    uint8_t             tx_bs_cnt;      /* Remaining in current block     */
    uint8_t             tx_stmin_ms;    /* STmin from FC                  */
    uint32_t            tx_timer_ms;    /* N_Bs / N_Cs deadline           */

    /* Callbacks */
    void (*on_rx_complete)(uint8_t *data, uint32_t len, void *ctx);
    void (*on_tx_complete)(VCI_Status_t result, void *ctx);
    void               *user_ctx;
} ISOTP_Channel_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t ISOTP_ChannelInit    (ISOTP_Channel_t *ch,
                                   const ISOTP_ChannelCfg_t *cfg);
VCI_Status_t ISOTP_Send           (ISOTP_Channel_t *ch,
                                   const uint8_t *data, uint32_t len);
VCI_Status_t ISOTP_ProcessRxFrame (ISOTP_Channel_t *ch,
                                   const VCI_CanFrame_t *fr);
VCI_Status_t ISOTP_Tick           (ISOTP_Channel_t *ch, uint32_t elapsed_ms);
void         ISOTP_Abort          (ISOTP_Channel_t *ch);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_H */
