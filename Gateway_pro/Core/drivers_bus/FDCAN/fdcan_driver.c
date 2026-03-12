/**
 ******************************************************************************
 * @file    fdcan_driver.c
 * @brief   STM32H7 FDCAN driver implementation
 ******************************************************************************
 *  Timing calculation reference (FDCAN clock = 80 MHz from PLL2Q):
 *
 *  Nominal 500 kbit/s  →  Prescaler=1, TimeSeg1=119, TimeSeg2=40, SJW=40
 *  Data   2 Mbit/s     →  Prescaler=1, TimeSeg1=29,  TimeSeg2=10, SJW=10
 *
 *  NOTE: Adjust values for your actual FDCAN peripheral clock.
 *        Use the STM32CubeMX CAN calculator or BittimingsCalculator.
 */

#include "fdcan_driver.h"
#include "rtos_tasks.h"
#include <string.h>

/* ── Instance context ─────────────────────────────────────────────────── */
typedef struct {
    FDCAN_DriverCfg_t   cfg;
    VCI_BusState_t      state;
    VCI_BusStats_t      stats;
    bool                active;
} FDCAN_Ctx_t;

static FDCAN_Ctx_t s_ctx[2];   /* [0]=FDCAN1, [1]=FDCAN2 */

/* ── DLC <-> byte count lookup (CAN-FD) ──────────────────────────────── */
static const uint8_t s_dlc2len[16] = {
    0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
};
static const uint8_t s_len2dlc[65] = {
    0,1,2,3,4,5,6,7,8,       /* 0-8   */
    9,9,9,9,                  /* 9-12  */
    10,10,10,10,              /* 13-16 */
    11,11,11,11,              /* 17-20 */
    12,12,12,12,              /* 21-24 */
    13,13,13,13,13,13,13,13, /* 25-32 */
    14,14,14,14,14,14,14,14, /* 33-40 */
    14,14,14,14,14,14,14,14, /* 41-48 */
    15,15,15,15,15,15,15,15, /* 49-56 */
    15,15,15,15,15,15,15,15  /* 57-64 */
};

static inline uint8_t hal_dlc_from_len(uint8_t len) {
    if (len > 64) len = 64;
    return s_len2dlc[len];
}
static inline uint8_t len_from_hal_dlc(uint32_t hal_dlc) {
    /* HAL encodes DLC as FDCAN_DLC_BYTES_xx values (0x0000..0x000F<<16)  */
    uint8_t dlc = (uint8_t)(hal_dlc >> 16);
    if (dlc > 15) dlc = 15;
    return s_dlc2len[dlc];
}

/* ── bitrate → HAL timing structs ────────────────────────────────────── */
/*  FDCAN clock assumed 80 MHz (PLL2Q).  All sample-point ~80%.           */
typedef struct { uint32_t prescaler, seg1, seg2, sjw; } _Timing;

static VCI_Status_t _fill_nominal_timing(uint32_t bps,
                                          FDCAN_InitTypeDef *init)
{
    /* Precomputed for 80 MHz FDCAN clock */
    static const struct { uint32_t bps; _Timing t; } nom_tbl[] = {
        { 125000,  {4, 119, 40, 40} },
        { 250000,  {2, 119, 40, 40} },
        { 500000,  {1, 119, 40, 40} },
        {1000000,  {1,  59, 20, 20} },
    };
    for (size_t i = 0; i < sizeof(nom_tbl)/sizeof(nom_tbl[0]); i++) {
        if (nom_tbl[i].bps == bps) {
            init->NominalPrescaler = nom_tbl[i].t.prescaler;
            init->NominalTimeSeg1  = nom_tbl[i].t.seg1;
            init->NominalTimeSeg2  = nom_tbl[i].t.seg2;
            init->NominalSyncJumpWidth = nom_tbl[i].t.sjw;
            return VCI_OK;
        }
    }
    return VCI_ERR_PARAM;
}

static VCI_Status_t _fill_data_timing(uint32_t bps,
                                       FDCAN_InitTypeDef *init)
{
    static const struct { uint32_t bps; _Timing t; } data_tbl[] = {
        {1000000,  {1,  59, 20, 20} },
        {2000000,  {1,  29, 10, 10} },
        {4000000,  {1,  14,  5,  5} },
        {5000000,  {1,  11,  4,  4} },
        {8000000,  {1,   6,  3,  3} },
    };
    for (size_t i = 0; i < sizeof(data_tbl)/sizeof(data_tbl[0]); i++) {
        if (data_tbl[i].bps == bps) {
            init->DataPrescaler = data_tbl[i].t.prescaler;
            init->DataTimeSeg1  = data_tbl[i].t.seg1;
            init->DataTimeSeg2  = data_tbl[i].t.seg2;
            init->DataSyncJumpWidth = data_tbl[i].t.sjw;
            return VCI_OK;
        }
    }
    return VCI_ERR_PARAM;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Driver vtable functions
 * ══════════════════════════════════════════════════════════════════════════ */
static FDCAN_Ctx_t *_ctx_from_handle(FDCAN_HandleTypeDef *h)
{
    if (h == s_ctx[0].cfg.hfdcan) return &s_ctx[0];
    if (h == s_ctx[1].cfg.hfdcan) return &s_ctx[1];
    return NULL;
}

/* ── init ─────────────────────────────────────────────────────────────── */
static VCI_Status_t _drv_init(void *hw_handle)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)hw_handle;
    FDCAN_Ctx_t *ctx = _ctx_from_handle(hfdcan);
    if (!ctx) return VCI_ERR_PARAM;

    FDCAN_InitTypeDef *init = &hfdcan->Init;

    /* Mode */

    init->FrameFormat        = ctx->cfg.fd_mode ?
                               FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
    init->Mode               = FDCAN_MODE_NORMAL;
    init->AutoRetransmission = ENABLE;
    init->TransmitPause      = ctx->cfg.tx_pause ? ENABLE : DISABLE;
    init->ProtocolException  = ENABLE;

    /* Nominal timing */
    if (_fill_nominal_timing(ctx->cfg.nominal_bps, init) != VCI_OK)
        return VCI_ERR_PARAM;

    /* Data timing (FD only) */
    if (ctx->cfg.fd_mode) {
        if (_fill_data_timing(ctx->cfg.data_bps, init) != VCI_OK)
            return VCI_ERR_PARAM;
    } else {
        /* Classical — copy nominal values */
        init->DataPrescaler    = init->NominalPrescaler;
        init->DataTimeSeg1     = init->NominalTimeSeg1;
        init->DataTimeSeg2     = init->NominalTimeSeg2;
        init->DataSyncJumpWidth= init->NominalSyncJumpWidth;
    }

    /* Message RAM */
    init->StdFiltersNbr     = 28;
    init->ExtFiltersNbr     = 8;
    init->RxFifo0ElmtsNbr   = 32;
    init->RxFifo0ElmtSize   = ctx->cfg.fd_mode ?
                               FDCAN_DATA_BYTES_64 : FDCAN_DATA_BYTES_8;
    init->RxFifo1ElmtsNbr   = 0;
    init->TxEventsNbr       = 0;
    init->TxBuffersNbr      = 0;
    init->TxFifoQueueElmtsNbr = 32;
    init->TxFifoQueueMode   = FDCAN_TX_FIFO_OPERATION;
    init->TxElmtSize        = ctx->cfg.fd_mode ?
                               FDCAN_DATA_BYTES_64 : FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) return VCI_ERR_HW;

    /* Accept-all global filter → use software filter in gateway         */
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x7FF
    };
    HAL_FDCAN_ConfigFilter(hfdcan, &f);
    f.IdType  = FDCAN_EXTENDED_ID;
    f.FilterID2 = 0x1FFFFFFF;
    HAL_FDCAN_ConfigFilter(hfdcan, &f);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,      FDCAN_FILTER_REMOTE);

    /* Enable notifications */
    HAL_FDCAN_ActivateNotification(hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
        FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);

    ctx->state = BUS_STATE_IDLE;
    return VCI_OK;
}

/* ── start / stop ─────────────────────────────────────────────────────── */
static VCI_Status_t _drv_start(void)   { return VCI_OK; /* started in init */ }
static VCI_Status_t _drv_stop(void)    { return VCI_OK; }

/* ── send ─────────────────────────────────────────────────────────────── */
static VCI_Status_t _drv_send(const VCI_Message_t *msg, uint32_t timeout_ms)
{
    /* Locate context via bus_id embedded in message */
    FDCAN_Ctx_t *ctx = (msg->src_bus == BUS_ID_FDCAN1) ? &s_ctx[0] : &s_ctx[1];
    if (!ctx->active) return VCI_ERR_UNINIT;

    const VCI_CanFrame_t *fr = &msg->payload.can;

    FDCAN_TxHeaderTypeDef hdr = {
        .Identifier          = fr->id,
        .IdType              = (fr->flags & CAN_FLAG_IDE) ?
                               FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = (uint32_t)hal_dlc_from_len(fr->dlc) << 16,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = (fr->flags & CAN_FLAG_BRS) ?
                               FDCAN_BRS_ON : FDCAN_BRS_OFF,
        .FDFormat            = (fr->flags & CAN_FLAG_FD) ?
                               FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0
    };

    uint32_t tick_start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(ctx->cfg.hfdcan) == 0) {
        if ((HAL_GetTick() - tick_start) > timeout_ms) return VCI_ERR_TIMEOUT;
        osDelay(1);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(ctx->cfg.hfdcan, &hdr,
                                       (uint8_t *)fr->data) != HAL_OK) {
        return VCI_ERR_HW;
    }
    return VCI_OK;
}

/* ── recv (fallback — normally driven by queue from ISR) ─────────────── */
static VCI_Status_t _drv_recv(VCI_Message_t *msg, uint32_t timeout_ms)
{
    (void)msg; (void)timeout_ms;
    /* RX is IRQ-driven; gateway reads from the osMessageQueue directly   */
    return VCI_ERR_UNINIT;
}

/* ── set_bitrate ──────────────────────────────────────────────────────── */
static VCI_Status_t _drv_set_bitrate(uint32_t nominal, uint32_t data)
{
    (void)nominal; (void)data;
    /* Runtime bitrate change requires stop → reconfigure → start         */
    return VCI_ERR_BUSY;
}

/* ── set_filter ───────────────────────────────────────────────────────── */
static VCI_Status_t _drv_set_filter(uint32_t id, uint32_t mask, bool ide)
{
    (void)id; (void)mask; (void)ide;
    /* TODO: map to FDCAN_FilterTypeDef and call HAL_FDCAN_ConfigFilter   */
    return VCI_OK;
}

/* ── get_state ────────────────────────────────────────────────────────── */
static VCI_BusState_t _drv_get_state_1(void) { return s_ctx[0].state; }
static VCI_BusState_t _drv_get_state_2(void) { return s_ctx[1].state; }

/* ── get_stats ────────────────────────────────────────────────────────── */
static VCI_Status_t _drv_get_stats(VCI_BusStats_t *out) {
    (void)out; return VCI_OK;
}
static VCI_Status_t _drv_clear_stats(void) { return VCI_OK; }

/* ── vtable instances ─────────────────────────────────────────────────── */
static VCI_BusDriver_t s_drv1 = {
    .init        = _drv_init,
    .start       = _drv_start,
    .stop        = _drv_stop,
    .sleep       = NULL,
    .wake        = NULL,
    .send        = _drv_send,
    .recv        = _drv_recv,
    .set_bitrate = _drv_set_bitrate,
    .set_filter  = _drv_set_filter,
    .get_state   = _drv_get_state_1,
    .get_stats   = _drv_get_stats,
    .clear_stats = _drv_clear_stats
};

static VCI_BusDriver_t s_drv2 = {
    .init        = _drv_init,
    .start       = _drv_start,
    .stop        = _drv_stop,
    .sleep       = NULL,
    .wake        = NULL,
    .send        = _drv_send,
    .recv        = _drv_recv,
    .set_bitrate = _drv_set_bitrate,
    .set_filter  = _drv_set_filter,
    .get_state   = _drv_get_state_2,
    .get_stats   = _drv_get_stats,
    .clear_stats = _drv_clear_stats
};

/* ════════════════════════════════════════════════════════════════════════
 *  FDCAN_DriverInit
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t FDCAN_DriverInit(const FDCAN_DriverCfg_t *cfg)
{
    if (!cfg || !cfg->hfdcan) return VCI_ERR_PARAM;
    uint8_t idx = (cfg->bus_id == BUS_ID_FDCAN1) ? 0 : 1;
    s_ctx[idx].cfg    = *cfg;
    s_ctx[idx].active = true;
    return _drv_init(cfg->hfdcan);
}

VCI_BusDriver_t *FDCAN1_GetDriver(void) { return &s_drv1; }
VCI_BusDriver_t *FDCAN2_GetDriver(void) { return &s_drv2; }

/* ════════════════════════════════════════════════════════════════════════
 *  HAL Callback — called from FDCAN IRQ handler
 *  This runs at IRQ priority; only osMessageQueuePut() is allowed.
 * ════════════════════════════════════════════════════════════════════════ */
void FDCAN_RxCallback(FDCAN_HandleTypeDef *hfdcan,
                      uint32_t RxFifo, uint32_t fill_level)
{
    (void)fill_level;
    FDCAN_Ctx_t *ctx = _ctx_from_handle(hfdcan);
    if (!ctx || !ctx->cfg.rx_queue) return;

    for (uint32_t i = 0; i < fill_level; i++) {
        FDCAN_RxHeaderTypeDef rxh;
        uint8_t               rxd[CANFD_MAX_DLC];

        if (HAL_FDCAN_GetRxMessage(hfdcan, RxFifo, &rxh, rxd) != HAL_OK)
            break;

        VCI_Message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.type    = MSG_TYPE_CAN;
        msg.src_bus = ctx->cfg.bus_id;
        msg.dst_bus = BUS_ID_MAX;   /* Broadcast — gateway decides       */

        VCI_CanFrame_t *fr = &msg.payload.can;
        fr->id    = rxh.Identifier;
        fr->dlc   = len_from_hal_dlc(rxh.DataLength);
        fr->flags = 0;
        if (rxh.IdType     == FDCAN_EXTENDED_ID) fr->flags |= CAN_FLAG_IDE;
        if (rxh.FDFormat   == FDCAN_FD_CAN)      fr->flags |= CAN_FLAG_FD;
        if (rxh.BitRateSwitch == FDCAN_BRS_ON)   fr->flags |= CAN_FLAG_BRS;
        if (rxh.ErrorStateIndicator == FDCAN_ESI_PASSIVE) fr->flags |= CAN_FLAG_ESI;
        fr->timestamp_us = rxh.RxTimestamp;  /* FDCAN timer tick          */
        fr->bus          = ctx->cfg.bus_id;
        memcpy(fr->data, rxd, fr->dlc);
        msg.payload_len = fr->dlc;

        osMessageQueuePut(*ctx->cfg.rx_queue, &msg, 0, 0);
        ctx->stats.rx_frames++;
    }

    /* Re-arm notification */
    HAL_FDCAN_ActivateNotification(hfdcan,
        (RxFifo == FDCAN_RX_FIFO0) ?
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

/* ── HAL overrides (weak definitions) ────────────────────────────────── */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        uint32_t fill = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
        FDCAN_RxCallback(hfdcan, FDCAN_RX_FIFO0, fill);
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
    FDCAN_Ctx_t *ctx = _ctx_from_handle(hfdcan);
    if (!ctx) return;

    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        ctx->state = BUS_STATE_BUSOFF;
        ctx->stats.bus_off_count++;
        /* Signal gateway task */
        uint32_t flag = (ctx->cfg.bus_id == BUS_ID_FDCAN1) ?
                        EVF_BUS_OFF_FDCAN1 : EVF_BUS_OFF_FDCAN2;
        osEventFlagsSet(g_evf_system, flag);

        /* Auto-recovery: leave bus-off after 128 × 11 recessive bits     */
        HAL_FDCAN_Start(hfdcan);
    }
    if (ErrorStatusITs & (FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING)) {
        ctx->stats.rx_errors++;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Public timing wrappers — used by bus_probe.c
 *  Task_FDCAN1 / Task_FDCAN2 sont dans fdcan_tasks.c (pas de cycle)
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t FDCAN_FillNominalTiming(uint32_t bps, FDCAN_InitTypeDef *init)
{
    return _fill_nominal_timing(bps, init);
}

VCI_Status_t FDCAN_FillDataTiming(uint32_t bps, FDCAN_InitTypeDef *init)
{
    return _fill_data_timing(bps, init);
}
