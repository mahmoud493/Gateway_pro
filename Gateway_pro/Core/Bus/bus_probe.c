/**
 ******************************************************************************
 * @file    bus_probe.c
 * @brief   VCI Gateway — Bus auto-detection implementation
 ******************************************************************************
 */

#include "bus_probe.h"
#include "fdcan_driver.h"   /* FDCAN_FillNominalTiming, FDCAN_FillDataTiming */
#include "bus_manager.h"
#include "rtos_tasks.h"
#include <string.h>

/* ── Contexte interne ─────────────────────────────────────────────────── */
static BusProbeCfg_t s_cfg[BUS_ID_MAX];

/* ════════════════════════════════════════════════════════════════════════
 *  BusProbe_Init
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusProbe_Init(const BusProbeCfg_t *cfg)
{
    if (!cfg || cfg->bus >= BUS_ID_MAX) return VCI_ERR_PARAM;
    s_cfg[cfg->bus] = *cfg;
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  _setup_accept_all_filter
 * ════════════════════════════════════════════════════════════════════════ */
static void _setup_accept_all_filter(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x7FF
    };
    HAL_FDCAN_ConfigFilter(hfdcan, &f);
    f.IdType    = FDCAN_EXTENDED_ID;
    f.FilterID2 = 0x1FFFFFFF;
    HAL_FDCAN_ConfigFilter(hfdcan, &f);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

/* ════════════════════════════════════════════════════════════════════════
 *  _probe_can_baudrate
 *  Configure FDCAN en BUS_MONITORING (listen-only, jamais de TX)
 *  puis compte les trames valides reçues pendant listen_ms.
 * ════════════════════════════════════════════════════════════════════════ */
static bool _probe_can_baudrate(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t nominal_bps,
                                 uint32_t *frame_count,
                                 uint32_t listen_ms)
{
    HAL_FDCAN_Stop(hfdcan);

    /* Mode listen-only : hardware interdit tout TX, y compris ACK       */
    hfdcan->Init.Mode               = FDCAN_MODE_BUS_MONITORING;
    hfdcan->Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
    hfdcan->Init.AutoRetransmission = DISABLE;
    hfdcan->Init.TransmitPause      = DISABLE;
    hfdcan->Init.ProtocolException  = ENABLE;

    /* ── Utiliser l'API publique ──────────────────────────────────────── */
    if (FDCAN_FillNominalTiming(nominal_bps, &hfdcan->Init) != VCI_OK)
        return false;

    /* Copier nominal → data (classic CAN, data phase ignorée)           */
    hfdcan->Init.DataPrescaler     = hfdcan->Init.NominalPrescaler;
    hfdcan->Init.DataTimeSeg1      = hfdcan->Init.NominalTimeSeg1;
    hfdcan->Init.DataTimeSeg2      = hfdcan->Init.NominalTimeSeg2;
    hfdcan->Init.DataSyncJumpWidth = hfdcan->Init.NominalSyncJumpWidth;

    hfdcan->Init.StdFiltersNbr        = 1;
    hfdcan->Init.ExtFiltersNbr        = 1;
    hfdcan->Init.RxFifo0ElmtsNbr     = 16;
    hfdcan->Init.RxFifo0ElmtSize     = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxFifoQueueElmtsNbr = 0;   /* Pas de TX pendant probe  */
    hfdcan->Init.TxElmtSize          = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) return false;

    _setup_accept_all_filter(hfdcan);
    HAL_FDCAN_Start(hfdcan);

    /* ── Phase d'écoute ─────────────────────────────────────────────── */
    uint32_t t0        = osKernelGetTickCount();
    uint32_t rx_count  = 0;
    uint32_t err_count = 0;

    while ((osKernelGetTickCount() - t0) < listen_ms) {

        uint32_t fill = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
        if (fill > 0) {
            FDCAN_RxHeaderTypeDef rxh;
            uint8_t rxd[8];
            if (HAL_FDCAN_GetRxMessage(hfdcan,
                    FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK) {
                rx_count++;
            }
        }

        /* PSR : error warning / error passive / bus-off = mauvais baud  */
        uint32_t psr = hfdcan->Instance->PSR;
        if (psr & (FDCAN_PSR_EW | FDCAN_PSR_EP | FDCAN_PSR_BO))
            err_count++;

        osDelay(1);
    }

    HAL_FDCAN_Stop(hfdcan);
    *frame_count = rx_count;

    /*
     * Critère :
     *  ≥3 trames reçues ET ratio erreurs < 5× les trames → bus détecté
     *  0 trame + beaucoup d'erreurs → mauvais baudrate
     */
    if (rx_count >= 3 && err_count < rx_count * 5U) return true;
    if (rx_count > 0  && err_count == 0)             return true;
    return false;
}

/* ════════════════════════════════════════════════════════════════════════
 *  _probe_canfd_data_rate
 * ════════════════════════════════════════════════════════════════════════ */
static bool _probe_canfd_data_rate(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t nominal_bps,
                                    uint32_t data_bps,
                                    uint32_t *frame_count,
                                    uint32_t listen_ms)
{
    HAL_FDCAN_Stop(hfdcan);
    hfdcan->Init.Mode        = FDCAN_MODE_BUS_MONITORING;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;

    /* ── Utiliser l'API publique ──────────────────────────────────────── */
    if (FDCAN_FillNominalTiming(nominal_bps, &hfdcan->Init) != VCI_OK)
        return false;
    if (FDCAN_FillDataTiming(data_bps, &hfdcan->Init) != VCI_OK)
        return false;

    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) return false;

    _setup_accept_all_filter(hfdcan);
    HAL_FDCAN_Start(hfdcan);

    uint32_t t0       = osKernelGetTickCount();
    uint32_t fd_count = 0;

    while ((osKernelGetTickCount() - t0) < listen_ms) {
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            FDCAN_RxHeaderTypeDef rxh;
            uint8_t rxd[64];
            if (HAL_FDCAN_GetRxMessage(hfdcan,
                    FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK) {
                if (rxh.FDFormat      == FDCAN_FD_CAN &&
                    rxh.BitRateSwitch == FDCAN_BRS_ON) {
                    fd_count++;
                }
            }
        }
        osDelay(1);
    }

    HAL_FDCAN_Stop(hfdcan);
    *frame_count = fd_count;
    return (fd_count >= 1);
}

/* ════════════════════════════════════════════════════════════════════════
 *  _probe_lin  — écoute passive UART, cherche BREAK(0x00) + SYNC(0x55)
 * ════════════════════════════════════════════════════════════════════════ */
static bool _probe_lin(UART_HandleTypeDef *huart, uint32_t *detected_baud)
{
    static const uint32_t lin_bauds[] = { 19200, 9600, 4800, 2400 };

    for (uint8_t i = 0; i < sizeof(lin_bauds)/sizeof(lin_bauds[0]); i++) {
        huart->Init.BaudRate = lin_bauds[i];
        HAL_UART_Init(huart);

        uint8_t  buf[32];
        uint8_t  received = 0;
        uint32_t t0 = osKernelGetTickCount();

        while ((osKernelGetTickCount() - t0) < 100U) {
            uint8_t byte;
            if (HAL_UART_Receive(huart, &byte, 1, 2U) == HAL_OK) {
                buf[received % sizeof(buf)] = byte;
                received++;
            }
        }

        for (uint8_t j = 1; j < received; j++) {
            if (buf[(j-1U) % sizeof(buf)] == 0x00U &&
                buf[ j      % sizeof(buf)] == 0x55U) {
                *detected_baud = lin_bauds[i];
                return true;
            }
        }
    }
    return false;
}

/* ════════════════════════════════════════════════════════════════════════
 *  _probe_kline — écoute passive, détecte activité à 10400 bps
 * ════════════════════════════════════════════════════════════════════════ */
static bool _probe_kline(UART_HandleTypeDef *huart)
{
    huart->Init.BaudRate = KLINE_BAUDRATE_FAST;
    HAL_UART_Init(huart);

    uint8_t  byte;
    uint32_t count = 0;
    uint32_t t0    = osKernelGetTickCount();

    while ((osKernelGetTickCount() - t0) < 300U) {
        if (HAL_UART_Receive(huart, &byte, 1, 5U) == HAL_OK)
            count++;
    }
    return (count >= 3U);
}

/* ════════════════════════════════════════════════════════════════════════
 *  BusProbe_Run
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusProbe_Run(VCI_BusID_t bus, BusProbeResult_t *result)
{
    if (!result || bus >= BUS_ID_MAX) return VCI_ERR_PARAM;
    memset(result, 0, sizeof(*result));

    extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;
    extern UART_HandleTypeDef  huart2,  huart3;

    uint32_t listen_ms = s_cfg[bus].listen_ms ?
                         s_cfg[bus].listen_ms : 200U;

    switch (bus) {

    case BUS_ID_FDCAN1:
    case BUS_ID_FDCAN2: {
        FDCAN_HandleTypeDef *h = (bus == BUS_ID_FDCAN1) ? &hfdcan1 : &hfdcan2;

        for (uint8_t i = 0;
             i < sizeof(CAN_BAUDRATES)/sizeof(CAN_BAUDRATES[0]); i++) {

            uint32_t frames = 0;
            if (_probe_can_baudrate(h, CAN_BAUDRATES[i],
                                    &frames, listen_ms)) {
                result->detected         = true;
                result->baudrate_nominal = CAN_BAUDRATES[i];
                result->frame_count      = frames;

                if (s_cfg[bus].try_fd) {
                    for (uint8_t j = 0;
                         j < sizeof(CANFD_DATA_RATES)/sizeof(CANFD_DATA_RATES[0]);
                         j++) {
                        uint32_t fd_frames = 0;
                        if (_probe_canfd_data_rate(h,
                                CAN_BAUDRATES[i],
                                CANFD_DATA_RATES[j],
                                &fd_frames, listen_ms)) {
                            result->is_fd         = true;
                            result->baudrate_data = CANFD_DATA_RATES[j];
                            result->frame_count  += fd_frames;
                            break;
                        }
                    }
                }
                BusProbe_OnDetected(bus, result);
                return VCI_OK;
            }
        }
        break;
    }

    case BUS_ID_LIN1: {
        if (!s_cfg[bus].try_lin) break;
        uint32_t baud = 0;
        if (_probe_lin(&huart2, &baud)) {
            result->detected         = true;
            result->baudrate_nominal = baud;
            BusProbe_OnDetected(bus, result);
        }
        break;
    }

    case BUS_ID_KLINE: {
        if (!s_cfg[bus].try_kline) break;
        if (_probe_kline(&huart3)) {
            result->detected         = true;
            result->baudrate_nominal = KLINE_BAUDRATE_FAST;
            BusProbe_OnDetected(bus, result);
        }
        break;
    }

    default: break;
    }

    return result->detected ? VCI_OK : VCI_ERR_TIMEOUT;
}

/* ════════════════════════════════════════════════════════════════════════
 *  BusProbe_RunAll
 * ════════════════════════════════════════════════════════════════════════ */
VCI_Status_t BusProbe_RunAll(BusProbeResult_t results[BUS_ID_MAX])
{
    for (uint8_t i = 0; i < (uint8_t)BUS_ID_USB; i++) {
        BusProbe_Run((VCI_BusID_t)i, &results[i]);
        osDelay(50U);
    }
    return VCI_OK;
}

/* ════════════════════════════════════════════════════════════════════════
 *  BusProbe_OnDetected  (weak — override dans votre BSP si besoin)
 * ════════════════════════════════════════════════════════════════════════ */
__attribute__((weak))
void BusProbe_OnDetected(VCI_BusID_t bus, const BusProbeResult_t *r)
{
    BusMgr_SetBitrate(bus, r->baudrate_nominal,
                      r->is_fd ? r->baudrate_data : r->baudrate_nominal);
    BusMgr_StartBus(bus);
}

/* ════════════════════════════════════════════════════════════════════════
 *  BusProbe_Task
 * ════════════════════════════════════════════════════════════════════════ */
void BusProbe_Task(void *arg)
{
    (void)arg;

    /* Configuration par défaut pour chaque bus */
    BusProbe_Init(&(BusProbeCfg_t){
        .bus = BUS_ID_FDCAN1, .listen_ms = 200, .try_fd = true });
    BusProbe_Init(&(BusProbeCfg_t){
        .bus = BUS_ID_FDCAN2, .listen_ms = 200, .try_fd = true });
    BusProbe_Init(&(BusProbeCfg_t){
        .bus = BUS_ID_LIN1,   .listen_ms = 150, .try_lin = true });
    BusProbe_Init(&(BusProbeCfg_t){
        .bus = BUS_ID_KLINE,  .listen_ms = 300, .try_kline = true });

    /* ── Phase 1 : détection initiale au boot ──────────────────────── */
    BusProbeResult_t results[BUS_ID_MAX] = {0};
    BusProbe_RunAll(results);

    /* ── Phase 2 : surveillance continue ───────────────────────────── */
    for (;;) {
        uint32_t flags = osEventFlagsWait(g_evf_system,
                             EVF_BUS_OFF_FDCAN1 | EVF_BUS_OFF_FDCAN2,
                             osFlagsWaitAny,
                             5000U);   /* re-probe des bus inactifs toutes les 5s */

        if (flags & EVF_BUS_OFF_FDCAN1) {
            osDelay(500U);
            BusProbe_Run(BUS_ID_FDCAN1, &results[BUS_ID_FDCAN1]);
        }
        if (flags & EVF_BUS_OFF_FDCAN2) {
            osDelay(500U);
            BusProbe_Run(BUS_ID_FDCAN2, &results[BUS_ID_FDCAN2]);
        }

        /* Re-probe des bus non détectés */
        for (uint8_t i = 0; i < (uint8_t)BUS_ID_USB; i++) {
            if (!results[i].detected) {
                BusProbe_Run((VCI_BusID_t)i, &results[i]);
            }
        }
    }
}
