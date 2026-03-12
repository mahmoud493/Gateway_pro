/**
 ******************************************************************************
 * @file    vci_config.h
 * @brief   VCI Gateway — Global configuration & type definitions
 * @target  STM32H723VGTx @ 550 MHz  (Cortex-M7, FDCAN, ETH, USB-HS)
 * @rtos    FreeRTOS / CMSIS-OS2
 ******************************************************************************
 *  Architecture overview
 * ──────────────────────────────────────────────────────────────────────────
 *  PC Software
 *     │  USB CDC (USB_OTG_HS)
 *  ┌──▼──────────────────────────────────────────────────────────────────┐
 *  │                    STM32H723  FreeRTOS / CMSIS-OS2                 │
 *  │  ┌──────────┐  ┌──────────┐  ┌────────┐  ┌───────┐  ┌──────────┐ │
 *  │  │ USB Host │  │ Gateway  │  │ISO-TP  │  │  UDS  │  │  Logger  │ │
 *  │  │  (CDC)   │  │  Core    │  │ISO15765│  │ISO14229│  │  DoIP    │ │
 *  │  └────┬─────┘  └────┬─────┘  └───┬────┘  └───┬───┘  └──────────┘ │
 *  │       │             │            │            │                    │
 *  │  ┌────▼─────────────▼────────────▼────────────▼──────────────────┐│
 *  │  │                    Bus Manager (abstraction)                   ││
 *  │  └───┬──────────┬──────────┬──────────┬──────────────────────────┘│
 *  │      │          │          │          │                            │
 *  │  FDCAN1(HS) FDCAN2(FD)  UART(LIN) UART(KLINE)  ETH(DoIP)          │
 *  └──────────────────────────────────────────────────────────────────-─┘
 *     TJA1044  TCAN4550    TJA1021    L9637     LAN8742A
 */

#ifndef VCI_CONFIG_H
#define VCI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "cmsis_os2.h"

/* ── Firmware version ─────────────────────────────────────────────────── */
#define VCI_FW_VERSION_MAJOR    1U
#define VCI_FW_VERSION_MINOR    0U
#define VCI_FW_VERSION_PATCH    0U
#define VCI_FW_VERSION_STR      "1.0.0"

/* ── System clock ─────────────────────────────────────────────────────── */
#define VCI_SYSCLK_HZ           550000000UL   /* 550 MHz — max for H723  */
#define VCI_APB1_HZ             137500000UL
#define VCI_APB2_HZ             275000000UL

/* ── Bus identifiers ──────────────────────────────────────────────────── */
typedef enum {
    BUS_ID_FDCAN1  = 0x00,   /* HS-CAN  — Classical CAN 1/2/5 Mbit/s   */
    BUS_ID_FDCAN2  = 0x01,   /* FD-CAN  — CAN-FD up to 5/8 Mbit/s      */
    BUS_ID_LIN1    = 0x02,   /* LIN     — UART + TJA1021                */
    BUS_ID_KLINE   = 0x03,   /* K-Line  — UART + L9637                  */
    BUS_ID_ETH     = 0x04,   /* DoIP    — LAN8742A                      */
    BUS_ID_USB     = 0x05,   /* USB CDC — OTG_HS to PC                  */
    BUS_ID_MAX     = 0x06
} VCI_BusID_t;

/* ── Bus state ────────────────────────────────────────────────────────── */
typedef enum {
    BUS_STATE_UNINIT  = 0,
    BUS_STATE_IDLE,
    BUS_STATE_ACTIVE,
    BUS_STATE_BUSOFF,
    BUS_STATE_ERROR,
    BUS_STATE_SLEEP
} VCI_BusState_t;

/* ── Generic VCI status ───────────────────────────────────────────────── */
typedef enum {
    VCI_OK          =  0,
    VCI_ERR_PARAM   = -1,
    VCI_ERR_TIMEOUT = -2,
    VCI_ERR_BUSY    = -3,
    VCI_ERR_OVFLOW  = -4,
    VCI_ERR_HW      = -5,
    VCI_ERR_UNINIT  = -6,
    VCI_ERR_PROTO   = -7,
    VCI_ERR_NOACK   = -8
} VCI_Status_t;

/* ── CAN frame ────────────────────────────────────────────────────────── */
#define CAN_MAX_DLC             8U
#define CANFD_MAX_DLC           64U

typedef struct {
    uint32_t        id;          /* 11-bit or 29-bit (IDE flag in flags) */
    uint8_t         dlc;
    uint8_t         flags;       /* bit0=FD, bit1=BRS, bit2=ESI, bit3=IDE */
    uint8_t         data[CANFD_MAX_DLC];
    uint32_t        timestamp_us;
    VCI_BusID_t     bus;
} VCI_CanFrame_t;

#define CAN_FLAG_FD             (1U << 0)
#define CAN_FLAG_BRS            (1U << 1)
#define CAN_FLAG_ESI            (1U << 2)
#define CAN_FLAG_IDE            (1U << 3)   /* Extended (29-bit) ID        */

/* ── LIN frame ────────────────────────────────────────────────────────── */
#define LIN_MAX_DATA            8U

typedef struct {
    uint8_t         pid;         /* Protected identifier (6-bit ID + 2 parity) */
    uint8_t         data[LIN_MAX_DATA];
    uint8_t         len;
    uint8_t         checksum;
    bool            enhanced_checksum;
    uint32_t        timestamp_us;
} VCI_LinFrame_t;

/* ── K-Line / ISO 9141 frame ─────────────────────────────────────────── */
#define KLINE_MAX_DATA          255U

typedef struct {
    uint8_t         data[KLINE_MAX_DATA];
    uint16_t        len;
    uint32_t        timestamp_us;
} VCI_KLineFrame_t;

/* ── Generic bus message (union envelope) ────────────────────────────── */
typedef enum {
    MSG_TYPE_CAN = 0,
    MSG_TYPE_LIN,
    MSG_TYPE_KLINE,
    MSG_TYPE_ISOTP,
    MSG_TYPE_UDS,
    MSG_TYPE_DOIP,
    MSG_TYPE_CTRL   /* Internal control message                          */
} VCI_MsgType_t;

typedef struct {
    VCI_MsgType_t   type;
    VCI_BusID_t     src_bus;
    VCI_BusID_t     dst_bus;
    uint32_t        timestamp_us;
    union {
        VCI_CanFrame_t   can;
        VCI_LinFrame_t   lin;
        VCI_KLineFrame_t kline;
        uint8_t          raw[256];
    } payload;
    uint16_t        payload_len;
} VCI_Message_t;

/* ── RTOS task priorities ─────────────────────────────────────────────── */
/* Lower number = lower priority in CMSIS-OS2  (osPriorityXxx)            */
#define PRIO_LOGGER          osPriorityLow
#define PRIO_POWER_MON       osPriorityBelowNormal
#define PRIO_USB_HOST        osPriorityNormal
#define PRIO_KLINE           osPriorityNormal
#define PRIO_LIN             osPriorityNormal
#define PRIO_GATEWAY         osPriorityAboveNormal
#define PRIO_ISOTP           osPriorityAboveNormal
#define PRIO_FDCAN1          osPriorityHigh
#define PRIO_FDCAN2          osPriorityHigh
#define PRIO_DIAG            osPriorityHigh
#define PRIO_ETH_DOIP        osPriorityRealtime

/* ── RTOS task stack sizes (words) ───────────────────────────────────── */
#define STACK_LOGGER         256U
#define STACK_POWER          256U
#define STACK_USB_HOST       512U
#define STACK_KLINE          256U
#define STACK_LIN            256U
#define STACK_GATEWAY        512U
#define STACK_ISOTP          512U
#define STACK_FDCAN          512U
#define STACK_DIAG           768U
#define STACK_ETH_DOIP       1024U

/* ── Queue depths ─────────────────────────────────────────────────────── */
#define QUEUE_DEPTH_CAN      32U
#define QUEUE_DEPTH_LIN      16U
#define QUEUE_DEPTH_KLINE    16U
#define QUEUE_DEPTH_GATEWAY  64U
#define QUEUE_DEPTH_USB      32U
#define QUEUE_DEPTH_LOG      64U

/* ── ISO-TP / UDS config ─────────────────────────────────────────────── */
#define ISOTP_BUFFER_SIZE    4096U  /* Max UDS payload                   */
#define ISOTP_TIMEOUT_MS     250U
#define ISOTP_STMIN_MS       1U
#define ISOTP_BS             0U     /* Block size 0 = no flow control     */

#define UDS_REQ_ID           0x7DFU /* Functional addressing             */
#define UDS_RESP_ID_BASE     0x7E0U /* Physical 7E0..7E7                 */
#define UDS_P2_TIMEOUT_MS    50U
#define UDS_P2EXT_TIMEOUT_MS 5000U

/* ── DoIP config ──────────────────────────────────────────────────────── */
#define DOIP_UDP_DISC_PORT   13400U
#define DOIP_TCP_PORT        13400U
#define DOIP_ENTITY_ADDR     0x0001U
#define DOIP_LOGICAL_ADDR    0xE400U

/* ── USB CDC config ───────────────────────────────────────────────────── */
#define USB_CDC_BAUD         3000000U   /* 3 Mbit/s virtual COM          */
#define USB_CDC_RX_BUF       2048U
#define USB_CDC_TX_BUF       2048U

/* ── LIN config ───────────────────────────────────────────────────────── */
#define LIN_BAUDRATE         19200U
#define LIN_BREAK_BITS       13U

/* ── K-Line config ────────────────────────────────────────────────────── */
#define KLINE_BAUDRATE_5BAUD 5U
#define KLINE_BAUDRATE_FAST  10400U

/* ── Debug / logging ─────────────────────────────────────────────────── */
#define VCI_LOG_ENABLE       1U
#define VCI_LOG_TIMESTAMP    1U
#define VCI_LOG_BUS_TRACE    1U   /* Log all raw bus frames              */

#ifdef __cplusplus
}
#endif

#endif /* VCI_CONFIG_H */
