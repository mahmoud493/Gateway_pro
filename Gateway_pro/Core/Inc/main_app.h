/**
 ******************************************************************************
 * @file    main_app.h
 * @brief   VCI Gateway — Application entry point declarations
 ******************************************************************************
 *  Include this header in CubeMX-generated main.c to hook the application:
 *
 *    #include "main_app.h"
 *    // ... inside main(), after all MX_xxx_Init() calls:
 *    VCI_AppMain();   // never returns
 */

#ifndef MAIN_APP_H
#define MAIN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "vci_config.h"

/* ── Application entry point ──────────────────────────────────────────── */
/**
 * @brief  Main application entry — initialises RTOS objects, launches all
 *         tasks, then calls osKernelStart() (never returns).
 *         Call this from CubeMX main.c after all MX_xxx_Init() are done.
 */
void VCI_AppMain(void);

/* ── Board-specific hooks (weak — override in board BSP if needed) ────── */
/**
 * @brief  Called before RTOS objects are created.
 *         Use for LED init, debug UART banner, power rail sequencing.
 */
void VCI_BoardEarlyInit(void);

/**
 * @brief  Called after all tasks are created, just before osKernelStart().
 *         Use for any last-minute board configuration.
 */
void VCI_BoardLateInit(void);

/* ── USB CDC callbacks (call from usbd_cdc_if.c overrides) ───────────── */
/**
 * @brief  Invoked by the USB CDC middleware when the PC sends data.
 *         Runs in USB interrupt context — must not block.
 * @param  buf  Pointer to received data buffer
 * @param  len  Number of bytes received
 */
void VCI_USB_CDC_RxCallback(uint8_t *buf, uint32_t len);

/**
 * @brief  Invoked when the USB CDC interface is connected/configured.
 *         Releases g_sem_usb_ready so dependent tasks can proceed.
 */
void VCI_USB_CDC_ConnectCallback(void);

/**
 * @brief  Invoked when the USB CDC interface is disconnected.
 */
void VCI_USB_CDC_DisconnectCallback(void);

/* ── Ethernet link callback ───────────────────────────────────────────── */
/**
 * @brief  Call from the lwIP / ETH driver when link state changes.
 * @param  link_up  true = link established, false = link lost
 */
void VCI_ETH_LinkCallback(bool link_up);

/* ── Microsecond timestamp ────────────────────────────────────────────── */
/**
 * @brief  Returns a free-running 32-bit microsecond counter.
 *         Driven by TIM2 configured at 1 MHz (prescaler = SYS_CLK/1e6 - 1).
 * @return Timestamp in microseconds (wraps every ~71 minutes)
 */
uint32_t VCI_GetTimestampUs(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_APP_H */
