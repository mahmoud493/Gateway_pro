/**
 ******************************************************************************
 * @file    rtos_tasks.h
 * @brief   VCI Gateway — RTOS task registry & inter-task communication
 ******************************************************************************
 *  All tasks, queues, semaphores and mutexes are declared here so every
 *  module can share handles without circular includes.
 */

#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "vci_config.h"

/* ── Task handles ─────────────────────────────────────────────────────── */
extern osThreadId_t g_task_fdcan1;
extern osThreadId_t g_task_fdcan2;
extern osThreadId_t g_task_lin;
extern osThreadId_t g_task_kline;
extern osThreadId_t g_task_gateway;
extern osThreadId_t g_task_isotp;
extern osThreadId_t g_task_diag;
extern osThreadId_t g_task_usb_host;
extern osThreadId_t g_task_eth_doip;
extern osThreadId_t g_task_logger;
extern osThreadId_t g_task_power;
extern osThreadId_t g_task_probe;   /* Bus auto-detection               */

/* ── Message queues ───────────────────────────────────────────────────── */
/*  Naming: q_<source>_to_<destination>                                    */
extern osMessageQueueId_t g_q_fdcan1_rx;     /* FDCAN1 RX → Gateway      */
extern osMessageQueueId_t g_q_fdcan2_rx;     /* FDCAN2 RX → Gateway      */
extern osMessageQueueId_t g_q_lin_rx;        /* LIN RX   → Gateway       */
extern osMessageQueueId_t g_q_kline_rx;      /* KLINE RX → Gateway       */
extern osMessageQueueId_t g_q_eth_rx;        /* ETH RX   → DoIP layer    */
extern osMessageQueueId_t g_q_usb_rx;        /* USB RX   → Gateway       */

extern osMessageQueueId_t g_q_gateway_out;   /* Gateway → TX dispatchers */
extern osMessageQueueId_t g_q_isotp_rx;      /* ISO-TP reassembled PDUs  */
extern osMessageQueueId_t g_q_diag_req;      /* UDS requests             */
extern osMessageQueueId_t g_q_diag_resp;     /* UDS responses            */
extern osMessageQueueId_t g_q_log;           /* Log entries              */

/* ── Mutexes ──────────────────────────────────────────────────────────── */
extern osMutexId_t  g_mtx_fdcan1;           /* FDCAN1 TX serialization   */
extern osMutexId_t  g_mtx_fdcan2;           /* FDCAN2 TX serialization   */
extern osMutexId_t  g_mtx_lin;
extern osMutexId_t  g_mtx_kline;
extern osMutexId_t  g_mtx_usb_tx;
extern osMutexId_t  g_mtx_eth_tx;

/* ── Semaphores / events ──────────────────────────────────────────────── */
extern osSemaphoreId_t g_sem_usb_ready;      /* USB enumeration done     */
extern osSemaphoreId_t g_sem_eth_link;       /* Ethernet link up         */
extern osEventFlagsId_t g_evf_system;

/* ── System event flags ───────────────────────────────────────────────── */
#define EVF_BUS_ERROR_FDCAN1   (1U << 0)
#define EVF_BUS_ERROR_FDCAN2   (1U << 1)
#define EVF_BUS_OFF_FDCAN1     (1U << 2)
#define EVF_BUS_OFF_FDCAN2     (1U << 3)
#define EVF_USB_CONNECT        (1U << 4)
#define EVF_USB_DISCONNECT     (1U << 5)
#define EVF_ETH_LINK_UP        (1U << 6)
#define EVF_ETH_LINK_DOWN      (1U << 7)
#define EVF_POWER_LOW          (1U << 8)
#define EVF_POWER_WAKE         (1U << 9)
#define EVF_DIAG_SESSION       (1U << 10)
#define EVF_SHUTDOWN_REQUEST   (1U << 11)
#define EVF_PROBE_DONE         (1U << 12)  /* BusProbe_Task terminée    */

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t RTOS_CreateAllObjects(void);
VCI_Status_t RTOS_LaunchAllTasks(void);

/* Task entry points (defined in respective modules) */
void Task_FDCAN1    (void *arg);
void Task_FDCAN2    (void *arg);
void Task_LIN       (void *arg);
void Task_KLine     (void *arg);
void Task_Gateway   (void *arg);
void Task_ISOTP     (void *arg);
void Task_Diag      (void *arg);
void Task_USBHost   (void *arg);
void Task_ETH_DoIP  (void *arg);
void Task_Logger    (void *arg);
void Task_Power     (void *arg);
void BusProbe_Task  (void *arg);   /* auto-detect — dans bus_probe.c   */

#ifdef __cplusplus
}
#endif

#endif /* RTOS_TASKS_H */
