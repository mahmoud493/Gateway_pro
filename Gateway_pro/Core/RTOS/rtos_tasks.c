/**
 ******************************************************************************
 * @file    rtos_tasks.c
 * @brief   VCI Gateway — RTOS object creation & task launch
 ******************************************************************************
 *  Call sequence (from main.c):
 *    1. HAL_Init() + SystemClock_Config()
 *    2. MX_*_Init()  (CubeMX peripherals)
 *    3. RTOS_CreateAllObjects()   ← create queues, mutexes, semaphores
 *    4. RTOS_LaunchAllTasks()     ← spawn all threads
 *    5. osKernelStart()
 */

#include "rtos_tasks.h"
#include "bus_probe.h"    /* BusProbe_Task entry point                  */
#include <string.h>

/* ── Handle storage ───────────────────────────────────────────────────── */
osThreadId_t g_task_fdcan1;
osThreadId_t g_task_fdcan2;
osThreadId_t g_task_lin;
osThreadId_t g_task_kline;
osThreadId_t g_task_gateway;
osThreadId_t g_task_isotp;
osThreadId_t g_task_diag;
osThreadId_t g_task_usb_host;
osThreadId_t g_task_eth_doip;
osThreadId_t g_task_logger;
osThreadId_t g_task_power;
osThreadId_t g_task_probe;

osMessageQueueId_t g_q_fdcan1_rx;
osMessageQueueId_t g_q_fdcan2_rx;
osMessageQueueId_t g_q_lin_rx;
osMessageQueueId_t g_q_kline_rx;
osMessageQueueId_t g_q_eth_rx;
osMessageQueueId_t g_q_usb_rx;
osMessageQueueId_t g_q_gateway_out;
osMessageQueueId_t g_q_isotp_rx;
osMessageQueueId_t g_q_diag_req;
osMessageQueueId_t g_q_diag_resp;
osMessageQueueId_t g_q_log;

osMutexId_t  g_mtx_fdcan1;
osMutexId_t  g_mtx_fdcan2;
osMutexId_t  g_mtx_lin;
osMutexId_t  g_mtx_kline;
osMutexId_t  g_mtx_usb_tx;
osMutexId_t  g_mtx_eth_tx;

osSemaphoreId_t  g_sem_usb_ready;
osSemaphoreId_t  g_sem_eth_link;
osEventFlagsId_t g_evf_system;

/* ── Helpers ──────────────────────────────────────────────────────────── */
#define ASSERT_HANDLE(h)  if ((h) == NULL) { return VCI_ERR_HW; }

/* ═════════════════════════════════════════════════════════════════════════
 *  RTOS_CreateAllObjects
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t RTOS_CreateAllObjects(void)
{
    /* ── Queues ─────────────────────────────────────────────────────── */
    g_q_fdcan1_rx   = osMessageQueueNew(QUEUE_DEPTH_CAN,     sizeof(VCI_Message_t), NULL);
    g_q_fdcan2_rx   = osMessageQueueNew(QUEUE_DEPTH_CAN,     sizeof(VCI_Message_t), NULL);
    g_q_lin_rx      = osMessageQueueNew(QUEUE_DEPTH_LIN,     sizeof(VCI_Message_t), NULL);
    g_q_kline_rx    = osMessageQueueNew(QUEUE_DEPTH_KLINE,   sizeof(VCI_Message_t), NULL);
    g_q_eth_rx      = osMessageQueueNew(QUEUE_DEPTH_CAN,     sizeof(VCI_Message_t), NULL);
    g_q_usb_rx      = osMessageQueueNew(QUEUE_DEPTH_USB,     sizeof(VCI_Message_t), NULL);
    g_q_gateway_out = osMessageQueueNew(QUEUE_DEPTH_GATEWAY, sizeof(VCI_Message_t), NULL);
    g_q_isotp_rx    = osMessageQueueNew(16U,                 sizeof(VCI_Message_t), NULL);
    g_q_diag_req    = osMessageQueueNew(8U,                  sizeof(VCI_Message_t), NULL);
    g_q_diag_resp   = osMessageQueueNew(8U,                  sizeof(VCI_Message_t), NULL);
    g_q_log         = osMessageQueueNew(QUEUE_DEPTH_LOG,     256U,                  NULL);

    ASSERT_HANDLE(g_q_fdcan1_rx);
    ASSERT_HANDLE(g_q_fdcan2_rx);
    ASSERT_HANDLE(g_q_lin_rx);
    ASSERT_HANDLE(g_q_kline_rx);
    ASSERT_HANDLE(g_q_eth_rx);
    ASSERT_HANDLE(g_q_usb_rx);
    ASSERT_HANDLE(g_q_gateway_out);
    ASSERT_HANDLE(g_q_isotp_rx);
    ASSERT_HANDLE(g_q_diag_req);
    ASSERT_HANDLE(g_q_diag_resp);
    ASSERT_HANDLE(g_q_log);

    /* ── Mutexes ─────────────────────────────────────────────────────── */
    static const osMutexAttr_t mtx_attr = {
        .attr_bits = osMutexPrioInherit | osMutexRobust
    };
    g_mtx_fdcan1  = osMutexNew(&mtx_attr);
    g_mtx_fdcan2  = osMutexNew(&mtx_attr);
    g_mtx_lin     = osMutexNew(&mtx_attr);
    g_mtx_kline   = osMutexNew(&mtx_attr);
    g_mtx_usb_tx  = osMutexNew(&mtx_attr);
    g_mtx_eth_tx  = osMutexNew(&mtx_attr);

    ASSERT_HANDLE(g_mtx_fdcan1);
    ASSERT_HANDLE(g_mtx_fdcan2);
    ASSERT_HANDLE(g_mtx_lin);
    ASSERT_HANDLE(g_mtx_kline);
    ASSERT_HANDLE(g_mtx_usb_tx);
    ASSERT_HANDLE(g_mtx_eth_tx);

    /* ── Semaphores ──────────────────────────────────────────────────── */
    g_sem_usb_ready = osSemaphoreNew(1U, 0U, NULL); /* starts locked      */
    g_sem_eth_link  = osSemaphoreNew(1U, 0U, NULL);

    ASSERT_HANDLE(g_sem_usb_ready);
    ASSERT_HANDLE(g_sem_eth_link);

    /* ── Event flags ─────────────────────────────────────────────────── */
    g_evf_system = osEventFlagsNew(NULL);
    ASSERT_HANDLE(g_evf_system);

    return VCI_OK;
}

/* ═════════════════════════════════════════════════════════════════════════
 *  RTOS_LaunchAllTasks
 * ═════════════════════════════════════════════════════════════════════════ */
VCI_Status_t RTOS_LaunchAllTasks(void)
{
    /*
     * NOTE: Le champ "name" de osThreadAttr_t ne peut PAS être utilisé
     * dans un initialiseur composé { .name = ... } car "name" est une
     * macro définie dans les headers FreeRTOS/CMSIS (task.h / portmacro.h).
     * Solution : memset + affectation explicite membre par membre.
     */
#define MAKE_TASK(task_name, entry, stack_words, prio, handle)  \
    do {                                                         \
        osThreadAttr_t _attr;                                    \
        memset(&_attr, 0, sizeof(_attr));                        \
        _attr.name       = (task_name);                          \
        _attr.stack_size = (uint32_t)(stack_words) * sizeof(uint32_t); \
        _attr.priority   = (prio);                               \
        (handle) = osThreadNew((entry), NULL, &_attr);           \
        ASSERT_HANDLE(handle);                                   \
    } while (0)

    /* Priorités croissantes : logger en premier, ETH en dernier         */
    MAKE_TASK("Logger",   Task_Logger,   STACK_LOGGER,   PRIO_LOGGER,          g_task_logger);
    MAKE_TASK("Probe",    BusProbe_Task, 768U,           osPriorityBelowNormal, g_task_probe);
    MAKE_TASK("Power",    Task_Power,    STACK_POWER,    PRIO_POWER_MON,        g_task_power);
    MAKE_TASK("USB_Host", Task_USBHost,  STACK_USB_HOST, PRIO_USB_HOST,         g_task_usb_host);
    MAKE_TASK("KLine",    Task_KLine,    STACK_KLINE,    PRIO_KLINE,            g_task_kline);
    MAKE_TASK("LIN",      Task_LIN,      STACK_LIN,      PRIO_LIN,              g_task_lin);
    MAKE_TASK("ISOTP",    Task_ISOTP,    STACK_ISOTP,    PRIO_ISOTP,            g_task_isotp);
    MAKE_TASK("Gateway",  Task_Gateway,  STACK_GATEWAY,  PRIO_GATEWAY,          g_task_gateway);
    MAKE_TASK("Diag",     Task_Diag,     STACK_DIAG,     PRIO_DIAG,             g_task_diag);
    MAKE_TASK("FDCAN1",   Task_FDCAN1,   STACK_FDCAN,    PRIO_FDCAN1,           g_task_fdcan1);
    MAKE_TASK("FDCAN2",   Task_FDCAN2,   STACK_FDCAN,    PRIO_FDCAN2,           g_task_fdcan2);
    MAKE_TASK("ETH_DoIP", Task_ETH_DoIP, STACK_ETH_DOIP, PRIO_ETH_DOIP,        g_task_eth_doip);

#undef MAKE_TASK
    return VCI_OK;
}
