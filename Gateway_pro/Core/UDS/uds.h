/**
 ******************************************************************************
 * @file    uds.h
 * @brief   VCI Gateway — UDS (ISO 14229) Tester / Client
 ******************************************************************************
 *  This module implements the UDS CLIENT (tester) side.
 *  The VCI acts as tester towards ECUs, forwarding requests from the PC.
 *
 *  Supported services (SID):
 *   0x10  DiagnosticSessionControl
 *   0x11  ECUReset
 *   0x14  ClearDiagnosticInformation
 *   0x19  ReadDTCInformation
 *   0x22  ReadDataByIdentifier
 *   0x23  ReadMemoryByAddress
 *   0x27  SecurityAccess
 *   0x28  CommunicationControl
 *   0x2E  WriteDataByIdentifier
 *   0x2F  InputOutputControlByIdentifier
 *   0x31  RoutineControl
 *   0x34  RequestDownload
 *   0x35  RequestUpload
 *   0x36  TransferData
 *   0x37  RequestTransferExit
 *   0x3E  TesterPresent
 */

#ifndef UDS_H
#define UDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vci_config.h"
#include "isotp.h"

/* ── Service IDs ──────────────────────────────────────────────────────── */
typedef enum {
    UDS_SID_DSC   = 0x10,
    UDS_SID_ER    = 0x11,
    UDS_SID_CDTCI = 0x14,
    UDS_SID_RDTCI = 0x19,
    UDS_SID_RDBI  = 0x22,
    UDS_SID_RMBA  = 0x23,
    UDS_SID_SA    = 0x27,
    UDS_SID_CC    = 0x28,
    UDS_SID_WDBI  = 0x2E,
    UDS_SID_IOCBI = 0x2F,
    UDS_SID_RC    = 0x31,
    UDS_SID_RD    = 0x34,
    UDS_SID_RU    = 0x35,
    UDS_SID_TD    = 0x36,
    UDS_SID_RTE   = 0x37,
    UDS_SID_TP    = 0x3E,
    UDS_SID_RESP  = 0x50,  /* Positive response offset (SID + 0x40)     */
    UDS_SID_NRC   = 0x7F   /* Negative response                         */
} UDS_ServiceID_t;

/* ── Session types ────────────────────────────────────────────────────── */
typedef enum {
    UDS_SESSION_DEFAULT    = 0x01,
    UDS_SESSION_PROGRAMMING= 0x02,
    UDS_SESSION_EXTENDED   = 0x03
} UDS_Session_t;

/* ── NRC codes ────────────────────────────────────────────────────────── */
typedef enum {
    UDS_NRC_PR    = 0x00,  /* Positive response (not a real NRC)         */
    UDS_NRC_GR    = 0x10,  /* generalReject                              */
    UDS_NRC_SNS   = 0x11,  /* serviceNotSupported                        */
    UDS_NRC_SFNS  = 0x12,  /* subFunctionNotSupported                    */
    UDS_NRC_IMLOIF= 0x13,  /* incorrectMessageLengthOrInvalidFormat      */
    UDS_NRC_RTL   = 0x14,  /* responseTooLong                            */
    UDS_NRC_BRR   = 0x21,  /* busyRepeatRequest                          */
    UDS_NRC_CNC   = 0x22,  /* conditionsNotCorrect                       */
    UDS_NRC_RSE   = 0x24,  /* requestSequenceError                       */
    UDS_NRC_NRFSC = 0x25,  /* noResponseFromSubnetComponent             */
    UDS_NRC_FPEORA= 0x26,  /* failurePreventsExecutionOfRequestedAction  */
    UDS_NRC_ROOR  = 0x31,  /* requestOutOfRange                          */
    UDS_NRC_SAD   = 0x33,  /* securityAccessDenied                       */
    UDS_NRC_AR    = 0x34,  /* authenticationRequired                     */
    UDS_NRC_IK    = 0x35,  /* invalidKey                                 */
    UDS_NRC_ENOA  = 0x36,  /* exceededNumberOfAttempts                   */
    UDS_NRC_RTDNE = 0x37,  /* requiredTimeDelayNotExpired                */
    UDS_NRC_UDNA  = 0x70,  /* uploadDownloadNotAccepted                  */
    UDS_NRC_TDS   = 0x71,  /* transferDataSuspended                      */
    UDS_NRC_GPF   = 0x72,  /* generalProgrammingFailure                  */
    UDS_NRC_WBSC  = 0x73,  /* wrongBlockSequenceCounter                  */
    UDS_NRC_RCRRP = 0x78,  /* requestCorrectlyReceivedResponsePending    */
    UDS_NRC_SFNSIAS=0x7E,  /* subFunctionNotSupportedInActiveSession      */
    UDS_NRC_SNSIAS = 0x7F  /* serviceNotSupportedInActiveSession         */
} UDS_NRC_t;

/* ── Request / Response ───────────────────────────────────────────────── */
typedef struct {
    UDS_ServiceID_t  sid;
    uint8_t          sub_func;      /* Sub-function (0xFF = not used)    */
    uint16_t         data_id;       /* DID / memory address lo           */
    uint8_t          data[ISOTP_BUFFER_SIZE];
    uint32_t         data_len;
    uint32_t         target_addr;   /* ECU physical CAN ID               */
    VCI_BusID_t      bus;
} UDS_Request_t;

typedef struct {
    UDS_ServiceID_t  sid;
    bool             positive;
    UDS_NRC_t        nrc;           /* Valid if !positive                */
    uint8_t          data[ISOTP_BUFFER_SIZE];
    uint32_t         data_len;
    uint32_t         elapsed_ms;    /* Round-trip time                   */
} UDS_Response_t;

/* ── Client context ───────────────────────────────────────────────────── */
typedef struct {
    ISOTP_Channel_t  isotp;         /* Underlying ISO-TP channel         */
    UDS_Session_t    session;
    bool             tp_running;    /* TesterPresent auto-send active    */
    uint32_t         p2_timeout_ms;
    uint32_t         p2ext_timeout_ms;

    /* Pending response buffer */
    volatile bool    resp_ready;
    uint8_t          resp_buf[ISOTP_BUFFER_SIZE];
    uint32_t         resp_len;
} UDS_Client_t;

/* ── Public API ───────────────────────────────────────────────────────── */
VCI_Status_t UDS_ClientInit    (UDS_Client_t *client,
                                 VCI_BusID_t bus,
                                 uint32_t tx_id, uint32_t rx_id,
                                 bool ext_id, bool fd_mode);

VCI_Status_t UDS_Request       (UDS_Client_t *client,
                                 const UDS_Request_t  *req,
                                 UDS_Response_t       *resp);

/* Convenience wrappers */
VCI_Status_t UDS_DiagSession   (UDS_Client_t *c, UDS_Session_t sess,
                                 UDS_Response_t *resp);
VCI_Status_t UDS_ECUReset      (UDS_Client_t *c, uint8_t reset_type,
                                 UDS_Response_t *resp);
VCI_Status_t UDS_ReadDID       (UDS_Client_t *c, uint16_t did,
                                 uint8_t *out, uint32_t *out_len);
VCI_Status_t UDS_WriteDID      (UDS_Client_t *c, uint16_t did,
                                 const uint8_t *data, uint32_t len);
VCI_Status_t UDS_SecurityAccess(UDS_Client_t *c, uint8_t level,
                                 const uint8_t *seed_fn_ctx,
                                 UDS_Response_t *resp);
VCI_Status_t UDS_TesterPresent (UDS_Client_t *c);
VCI_Status_t UDS_ClearDTC      (UDS_Client_t *c, uint32_t group,
                                 UDS_Response_t *resp);

/* TesterPresent auto-sender — call from periodic task */
void         UDS_Tick          (UDS_Client_t *c, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* UDS_H */
