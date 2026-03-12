# VCI Gateway Firmware — STM32H723VGT6
## Architecture & Developer Guide

---

## 1. Overview

Professional automotive VCI (Vehicle Communication Interface) gateway firmware for STM32H723VGT6 running FreeRTOS / CMSIS-OS2 at 550 MHz.

```
PC Software (J2534 / vendor tool)
         │
    USB CDC (USB_OTG_HS • 480 Mbit/s)
         │
┌────────▼────────────────────────────────────────────────┐
│                   STM32H723VGT6                          │
│  ┌──────────┐  ┌───────────┐  ┌────────┐  ┌──────────┐ │
│  │Task_Diag │  │Task_ISOTP │  │Task_GW │  │Task_USB  │ │
│  │  (UDS)   │  │(ISO15765) │  │(Router)│  │  (CDC)   │ │
│  └────┬─────┘  └─────┬─────┘  └───┬────┘  └──────────┘ │
│       └──────────────▼────────────┘                      │
│                Bus Manager (abstraction)                  │
│       ┌──────────┬──────────┬────────┬────────────────┐  │
│    FDCAN1      FDCAN2      LIN    K-Line   ETH(DoIP)   │  │
│   (HS-CAN)  (FD-CAN)  (TJA1021) (L9637)  (LAN8742A)  │  │
└──────────────────────────────────────────────────────────┘
    TJA1044   TCAN4550

