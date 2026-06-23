/*
 * ext_pos_uart.h - External position receiver using UART2 (PA2/PA3)
 *
 * Receives 16-byte binary position packets from the OpenMV N6 and put
 * them into the Kalman estimator via estimatorEnqueuePosition(
 *
 * Packet format (16 bytes):
 *   Byte  0-1 : Magic  0xAB 0xCD
 *   Byte  2   : Validity  0x01 = valid (Phase 2 VIO), 0x00 = invalid
 *   Byte  3-6 : x  float32  [m]
 *   Byte  7-10: y  float32  [m]
 *   Byte 11-14: z  float32  [m]
 *   Byte 15   : XOR checksum of bytes 3-14
 *
 * Wiring: 
 *   OpenMV P14 (UART7 TX)  ->  Bolt PA3 (UART2 RX)
 *   OpenMV P13 (UART7 RX)  <-  Bolt PA2 (UART2 TX)
 *   OpenMV GND              -  Bolt STM GND
 */
#pragma once

/**
 * Initialize the UART2 and start the receiver task.
 * 
 */
void extPosUartInit(void);
