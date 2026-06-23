/*
 * ext_pos_uart.c - External position over UART2 (PA2/PA3), both directions!
 *
 *
 */

#include "ext_pos_uart.h"

#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart2.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"

#define EXT_POS_UART_BAUDRATE        115200
#define EXT_POS_UART_TASK_NAME       "extPosUart"
#define EXT_POS_UART_TASK_PRI        1
#define EXT_POS_UART_TASK_STACKSIZE  300  // words

#define PACKET_MAGIC_0    0xAB
#define PACKET_MAGIC_1    0xCD
#define PACKET_PAYLOAD_LEN 13  // validity(1) + x(4) + y(4) + z(4)
#define PACKET_TOTAL_LEN  16

// START command (CF -> OpenMV)
#define START_MAGIC_0     0x53
#define START_MAGIC_1     0x54
// we also feed position to start so we have a honest/fresh start
#define START_PAYLOAD_LEN 16  // x(4) + y(4) + z(4) + yaw(4)

// position measurement noise [m]. runtime-tunable via PARAM extPosUart.stdDev
static float extPosStdDev = 0.1f;

//   startVio      0->1 : send START (current pose) to OpenMV,
//   feedEstimator 0/1  : gate whether EKF positions enter the estimator.
static uint8_t startVio = 0;
static uint8_t feedEstimator = 0;

// ── Log variables ────────────────────────────────────────────────────────────
static float logX = 0.0f;
static float logY = 0.0f;
static float logZ = 0.0f;
static uint8_t logValid = 0;
static uint32_t logPktCount = 0;
static uint32_t logTimeouts = 0;
static uint32_t logMagicErrors = 0;
static uint32_t logBadChecksum = 0;
static uint32_t logStartSent = 0;  // 

// ── Task ─────────────────────────────────────────────────────────────────────
static void extPosUartTask(void *param);
STATIC_MEM_TASK_ALLOC(extPosUartTask, EXT_POS_UART_TASK_STACKSIZE);

void extPosUartInit(void)
{
  uart2Init(EXT_POS_UART_BAUDRATE);
  DEBUG_PRINT("extPosUart: uart2 initialized at %u baud\n", EXT_POS_UART_BAUDRATE);
  STATIC_MEM_TASK_CREATE(extPosUartTask, extPosUartTask,
                         EXT_POS_UART_TASK_NAME, NULL, EXT_POS_UART_TASK_PRI);
}

// Send the current CF estimated pose to the OpenMV so its EKF resets to it.
// yaw is extracted from the body->world rotation matrix
static void sendStartCommand(void)
{
  point_t pos;
  float R[9];  // row-major 3x3
  estimatorKalmanGetEstimatedPos(&pos);
  estimatorKalmanGetEstimatedRot(R);
  float yaw = atan2f(R[3], R[0]);  // R[1][0]=R[3], R[0][0]=R[0], [rad]

  uint8_t pkt[2 + START_PAYLOAD_LEN + 1];
  pkt[0] = START_MAGIC_0;
  pkt[1] = START_MAGIC_1;
  memcpy(&pkt[2],  &pos.x, sizeof(float));
  memcpy(&pkt[6],  &pos.y, sizeof(float));
  memcpy(&pkt[10], &pos.z, sizeof(float));
  memcpy(&pkt[14], &yaw,   sizeof(float));
  uint8_t chk = 0;
  for (int i = 2; i < 2 + START_PAYLOAD_LEN; i++) {
    chk ^= pkt[i];
  }
  pkt[2 + START_PAYLOAD_LEN] = chk;

  uart2SendData(sizeof(pkt), pkt);
  logStartSent++;
  DEBUG_PRINT("extPosUart: START sent pose=(%.2f,%.2f,%.2f) yaw=%.1fdeg\n",
              (double)pos.x, (double)pos.y, (double)pos.z,
              (double)(yaw * 57.29578f));
}

static void extPosUartTask(void *param)
{
  (void)param;

  uint8_t c;
  uint8_t buf[PACKET_PAYLOAD_LEN + 1]; 

  bool handshakeArmed = false;   // re-send until start streaming
  uint32_t pktsAtArm = 0;

  DEBUG_PRINT("extPosUart: task started\n");

  while (1) {
    // we keep sending START (current pose) until the openmv begins streaming position packets
    if (startVio) {
      handshakeArmed = true;
      pktsAtArm = logPktCount;
      startVio = 0;
    }
    if (handshakeArmed) {
      if (logPktCount > pktsAtArm) {
        handshakeArmed = false;   // OpenMV is up and streaming -> done
      } else {
        sendStartCommand();
      }
    }

    // wait for magic byte 0
    if (!uart2GetCharWithDefaultTimeout(&c)) {
      logTimeouts++;
      if ((logTimeouts & 0x0F) == 0) {
        DEBUG_PRINT("extPosUart: waiting for data, timeouts=%lu\n", (unsigned long)logTimeouts);
      }
      continue;
    }
    if (c != PACKET_MAGIC_0) {
      logMagicErrors++;
      if ((logMagicErrors & 0x0F) == 0) {
        DEBUG_PRINT("extPosUart: magic0 mismatch=%lu\n", (unsigned long)logMagicErrors);
      }
      continue;
    }

    // wait for magic byte 1
    if (!uart2GetCharWithDefaultTimeout(&c)) {
      logTimeouts++;
      if ((logTimeouts & 0x0F) == 0) {
        DEBUG_PRINT("extPosUart: waiting for second magic byte, timeouts=%lu\n", (unsigned long)logTimeouts);
      }
      continue;
    }
    if (c != PACKET_MAGIC_1) {
      logMagicErrors++;
      if ((logMagicErrors & 0x0F) == 0) {
        DEBUG_PRINT("extPosUart: magic1 mismatch=%lu\n", (unsigned long)logMagicErrors);
      }
      continue;
    }

    // ── Read payload + checksum ───────────────────────────────────────────
    // buf[0]   = validity
    // buf[1-4] = x float32 LE
    // buf[5-8] = y float32 LE
    // buf[9-12]= z float32 LE
    // buf[13]  = XOR checksum of buf[0..12]
    uart2GetData(sizeof(buf), buf);

    // verify checksum
    uint8_t chk = 0;
    for (int i = 0; i < PACKET_PAYLOAD_LEN; i++) {
      chk ^= buf[i];
    }
    if (chk != buf[PACKET_PAYLOAD_LEN]) {
      logBadChecksum++;
      if ((logBadChecksum & 0x0F) == 0) {
        DEBUG_PRINT("extPosUart: bad checksum=%lu\n", (unsigned long)logBadChecksum);
      }
      continue;  // bad packet
    }

    uint8_t validity = buf[0];
    logValid = validity;
    logPktCount++;
    // DEBUG_PRINT("extPosUart: packet %lu received validity=%u\n", (unsigned long)logPktCount, validity);

    if (validity != 0x01) {
      continue;  // EKF not converged yet, so skip
    }

    // deserialize x, y, z 
    float x, y, z;
    memcpy(&x, &buf[1], sizeof(float));
    memcpy(&y, &buf[5], sizeof(float));
    memcpy(&z, &buf[9], sizeof(float));

    // always log (so the PC can compare EKF vs OptiTrack during the dry-run)
    logX = x;
    logY = y;
    logZ = z;

    // insert into Kalman filter 
    if (feedEstimator) {
      positionMeasurement_t pos = {
        .x      = x,
        .y      = y,
        .z      = z,
        .stdDev = extPosStdDev,
        .source = MeasurementSourceLocationService,
      };
      estimatorEnqueuePosition(&pos);
    }
  }
}

// ── Log group ─────────────────────────────────────────────────────────────────
LOG_GROUP_START(extPosUart)
LOG_ADD(LOG_FLOAT, x,     &logX)
LOG_ADD(LOG_FLOAT, y,     &logY)
LOG_ADD(LOG_FLOAT, z,     &logZ)
LOG_ADD(LOG_UINT8, valid, &logValid)
LOG_ADD(LOG_UINT32, pkts, &logPktCount)
LOG_ADD(LOG_UINT32, timeouts, &logTimeouts)
LOG_ADD(LOG_UINT32, magicErr, &logMagicErrors)
LOG_ADD(LOG_UINT32, badCsum, &logBadChecksum)
LOG_ADD(LOG_UINT32, startSent, &logStartSent)
LOG_GROUP_STOP(extPosUart)

// ── Param group ─────────────────────────────────────────────────────────────
PARAM_GROUP_START(extPosUart)
/**
 * @brief Position measurement noise std-dev [m] for the external VIO position
 * update (mm_position). Lower = trust the OpenMV position more.
 */
PARAM_ADD(PARAM_FLOAT, stdDev, &extPosStdDev)
/**
 * @brief Set to 1 to send the current CF pose to the OpenMV as the EKF start
 * pose (one-shot; auto-clears). Triggers the VIO handoff.
 */
PARAM_ADD(PARAM_UINT8, startVio, &startVio)
/**
 * @brief Gate for feeding EKF positions into the estimator. 0 = dry-run
 * (log only, validate vs OptiTrack), 1 = feed (commit to VIO).
 */
PARAM_ADD(PARAM_UINT8, feedEstimator, &feedEstimator)
PARAM_GROUP_STOP(extPosUart)
