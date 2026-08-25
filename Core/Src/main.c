/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : SI4463 Packet Beacon over UART3 — CORRECTED
 *
 * Pin mapping (from main.h):
 *   SPI1 SCK  → PA5   (CubeMX default SPI1)
 *   SPI1 MISO → PA6
 *   SPI1 MOSI → PA7
 *   SI_NSS    → PA4   (software CS, defined in main.h)
 *   RF_SWITCH → PA1   (output; HIGH = TX mode, LOW = RX/idle)
 *   SDN       → PB14  (SDN_Pin / SDN_GPIO_Port)
 *   NIRQ      → PB15  (NIRQ_Pin / NIRQ_GPIO_Port) — input, falling edge
 *   UART3 TX  → PB10  (115200 8N1)
 *
 * NOTE: radio_config.h (RADIO_CONFIGURATION_DATA_ARRAY) is left untouched.
 * That config sets RADIO_STATE_AFTER_POWER_UP so the SI4463 comes up in
 * READY after SI4463_Init() loads the array. This firmware builds and
 * transmits the "Hello HAMS" packet on its own via APP_TransmitPacket().
 *
 * Per RF_FRR_CTL_A_MODE_4 in radio_config.h, the FRR slots are mapped:
 *   FRR_A (property 0x03) → PH_PEND    (Packet Handler Interrupt Pending)
 *   FRR_B (property 0x07) → CHIP_PEND  (Chip Interrupt Pending)
 *   FRR_C (property 0x0A) → LATCHED_RSSI
 *   FRR_D (property 0x00) → DISABLED (reads back 0x00, ignore)
 *
 * ----------------------------------------------------------------------
 * FIX HISTORY
 *
 * 1) RF_SWITCH was never driven HIGH before transmission in the earlier
 *    continuous-TX revision of this firmware — it was set LOW once in
 *    MX_GPIO_Init() and never touched again, so the external antenna
 *    switch stayed parked on the RX/idle leg for the whole run even
 *    though the SI4463 itself was correctly entering TX. Fixed by
 *    driving RF_SWITCH HIGH in APP_StartTX() before START_TX is issued,
 *    and back LOW on completion / error / timeout, in every exit path
 *    of APP_WaitPacketSent() and APP_TransmitPacket().
 *
 * 2) APP_WaitPacketSent() was reading the wrong bytes out of the
 *    GET_INT_STATUS (0x20) response. Per AN633 Table 4 / the Si4463 API,
 *    the actual response layout is:
 *
 *      response[0] = INT_PEND
 *      response[1] = INT_STATUS
 *      response[2] = PH_PEND      <- PACKET_SENT is bit 5 here
 *      response[3] = PH_STATUS
 *      response[4] = MODEM_PEND
 *      response[5] = MODEM_STATUS
 *      response[6] = CHIP_PEND    <- CMD_ERROR is bit 3 here
 *      response[7] = CHIP_STATUS
 *
 *    The earlier code checked response[0] for PACKET_SENT (wrong
 *    register) and treated response[6] as a fabricated "CMD_ERR_STATUS"
 *    field that doesn't exist in this response (it only exists in the
 *    response to GET_CHIP_STATUS, command 0x23). CHIP_PEND bit 4
 *    (STATE_CHANGE) fires on essentially every TX-complete transition,
 *    so the old code aborted the wait loop on a false "radio error" and
 *    then timed out waiting on a PACKET_SENT bit that was never going
 *    to be set in that byte.
 *
 * 3) Command ordering in APP_StartTX() reordered to match AN633
 *    Figure 15 (SPI Communications Example): FIFO_INFO reset → clear
 *    all pending interrupts → WRITE_TX_FIFO → START_TX. Functionally
 *    equivalent to the previous order, kept for direct traceability
 *    against the app note's captured reference transaction.
 *
 * 4) The AN633 §10.1-style carrier-hold diagnostic (forces txState
 *    directly via CHANGE_STATE, bypassing the packet handler, so a
 *    multimeter/SDR has an easy continuous window to catch the
 *    carrier) is now gated behind APP_RUN_CARRIER_HOLD_TEST so it only
 *    runs when you explicitly want it, instead of on every power-up.
 * ----------------------------------------------------------------------
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "si4463.h"
#include "radio_config.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * Dummy satellite-health telemetry frame. Packed, fixed layout, sent as
 * raw bytes over the SI4463 in place of a plain ASCII payload. All
 * field values are simulated (see APP_BuildDummyTelemetry()) -- this is
 * a placeholder structure/format for exercising the RF link with a
 * realistic-looking beacon frame, not real sensor data.
 *
 * Size is fixed at compile time (sizeof(SatTelemetry_t), currently 26
 * bytes) and must stay <= SI4463_MAX_TX_FIFO_LEN (64 bytes, si4463.h).
 */
typedef struct __attribute__((packed))
{
    char     callsign[8];       /* Station ID, NUL-padded (e.g. "VU3LTQ") */
    uint32_t uptimeSeconds;     /* Seconds since MCU boot */
    uint16_t batteryVoltage_mV; /* Battery bus voltage, millivolts */
    int16_t  batteryCurrent_mA; /* Battery current, mA (+ = charging) */
    int16_t  temperature_dC;    /* Onboard temperature, deci-degC (235 = 23.5C) */
    uint16_t solarVoltage_mV;   /* Solar panel bus voltage, millivolts */
    uint8_t  operatingMode;     /* 0=SAFE 1=NOMINAL 2=SCIENCE 3=LOW_POWER */
    uint8_t  statusFlags;       /* bit0=deployed bit1=sunSide bit2=lowBatt bit3=wdtReset */
    uint16_t telemetryCounter;  /* Rolling telemetry frame counter */
    uint8_t  resetCount;        /* MCU reset count since deployment (dummy) */
    uint8_t  checksum;          /* 8-bit additive checksum over all preceding bytes */
} SatTelemetry_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Set to 1 to run the 5-second carrier-hold diagnostic once at boot
 * (AN633 Section 10.1 equivalent). Leave at 0 for normal beacon
 * operation once the RF path has been confirmed with a meter/SDR. */
#define APP_RUN_CARRIER_HOLD_TEST   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*
 * SI4463 command constants.
 *
 * Do NOT redefine commands that already exist in si4463.h.
 */
#define APP_CMD_READ_CMD_BUFF       0x44u
#define APP_CMD_FIFO_INFO           0x15u
#define APP_CMD_GET_INT_STATUS      0x20u
#define APP_CMD_WRITE_TX_FIFO       0x66u
#define APP_CMD_START_TX            0x31u

/* FIFO_INFO argument. Bit1 = TX FIFO reset, Bit0 = RX FIFO reset
 * (Si4463 API command encoding). */
#define APP_FIFO_CLEAR_TX           0x02u

/* START_TX CONDITION byte: TXCOMPLETE_STATE occupies bits [6:4] and is
 * encoded as (next_state_enum_value << 4). Using the same state values
 * as si4463_state_t (readyState = 3) so the radio returns cleanly to
 * READY once the packet is out, rather than leaving the next-state
 * field at 0 ("no change" / undefined for this transition). */
#define APP_START_TX_STATE_READY    (0x03u << 4)   /* -> 0x30 */

/* PH_PEND bit 5 = PACKET_SENT */
#define APP_PH_PEND_PACKET_SENT     0x20u
/* CHIP_PEND bit 3 = CMD_ERROR */
#define APP_CHIP_PEND_CMD_ERROR     0x08u

#define TX_PAYLOAD                  "Hello HAMS, this is VU3LTQ operating from KJSIT! 73!!"
#define TX_PAYLOAD_LEN               ((uint8_t)(sizeof(TX_PAYLOAD) - 1u))
#define TX_INTERVAL_MS               5000u
#define TX_TIMEOUT_MS                1000u

/* Dummy satellite operating modes for SatTelemetry_t.operatingMode. */
#define APP_SAT_MODE_SAFE           0u
#define APP_SAT_MODE_NOMINAL        1u
#define APP_SAT_MODE_SCIENCE        2u
#define APP_SAT_MODE_LOW_POWER      3u

/* Dummy satellite status bits for SatTelemetry_t.statusFlags. */
#define APP_SAT_FLAG_DEPLOYED       (1u << 0)
#define APP_SAT_FLAG_SUN_SIDE       (1u << 1)
#define APP_SAT_FLAG_LOW_BATTERY    (1u << 2)
#define APP_SAT_FLAG_WDT_RESET      (1u << 3)

_Static_assert(sizeof(SatTelemetry_t) <= SI4463_MAX_TX_FIFO_LEN,
               "SatTelemetry_t must fit in the 64-byte SI4463 TX FIFO");

static uint32_t packetCounter = 0u;
static char uartBuffer[256];

si4463_t si4463;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* UART */
static void UART_Print(const char *fmt, ...);

/* SPI / SI4463 hardware interface */
static uint8_t APP_IsCTS(void);
static void APP_WriteRead(const uint8_t *pTxData, uint8_t *pRxData, uint16_t size);
static void APP_Select(void);
static void APP_Deselect(void);
static void APP_SetShutdown(void);
static void APP_ClearShutdown(void);

/* SI4463 application-level functions */
static bool APP_SendCommand(const uint8_t *cmd, uint8_t cmdLen);
static bool APP_SendCommandResponse(const uint8_t *cmd, uint8_t cmdLen,
                                     uint8_t *response, uint8_t responseLen);
static bool APP_ClearInterrupts(void);
static bool APP_FlushTXFIFO(void);
static bool APP_WriteTXFIFO(const uint8_t *data, uint8_t length);
static bool APP_StartTX(const uint8_t *data, uint8_t length);
static bool APP_WaitPacketSent(uint32_t timeoutMs);
static bool APP_TransmitPacket(const uint8_t *data, uint8_t length);

/* Dummy satellite-health telemetry */
static uint8_t APP_ComputeChecksum(const uint8_t *data, uint8_t length);
static void APP_BuildDummyTelemetry(SatTelemetry_t *tlm, uint32_t seqNum);
static void APP_PrintTelemetry(const SatTelemetry_t *tlm);
#if APP_RUN_CARRIER_HOLD_TEST
static void APP_CarrierHoldTest(uint32_t holdMs);
static void APP_RxCurrentTest(uint32_t holdMs);
static void APP_RealTxBurstTest(uint32_t durationMs);
#endif

/* Diagnostics: confirm the chip is genuinely a Si4463 and that the PA
 * properties actually landed in the radio's property RAM as configured
 * by radio_config.h, rather than being silently rejected/reset. */
static bool APP_ReadPartInfo(void);
static bool APP_ReadPAProperties(void);
static bool APP_ReadModemModType(void);
static bool APP_ReadFreqControl(void);
/* USER CODE END PFP */

/* ========================================================================== */
/* UART                                                                       */
/* ========================================================================== */
static void UART_Print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int length = vsnprintf(uartBuffer, sizeof(uartBuffer), fmt, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }
    if (length >= (int)sizeof(uartBuffer))
    {
        length = (int)sizeof(uartBuffer) - 1;
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)uartBuffer, (uint16_t)length, 1000);
}

/* ========================================================================== */
/* SI4463 CHIP SELECT                                                         */
/* ========================================================================== */
static void APP_Select(void)
{
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
}

static void APP_Deselect(void)
{
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

/* ========================================================================== */
/* SI4463 SHUTDOWN                                                            */
/* ========================================================================== */
static void APP_SetShutdown(void)
{
    HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_SET);
}

static void APP_ClearShutdown(void)
{
    HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);
}

/* ========================================================================== */
/* SPI TRANSFER                                                               */
/* ========================================================================== */
static void APP_WriteRead(const uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    static uint8_t dummyTx[260];
    static uint8_t dummyRx[260];

    if (size == 0u)
    {
        return;
    }
    if (size > sizeof(dummyTx))
    {
        size = sizeof(dummyTx);
    }

    if ((pTxData != NULL) && (pRxData != NULL))
    {
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)pTxData, pRxData, size, HAL_MAX_DELAY);
    }
    else if (pTxData != NULL)
    {
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)pTxData, dummyRx, size, HAL_MAX_DELAY);
    }
    else if (pRxData != NULL)
    {
        memset(dummyTx, 0xFF, size);
        HAL_SPI_TransmitReceive(&hspi1, dummyTx, pRxData, size, HAL_MAX_DELAY);
    }
}

/* ========================================================================== */
/* CTS — AN633 §7.2.1: send 0x44 + 8 clock pulses, CTS returned on 2nd byte  */
/* ========================================================================== */
static uint8_t APP_IsCTS(void)
{
    uint8_t tx[2] = { APP_CMD_READ_CMD_BUFF, 0xFFu };
    uint8_t rx[2] = { 0x00u, 0x00u };

    APP_Select();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    APP_Deselect();

    return (rx[1] == 0xFFu);
}

/* ========================================================================== */
/* SEND COMMAND — AN633 Figure 7                                             */
/* ========================================================================== */
static bool APP_SendCommand(const uint8_t *cmd, uint8_t cmdLen)
{
    uint32_t start;

    if ((cmd == NULL) || (cmdLen == 0u))
    {
        return false;
    }

    start = HAL_GetTick();
    while (!APP_IsCTS())
    {
        if ((HAL_GetTick() - start) >= 500u)
        {
            UART_Print("ERROR: CTS timeout, command=0x%02X\r\n", cmd[0]);
            return false;
        }
        HAL_Delay(1);
    }

    APP_Select();
    APP_WriteRead(cmd, NULL, cmdLen);
    APP_Deselect();

    return true;
}

/* ========================================================================== */
/* SEND COMMAND + RESPONSE — AN633 Figures 9/10: CTS check and response      */
/* retrieval combined into one NSEL-low transaction once CTS == 0xFF.        */
/* ========================================================================== */
static bool APP_SendCommandResponse(const uint8_t *cmd, uint8_t cmdLen,
                                     uint8_t *response, uint8_t responseLen)
{
    uint8_t tx[32];
    uint8_t rx[32];
    uint32_t start;

    if ((cmd == NULL) || (cmdLen == 0u))
    {
        return false;
    }
    if ((response == NULL) || (responseLen == 0u))
    {
        return false;
    }
    if (responseLen > 16u)
    {
        return false;
    }

    if (!APP_SendCommand(cmd, cmdLen))
    {
        return false;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 500u)
    {
        uint16_t transferLength = (uint16_t)responseLen + 2u;

        memset(tx, 0xFF, sizeof(tx));
        memset(rx, 0x00, sizeof(rx));
        tx[0] = APP_CMD_READ_CMD_BUFF;

        APP_Select();
        APP_WriteRead(tx, rx, transferLength);
        APP_Deselect();

        /*
         * rx[0] is the dummy byte.
         * rx[1] is CTS.
         * rx[2...] contains the command response.
         */
        if (rx[1] == 0xFFu)
        {
            memcpy(response, &rx[2], responseLen);
            return true;
        }

        HAL_Delay(1);
    }

    UART_Print("ERROR: command response timeout, command=0x%02X\r\n", cmd[0]);
    return false;
}

/* ========================================================================== */
/* CLEAR INTERRUPTS — GET_INT_STATUS with all-zero clear masks clears every  */
/* pending flag in all three groups (PH, MODEM, CHIP).                       */
/* ========================================================================== */
static bool APP_ClearInterrupts(void)
{
    uint8_t cmd[4] = { APP_CMD_GET_INT_STATUS, 0x00u, 0x00u, 0x00u };
    uint8_t response[8];
    memset(response, 0, sizeof(response));

    return APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response));
}

/* ========================================================================== */
/* FLUSH TX FIFO                                                              */
/* ========================================================================== */
static bool APP_FlushTXFIFO(void)
{
    uint8_t cmd[2] = { APP_CMD_FIFO_INFO, APP_FIFO_CLEAR_TX };

    /* FIFO_INFO returns two response bytes: TX FIFO count and RX FIFO count. */
    uint8_t response[2];
    memset(response, 0, sizeof(response));

    if (!APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
    {
        return false;
    }

    return true;
}

/* ========================================================================== */
/* WRITE TX FIFO — AN633 §7.5 / Figure 13                                    */
/* ========================================================================== */
static bool APP_WriteTXFIFO(const uint8_t *data, uint8_t length)
{
    uint8_t command = APP_CMD_WRITE_TX_FIFO;
    uint32_t start;

    if ((data == NULL) || (length == 0u))
    {
        return false;
    }

    start = HAL_GetTick();
    while (!APP_IsCTS())
    {
        if ((HAL_GetTick() - start) >= 500u)
        {
            UART_Print("ERROR: CTS timeout before TX FIFO write\r\n");
            return false;
        }
        HAL_Delay(1);
    }

    /*
     * NSEL MUST remain LOW for the complete WRITE_TX_FIFO command +
     * payload transaction.
     */
    APP_Select();
    APP_WriteRead(&command, NULL, 1);
    APP_WriteRead(data, NULL, length);
    APP_Deselect();

    return true;
}

/* ========================================================================== */
/* START TX                                                                   */
/*                                                                             */
/* Command / FIFO ordering here matches AN633 Figure 15 (SPI Communications  */
/* Example): FIFO_INFO reset TX FIFO -> clear all pending ITs -> write       */
/* payload into TX FIFO -> issue START_TX.                                   */
/*                                                                             */
/* RF_SWITCH is driven HIGH (TX) here, before START_TX is sent, and is       */
/* always returned LOW on any failure path in this function so the external */
/* antenna switch never gets stuck on the TX leg without an active TX.       */
/* ========================================================================== */
static bool APP_StartTX(const uint8_t *data, uint8_t length)
{
    uint8_t cmd[5];

    if ((data == NULL) || (length == 0u))
    {
        return false;
    }

    /* External RF switch -> TX. HIGH = TX per RF_SWITCH pin contract
     * (see header comment). This is the fix for the earlier revision
     * where RF_SWITCH was only ever set once, at LOW, in MX_GPIO_Init()
     * and never touched again — the SI4463 could be transmitting
     * correctly on-die while the antenna path stayed parked on RX. */
    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    /* Reset TX FIFO first, per AN633 Figure 15 ordering. */
    if (!APP_FlushTXFIFO())
    {
        UART_Print("ERROR: TX FIFO flush failed\r\n");
        HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
        return false;
    }

    /* Clear stale interrupt flags. */
    if (!APP_ClearInterrupts())
    {
        UART_Print("ERROR: GET_INT_STATUS failed\r\n");
        HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
        return false;
    }

    /* Write payload. */
    if (!APP_WriteTXFIFO(data, length))
    {
        UART_Print("ERROR: TX FIFO write failed\r\n");
        HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
        return false;
    }

    /*
     * START_TX command:
     *
     * Byte 0 = command
     * Byte 1 = channel
     * Byte 2 = condition (bits[6:4] = TXCOMPLETE_STATE)
     * Byte 3 = TX length MSB (bits [12:8])
     * Byte 4 = TX length LSB (bits [7:0])
     *
     * TXCOMPLETE_STATE is explicitly set to READY so the chip lands in
     * a known state once the packet goes out, instead of leaving the
     * field at 0 (undefined "no change" for this transition).
     */
    cmd[0] = APP_CMD_START_TX;
    cmd[1] = RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER;
    cmd[2] = APP_START_TX_STATE_READY;
    cmd[3] = 0x00u;
    cmd[4] = length;

    UART_Print("START_TX: channel=%u length=%u\r\n", (unsigned int)cmd[1], (unsigned int)length);

    if (!APP_SendCommand(cmd, sizeof(cmd)))
    {
        UART_Print("ERROR: START_TX command failed\r\n");
        HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
        return false;
    }

    /* Give the radio state machine time to enter TX. */
    HAL_Delay(2);

    UART_Print("GPIO0=%d GPIO1=%d\r\n",
               (int)HAL_GPIO_ReadPin(GPIO0_GPIO_Port, GPIO0_Pin),
               (int)HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin));

    return true;
}

/* ========================================================================== */
/* WAIT PACKET SENT                                                           */
/*                                                                             */
/* GET_INT_STATUS response layout (AN633 Table 4 / Si4463 API):              */
/*   response[0] = INT_PEND                                                  */
/*   response[1] = INT_STATUS                                                */
/*   response[2] = PH_PEND      <- PACKET_SENT is bit 5 here                 */
/*   response[3] = PH_STATUS                                                 */
/*   response[4] = MODEM_PEND                                                */
/*   response[5] = MODEM_STATUS                                              */
/*   response[6] = CHIP_PEND    <- CMD_ERROR is bit 3 here                   */
/*   response[7] = CHIP_STATUS                                               */
/* ========================================================================== */
static bool APP_WaitPacketSent(uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeoutMs)
    {
        uint8_t cmd[4] = { APP_CMD_GET_INT_STATUS, 0x00u, 0x00u, 0x00u };
        uint8_t response[8];
        memset(response, 0, sizeof(response));

        if (APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
        {
            if ((response[2] & APP_PH_PEND_PACKET_SENT) != 0u)
            {
                UART_Print("PACKET_SENT: PH_PEND=0x%02X\r\n", response[2]);

                /* Return RF switch to idle/RX. */
                HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
                return true;
            }

            /*
             * Only a genuine CMD_ERROR bit in CHIP_PEND means something
             * actually went wrong. STATE_CHANGE (bit 4) fires routinely
             * as the radio transitions TX_TUNE -> TX -> READY and must
             * NOT be treated as an error.
             */
            if ((response[6] & APP_CHIP_PEND_CMD_ERROR) != 0u)
            {
                UART_Print("RADIO CMD ERROR: CHIP_PEND=0x%02X\r\n", response[6]);
                break;
            }
        }

        HAL_Delay(2);
    }

    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
    UART_Print("ERROR: PACKET_SENT timeout\r\n");
    return false;
}

/* ========================================================================== */
/* TRANSMIT PACKET                                                            */
/* ========================================================================== */
static bool APP_TransmitPacket(const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0u))
    {
        return false;
    }

    if (length > 64u)
    {
        UART_Print("ERROR: payload too large: %u bytes\r\n", (unsigned int)length);
        return false;
    }

    if (!APP_StartTX(data, length))
    {
        HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
        return false;
    }

    if (!APP_WaitPacketSent(TX_TIMEOUT_MS))
    {
        return false;
    }

    return true;
}

/* ========================================================================== */
/* DUMMY SATELLITE-HEALTH TELEMETRY                                          */
/* ========================================================================== */
/*
 * Simple 8-bit additive checksum over `length` bytes starting at `data`.
 * Not a substitute for the SI4463's own hardware CRC on the air
 * interface -- this is an application-layer field so a receiver can
 * sanity-check the decoded struct itself after CRC-valid reception.
 */
static uint8_t APP_ComputeChecksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0u;

    for (uint8_t i = 0; i < length; i++)
    {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

/*
 * Fills tlm with simulated "satellite health" values. There is no real
 * sensor data behind any of this -- every field is a deterministic
 * function of seqNum and HAL_GetTick() so successive telemetry frames
 * look plausibly alive (battery drifting, temperature drifting, etc.)
 * for exercising the RF link and a receiver-side decoder.
 */
static void APP_BuildDummyTelemetry(SatTelemetry_t *tlm, uint32_t seqNum)
{
    memset(tlm, 0, sizeof(*tlm));

    memcpy(tlm->callsign, "VU3LTQ", 6u);

    tlm->uptimeSeconds = HAL_GetTick() / 1000u;

    /* Battery voltage dummy-wanders between ~7.2V and ~8.0V. */
    tlm->batteryVoltage_mV = (uint16_t)(7200u + (seqNum * 37u) % 800u);

    /* Current alternates sign to simulate charge/discharge cycles. */
    tlm->batteryCurrent_mA = (int16_t)(((seqNum % 2u) ? 1 : -1)
                                        * (int16_t)(150 + (seqNum * 7u) % 250u));

    /* Temperature dummy-wanders between -5.0C and +25.0C. */
    tlm->temperature_dC = (int16_t)(-50 + (int16_t)((seqNum * 11u) % 300u));

    /* Solar bus voltage dummy-wanders between 0V (eclipse) and ~9V (sun). */
    tlm->solarVoltage_mV = (uint16_t)((seqNum % 3u) ? (6000u + (seqNum * 53u) % 3000u) : 0u);

    tlm->operatingMode = (uint8_t)(tlm->solarVoltage_mV == 0u
                                    ? APP_SAT_MODE_LOW_POWER
                                    : APP_SAT_MODE_NOMINAL);

    tlm->statusFlags = APP_SAT_FLAG_DEPLOYED;
    if (tlm->solarVoltage_mV > 0u)
    {
        tlm->statusFlags |= APP_SAT_FLAG_SUN_SIDE;
    }
    if (tlm->batteryVoltage_mV < 7400u)
    {
        tlm->statusFlags |= APP_SAT_FLAG_LOW_BATTERY;
    }

    tlm->telemetryCounter = (uint16_t)seqNum;
    tlm->resetCount = 0u;

    tlm->checksum = APP_ComputeChecksum((const uint8_t *)tlm,
                                         (uint8_t)(sizeof(*tlm) - 1u));
}

static void APP_PrintTelemetry(const SatTelemetry_t *tlm)
{
    char callsignBuf[sizeof(tlm->callsign) + 1u];
    memcpy(callsignBuf, tlm->callsign, sizeof(tlm->callsign));
    callsignBuf[sizeof(tlm->callsign)] = '\0';

    UART_Print("Telemetry #%u: callsign=%s uptime=%lus battV=%umV battI=%dmA "
               "temp=%d.%01dC solarV=%umV mode=%u flags=0x%02X chk=0x%02X\r\n",
               (unsigned int)tlm->telemetryCounter, callsignBuf,
               (unsigned long)tlm->uptimeSeconds,
               (unsigned int)tlm->batteryVoltage_mV,
               (int)tlm->batteryCurrent_mA,
               (int)(tlm->temperature_dC / 10),
               (int)((tlm->temperature_dC < 0 ? -tlm->temperature_dC : tlm->temperature_dC) % 10),
               (unsigned int)tlm->solarVoltage_mV,
               (unsigned int)tlm->operatingMode,
               (unsigned int)tlm->statusFlags,
               (unsigned int)tlm->checksum);
}

#if APP_RUN_CARRIER_HOLD_TEST
/* ========================================================================== */
/* CARRIER HOLD TEST (diagnostic only, AN633 Section 10.1)                   */
/* ========================================================================== */
/*
 * Forces the radio directly into txState (see si4463_state_t in si4463.h,
 * txState = 7) via CHANGE_STATE, bypassing START_TX / the packet handler
 * entirely. Per AN633 Figure 16, once in txState the PA stays on and draws
 * ~18 mA continuously (vs. the ~10-30 ms burst a single packet produces),
 * and the carrier will sit at your configured 435 MHz continuously.
 *
 * This isolates "is the RF chain / antenna / PA actually working" from
 * "is my packet timing too short to observe" - if you see 18 mA on your
 * meter and a solid line at 435.000 MHz in SDR++ during this hold, the
 * hardware path is fine and TX #n: SUCCESS in the beacon loop is real;
 * the packets are just too short-duty-cycle to catch by eye.
 *
 * If you see NOTHING here either, the problem is upstream of packet
 * timing - check PA_PWR_LVL/PA_MODE (RF_PA_MODE_4 in radio_config.h),
 * the RF_SWITCH GPIO polarity/wiring to the external antenna switch,
 * and the SMA/antenna connection itself.
 */
static void APP_CarrierHoldTest(uint32_t holdMs)
{
    UART_Print("\r\n=== CARRIER HOLD TEST: forcing txState for %lu ms ===\r\n",
               (unsigned long)holdMs);

    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    /* txState = 7 in si4463_state_t (si4463.h) */
    if (SI4463_SetCurrentState(&si4463, txState) != SI4463_OK)
    {
        UART_Print("WARNING: CHANGE_STATE to txState did not verify cleanly "
                   "(driver still issued the command - check meter/SDR now)\r\n");
    }

    UART_Print("Radio should now be parked in TX. Check multimeter (~18 mA) "
               "and SDR++ at 435.000 MHz now.\r\n");

    /* Poll GET_CHIP_STATUS / GET_INT_STATUS a couple of times during the
     * hold to see if the chip itself is flagging LOW_BATT (supply sag
     * under the PA's current draw) or a CMD_ERROR while parked in TX. */
    for (uint8_t i = 0; i < 3u; i++)
    {
        HAL_Delay(holdMs / 3u);

        if (SI4463_GetInterrupts(&si4463) == SI4463_OK)
        {
            UART_Print("  TX hold chip status: lowBatt=%d cmdError=%d "
                       "cal=%d fifoErr=%d\r\n",
                       (int)si4463.interrupts.lowBatt,
                       (int)si4463.interrupts.cmdError,
                       (int)si4463.interrupts.cal,
                       (int)si4463.interrupts.fifoUnderflowOverflowError);
        }
        else
        {
            UART_Print("  TX hold chip status: GET_INT_STATUS read failed\r\n");
        }
    }

    SI4463_SetCurrentState(&si4463, readyState);
    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);

    UART_Print("=== CARRIER HOLD TEST DONE, back to READY ===\r\n\r\n");
}

/* ========================================================================== */
/* RX CURRENT TEST (diagnostic only)                                         */
/* ========================================================================== */
/*
 * Companion to APP_CarrierHoldTest(). Forces the radio directly into
 * rxState via CHANGE_STATE for holdMs. RX, like TX, requires the
 * synthesizer to be locked (for down-conversion), and has a
 * well-characterized current draw (~11 mA typical for the Si4463 in RX,
 * per the datasheet, vs. ~1.5 uA in standby / a few hundred uA in READY).
 *
 * Run this alongside APP_CarrierHoldTest() to distinguish two different
 * root causes for "no current bump / no signal during TX carrier hold":
 *
 *   - If RX ALSO shows no current increase over the READY baseline, the
 *     crystal/synthesizer most likely never locks at all (both RX and TX
 *     depend on it equally) -- check the XTAL itself, its load
 *     capacitors, and GLOBAL_XO_TUNE_2 in radio_config.h against the
 *     actual crystal fitted on this board.
 *
 *   - If RX DOES show the expected current bump but TX does not, the
 *     synthesizer/crystal is fine and the fault is specific to the PA /
 *     TX output path (PA bias, matching network, or a damaged TX pin).
 */
static void APP_RxCurrentTest(uint32_t holdMs)
{
    UART_Print("\r\n=== RX CURRENT TEST: forcing rxState for %lu ms ===\r\n",
               (unsigned long)holdMs);

    /* RF_SWITCH stays LOW here -- LOW is the RX/idle leg per this board's
     * contract (see header comment at the top of this file). */
    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    /* rxState = 8 in si4463_state_t (si4463.h) */
    if (SI4463_SetCurrentState(&si4463, rxState) != SI4463_OK)
    {
        UART_Print("WARNING: CHANGE_STATE to rxState did not verify cleanly "
                   "(driver still issued the command - check meter now)\r\n");
    }

    UART_Print("Radio should now be parked in RX. Check multimeter (~11 mA "
               "typical) now.\r\n");

    HAL_Delay(holdMs);

    SI4463_SetCurrentState(&si4463, readyState);

    UART_Print("=== RX CURRENT TEST DONE, back to READY ===\r\n"
               "Compare this reading against the TX carrier-hold test above:\r\n"
               "  - Neither shows a bump  -> crystal/synthesizer likely never\r\n"
               "    locks. Check XTAL, its load caps, and GLOBAL_XO_TUNE_2.\r\n"
               "  - RX bumps but TX doesn't -> fault is specific to the PA /\r\n"
               "    TX output path, not the crystal.\r\n\r\n");
}

/* ========================================================================== */
/* REAL TX BURST TEST (diagnostic only)                                      */
/* ========================================================================== */
/*
 * The raw CHANGE_STATE carrier-hold test above bypasses the packet
 * handler entirely, which starves the TX FIFO and reliably raises
 * FIFO_UNDERFLOW (visible as fifoErr=1 in the TX hold chip-status log).
 * Since radio_config.h has the packet handler enabled (not raw/direct
 * TX mode), the chip does not fully key the PA against an empty/garbage
 * FIFO, so a low current reading during that specific test does NOT by
 * itself prove a broken PA -- it may just be a diagnostic-method
 * artifact.
 *
 * This test instead fires real, properly-framed packets (the exact same
 * APP_TransmitPacket() path used by the beacon loop -- FIFO_INFO reset,
 * WRITE_TX_FIFO, START_TX, wait for PACKET_SENT) back-to-back for
 * durationMs with effectively no gap between them. A single "Hello HAMS"
 * packet only lasts a few ms, too short to reliably catch on a handheld
 * meter or an SDR without persistence/hold enabled -- running dozens of
 * them back-to-back raises the average duty cycle enough to actually
 * see on a meter, and gives an SDR waterfall with hold/max-hold enabled
 * a much better chance of showing repeated bursts at the configured
 * frequency.
 */
static void APP_RealTxBurstTest(uint32_t durationMs)
{
    uint32_t start = HAL_GetTick();
    uint32_t sent = 0;
    uint32_t failed = 0;

    UART_Print("\r\n=== REAL TX BURST TEST: sending framed packets back-to-back "
               "for %lu ms ===\r\n", (unsigned long)durationMs);
    UART_Print("Watch meter (average current) and SDR waterfall (with "
               "hold/max-hold) at 435.000 MHz now.\r\n");

    while ((HAL_GetTick() - start) < durationMs)
    {
        if (APP_TransmitPacket((const uint8_t *)TX_PAYLOAD, TX_PAYLOAD_LEN))
        {
            sent++;
        }
        else
        {
            failed++;
        }
    }

    UART_Print("=== REAL TX BURST TEST DONE: %lu sent OK, %lu failed ===\r\n\r\n",
               (unsigned long)sent, (unsigned long)failed);
}
#endif /* APP_RUN_CARRIER_HOLD_TEST */

/* ========================================================================== */
/* PART_INFO — AN633 Table 4, command 0x01                                   */
/*                                                                             */
/* Response (8 bytes): CHIPREV, PART[1], PART[0], PBUILD, ID[1], ID[0],      */
/* CUSTOMER, ROMID.                                                          */
/*                                                                             */
/* PART should read back as 0x4463. If it reads 0x0000, all-0xFF, or         */
/* anything else, the SPI link to the chip itself is suspect (bad solder     */
/* joint, wrong VDD, dead part, wrong NSS/SCK/MISO/MOSI mapping) — that      */
/* would explain full-scale current staying flat regardless of what         */
/* commands you send, since the chip may not be receiving them at all.      */
/* ========================================================================== */
static bool APP_ReadPartInfo(void)
{
    uint8_t cmd[1] = { 0x01u }; /* PART_INFO */
    uint8_t response[8];
    memset(response, 0, sizeof(response));

    if (!APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
    {
        UART_Print("ERROR: PART_INFO read failed\r\n");
        return false;
    }

    uint16_t part = ((uint16_t)response[1] << 8) | response[2];
    UART_Print("PART_INFO: CHIPREV=0x%02X PART=0x%04X PBUILD=0x%02X "
               "ID=0x%02X%02X CUSTOMER=0x%02X ROMID=0x%02X\r\n",
               response[0], part, response[3], response[4], response[5],
               response[6], response[7]);

    if (part != 0x4463u)
    {
        UART_Print("WARNING: expected PART=0x4463, got 0x%04X — check SPI "
                   "wiring, VDD, and that the part is genuinely a Si4463\r\n",
                   part);
    }

    return true;
}

/* ========================================================================== */
/* GET_PROPERTY on the PA group (0x22, start 0x00, 4 props) — AN633 Table 4, */
/* command 0x12.                                                             */
/*                                                                             */
/* Reads back PA_MODE, PA_PWR_LVL, PA_BIAS_CLKDUTY, PA_TC as they currently  */
/* sit in the radio's property RAM. If these don't match radio_config.h's    */
/* RF_PA_MODE_4 values (0x08, 0x7F, 0x00, 0x1D), the SET_PROPERTY calls      */
/* during SI4463_Init() didn't take — which would explain low output power  */
/* even though the chip is otherwise responding fine on SPI (e.g. CTS       */
/* works, PART_INFO reads correctly).                                       */
/* ========================================================================== */
static bool APP_ReadPAProperties(void)
{
    /* GET_PROPERTY command: cmd, GROUP, NUM_PROPS, START_PROP */
    uint8_t cmd[4] = { 0x12u, 0x22u, 0x04u, 0x00u };
    uint8_t response[4];
    memset(response, 0, sizeof(response));

    if (!APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
    {
        UART_Print("ERROR: GET_PROPERTY (PA group) read failed\r\n");
        return false;
    }

    UART_Print("PA properties: PA_MODE=0x%02X PA_PWR_LVL=0x%02X "
               "PA_BIAS_CLKDUTY=0x%02X PA_TC=0x%02X "
               "(radio_config.h expects 0x08 0x7F 0x00 0x1D)\r\n",
               response[0], response[1], response[2], response[3]);

    if ((response[0] != 0x08u) || (response[1] != 0x7Fu))
    {
        UART_Print("WARNING: PA properties don't match radio_config.h — "
                   "configuration did not take\r\n");
    }

    return true;
}

/* ========================================================================== */
/* GET_PROPERTY on MODEM_MOD_TYPE (group 0x20, prop 0x00) — confirms the     */
/* MOD_TYPE fix (0x03 2GFSK -> 0x05 4GFSK, radio_config.h) actually landed   */
/* in the chip's property RAM rather than being silently ignored/reset.      */
/* ========================================================================== */
static bool APP_ReadModemModType(void)
{
    uint8_t cmd[4] = { 0x12u, 0x20u, 0x01u, 0x00u };
    uint8_t response[1];
    memset(response, 0, sizeof(response));

    if (!APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
    {
        UART_Print("ERROR: GET_PROPERTY (MODEM_MOD_TYPE) read failed\r\n");
        return false;
    }

    UART_Print("MOD_TYPE=0x%02X (expect 0x05 = 4GFSK; 0x03 would be stale "
               "2GFSK)\r\n", response[0]);

    if (response[0] != 0x05u)
    {
        UART_Print("WARNING: MOD_TYPE does not read back as 4GFSK — "
                   "configuration did not take\r\n");
    }

    return true;
}

/* ========================================================================== */
/* GET_PROPERTY on FREQ_CONTROL (group 0x40, props 0x00-0x03: INTE, FRAC2,   */
/* FRAC1, FRAC0) — radio_config.h's RADIO_CONFIGURATION_DATA_ARRAY sends     */
/* RF_FREQ_CONTROL_INTE_8 and then RF_FREQ_CONTROL_INTE_8_1 to this SAME     */
/* property address; the second one wins on real hardware and the two       */
/* macros carry DIFFERENT INTE/FRAC values, so the chip's actual TX/RX      */
/* carrier frequency is NOT necessarily whatever any comment elsewhere in   */
/* this codebase (e.g. "435.000 MHz") assumes. This reads back what's       */
/* actually active so that can be confirmed instead of guessed.             */
/* ========================================================================== */
static bool APP_ReadFreqControl(void)
{
    uint8_t cmd[4] = { 0x12u, 0x40u, 0x04u, 0x00u };
    uint8_t response[4];
    memset(response, 0, sizeof(response));

    if (!APP_SendCommandResponse(cmd, sizeof(cmd), response, sizeof(response)))
    {
        UART_Print("ERROR: GET_PROPERTY (FREQ_CONTROL) read failed\r\n");
        return false;
    }

    UART_Print("FREQ_CONTROL: INTE=0x%02X FRAC=0x%02X%02X%02X "
               "(block1/RF_FREQ_CONTROL_INTE_8 expects INTE=0x37 FRAC=0x090000, "
               "block2/RF_FREQ_CONTROL_INTE_8_1 expects INTE=0x39 FRAC=0x080000 "
               "-- block2 should win)\r\n",
               response[0], response[1], response[2], response[3]);

    return true;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */
    si4463.IsClearToSend = APP_IsCTS;
    si4463.WriteRead     = APP_WriteRead;
    si4463.Select        = APP_Select;
    si4463.Deselect      = APP_Deselect;
    si4463.SetShutdown   = APP_SetShutdown;
    si4463.ClearShutdown = APP_ClearShutdown;
    si4463.DelayMs       = HAL_Delay;

    UART_Print("\r\n       SI4463 4FSK PACKET BEACON\r\n");
    UART_Print("Frame   : SatTelemetry_t (dummy satellite health)\r\n");
    UART_Print("Length  : %u bytes\r\n", (unsigned int)sizeof(SatTelemetry_t));
    UART_Print("Interval: %lu ms\r\n", (unsigned long)TX_INTERVAL_MS);
    UART_Print("Initializing SI4463...\r\n");

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    int8_t result = SI4463_Init(&si4463);
    UART_Print("SI4463 Init: %s\r\n", (result == SI4463_OK) ? "OK" : "FAIL");

    if (result != SI4463_OK)
    {
        UART_Print("FATAL: SI4463 initialization failed\r\n");
        while (1)
        {
            HAL_Delay(1000);
        }
    }

    if (SI4463_ClearInterrupts(&si4463) != SI4463_OK)
    {
        UART_Print("WARNING: initial interrupt clear failed\r\n");
    }

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* RF switch starts in idle/RX state. */
    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);

    UART_Print("SI4463 initialized successfully\r\n");
    UART_Print("Beacon ready\r\n");

    /* Diagnostics for the "carrier hold current stays low" symptom:
     * confirm the chip identifies itself correctly and that the PA
     * properties actually made it into the radio's property RAM. */
    APP_ReadPartInfo();
    APP_ReadPAProperties();
    APP_ReadModemModType();
    APP_ReadFreqControl();

#if APP_RUN_CARRIER_HOLD_TEST
    APP_CarrierHoldTest(5000u);
    APP_RxCurrentTest(5000u);
    APP_RealTxBurstTest(5000u);
#endif
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        bool txOK;
        SatTelemetry_t telemetry;

        packetCounter++;

        APP_BuildDummyTelemetry(&telemetry, packetCounter);

        UART_Print("\r\n----------------------------------------\r\n");
        UART_Print("TX #%lu\r\n", (unsigned long)packetCounter);
        APP_PrintTelemetry(&telemetry);

        txOK = APP_TransmitPacket((const uint8_t *)&telemetry, (uint8_t)sizeof(telemetry));

        if (txOK)
        {
            UART_Print("TX #%lu: SUCCESS\r\n", (unsigned long)packetCounter);
        }
        else
        {
            UART_Print("TX #%lu: FAILED\r\n", (unsigned long)packetCounter);
        }

        UART_Print("GPIO0=%d GPIO1=%d RF_SWITCH=%d\r\n",
                   (int)HAL_GPIO_ReadPin(GPIO0_GPIO_Port, GPIO0_Pin),
                   (int)HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin),
                   (int)HAL_GPIO_ReadPin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin));

        HAL_Delay(TX_INTERVAL_MS);
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 128;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * @brief SPI1 Initialization Function
 * @retval None
 */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = RF_SWITCH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RF_SWITCH_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI1_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SDN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NIRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NIRQ_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO3_Pin | GPIO2_Pin | GPIO1_Pin | GPIO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
