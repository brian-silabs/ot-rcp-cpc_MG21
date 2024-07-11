/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/ncp.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>

#include "openthread-system.h"
#include "app.h"

#include "reset_util.h"

#include "wake-on-rf/magic_packet.h"

#include "link.h"
#include "gpiointerrupt.h"
#include "em_gpio.h"

/**
 * This function initializes the NCP app.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
extern void otAppNcpInit(otInstance *aInstance);
static uint8_t *eventData;
static otInstance* sInstance = NULL;

otInstance *otGetInstance(void)
{
    return sInstance;
}

void sl_ot_create_instance(void)
{
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;

    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    sInstance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    sInstance = otInstanceInitSingle();
#endif
    assert(sInstance);
}

void sl_ot_ncp_init(void)
{
    otAppNcpInit(sInstance);
}

// Gpio callbacks called when pin interrupt was triggered.
void gpioCallback(uint8_t intNo)
{
  MagicPacketPayload_t magicPayload;
  magicPayload.frameCounter = 0;
  magicPayload.timeToLive = MAGIC_PACKET_DEFAULT_TTL;
  magicPayload.status = 1;

  otRadioFrame *aFrame = otPlatRadioGetTransmitBuffer(sInstance);

  createMagicPacket(0xFFFF, 0xFFFF, otLinkGetPanId(sInstance), aFrame->mPsdu, &magicPayload);
  aFrame->mLength = MAGIC_PACKET_PAYLOAD_LENGTH + HEADER_802154_LENGTH + CRC_802154_LENGTH;
  aFrame->mChannel = otLinkGetChannel(sInstance);
  otPlatRadioTransmit(sInstance, aFrame);
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/

void app_init(void)
{
    GPIO_PinModeSet(gpioPortD, 2, gpioModeInputPull, 1);
    GPIOINT_Init();

    // Register callback functions and enable interrupts
    GPIOINT_CallbackRegister(2, gpioCallback);
    GPIO_ExtIntConfig(gpioPortD, 2, 2, false, true, true);

    OT_SETUP_RESET_JUMP(argv);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
    otTaskletsProcess(sInstance);
    otSysProcessDrivers(sInstance);
}

/**************************************************************************//**
 * Application Exit.
 *****************************************************************************/
void app_exit(void)
{
    otInstanceFinalize(sInstance);
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    free(otInstanceBuffer);
#endif
    // TO DO : pseudo reset?
}

MagicPacketError_t magicPacketCallback(MagicPacketCallbackEvent_t event, void *data)
{
  switch (event) {
    case MAGIC_PACKET_EVENT_ENABLED:
      if(NULL != data)
      {
        eventData = (uint8_t*)data;
      }
      break;
    case MAGIC_PACKET_EVENT_DISABLED:
      break;
    case MAGIC_PACKET_EVENT_WAKE_RX:
      if(NULL != data)
      {
        eventData = (uint8_t*)data;
      }
      break;
    case MAGIC_PACKET_EVENT_TX:
      if(NULL != data)
      {
        eventData = (uint8_t*)data;
      }
      break;
    default:
      break;
  }

  return MAGIC_PACKET_SUCCESS;
}
