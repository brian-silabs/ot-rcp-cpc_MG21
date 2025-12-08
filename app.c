/*******************************************************************************
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/diag.h>
#include <openthread/ncp.h>
#include <openthread/tasklet.h>

#include "app.h"
#include "openthread-system.h"

#include "reset_util.h"

#include "sl_component_catalog.h"
#include "sl_memory_manager.h"

#include "wake-on-rf/magic_packet.h"

#include "gpiointerrupt.h"
#include "em_gpio.h"
#include "string.h"


#if SL_OPENTHREAD_ENABLE_HOST_WAKE_GPIO
#include "sl_gpio.h"
sl_gpio_t host_wakeup_gpio;
#endif

#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
#if OPENTHREAD_CONFIG_MULTIPLE_STATIC_INSTANCE_ENABLE == 0
#error "Support for multiple OpenThread static instance is disabled."
#endif
otInstance *sInstances[OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_NUM] = {NULL};
#endif // OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE

/**
 * This function initializes the NCP app.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
extern void otAppNcpInitMulti(otInstance **aInstances, uint8_t aCount);
#else
extern void otAppNcpInit(otInstance *aInstance);
#endif

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE && !OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
static uint8_t *sOtInstanceBuffer = NULL;
#endif
static otInstance *sInstance = NULL;
static MagicPacketPayload_t magicPayload_g;
static bool sendRequested_g = false;

otInstance *otGetInstance(void)
{
    return sInstance;
}

void sl_ot_create_instance(void)
{
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    for (int i = 0; i < OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_NUM; i++)
    {
        sInstances[i] = otInstanceInitMultiple(i);

        assert(sInstances[i]);
    }
    sInstance = sInstances[0];
#elif OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE && !OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    size_t otInstanceBufferLength = 0;

    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    sOtInstanceBuffer = (uint8_t *)sl_malloc(otInstanceBufferLength);
    assert(sOtInstanceBuffer);

    // Initialize OpenThread with the buffer
    sInstance = otInstanceInit(sOtInstanceBuffer, &otInstanceBufferLength);
#else
    sInstance = otInstanceInitSingle();
#endif
    assert(sInstance);
}

void sl_ot_ncp_init(void)
{
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    otAppNcpInitMulti(sInstances, OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_NUM);
#else
    otAppNcpInit(sInstance);
#endif
}

// Gpio callbacks called when pin interrupt was triggered.
void gpioCallback(uint8_t intNo)
{
  sendRequested_g = true;
}

/******************************************************************************
 * Application Init.
 *****************************************************************************/
OT_TOOL_WEAK void sl_host_wakeup_init(void)
{
#if SL_OPENTHREAD_ENABLE_HOST_WAKE_GPIO
    host_wakeup_gpio.port = (uint8_t)SL_OPENTHREAD_HOST_WAKEUP_GPIO_PORT;
    host_wakeup_gpio.pin  = (uint8_t)SL_OPENTHREAD_HOST_WAKEUP_GPIO_PIN;
    sl_gpio_set_pin_mode(&host_wakeup_gpio, SL_GPIO_MODE_PUSH_PULL, 0);
#endif
}

void app_init(void)
{
    GPIO_PinModeSet(gpioPortD, 2, gpioModeInputPull, 1);
    GPIOINT_Init();

    // Register callback functions and enable interrupts
    GPIOINT_CallbackRegister(2, gpioCallback);
    GPIO_ExtIntConfig(gpioPortD, 2, 2, false, true, true);

    OT_SETUP_RESET_JUMP(argv);
    sl_host_wakeup_init();
}

/******************************************************************************
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
    otTaskletsProcess(sInstance);
    otSysProcessDrivers(sInstance);

    if(sendRequested_g){
      sendRequested_g = false;
      magicPayload_g.frameCounter = 0;
      magicPayload_g.timeToLive = MAGIC_PACKET_DEFAULT_TTL;
      magicPayload_g.status = 1;
      sendMagicPacket(&magicPayload_g);
    }
}

/******************************************************************************
 * Application Exit.
 *****************************************************************************/
void app_exit(void)
{
    otInstanceFinalize(sInstance);
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE && !OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    sl_free(sOtInstanceBuffer);
#endif
    // TO DO : pseudo reset?
}

MagicPacketError_t magicPacketCallback(MagicPacketCallbackEvent_t event, void *data)
{
  switch (event) {
    case MAGIC_PACKET_EVENT_ENABLED:
      if(NULL != data)
      {
      }
      break;
    case MAGIC_PACKET_EVENT_DISABLED:
      break;
    case MAGIC_PACKET_EVENT_WAKE_RX:
      if(NULL != data)
      {
      }
      break;
    case MAGIC_PACKET_EVENT_TX:
      if(NULL != data)
      {
        // Get the tx frame and send it without csma.
        otRadioFrame *aTxFrame                   = otPlatRadioGetTransmitBuffer(sInstance);
        aTxFrame->mInfo.mTxInfo.mCsmaCaEnabled   = false;
        aTxFrame->mInfo.mTxInfo.mMaxCsmaBackoffs = 0;
        aTxFrame->mLength = MAGIC_PACKET_PAYLOAD_LENGTH + HEADER_802154_LENGTH + CRC_802154_LENGTH;
        aTxFrame->mChannel = otLinkGetChannel(sInstance);
        memcpy(aTxFrame->mPsdu, &((uint8_t *)data)[1], MAGIC_PACKET_PAYLOAD_LENGTH + HEADER_802154_LENGTH);
        // On successful transmit, this will call the transmit complete callback for the WORF packet
        otPlatRadioTransmit(sInstance, aTxFrame);
      }
      break;
    default:
      break;
  }

  return MAGIC_PACKET_SUCCESS;
}
