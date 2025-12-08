# ot-rcp-cpc_MG21

Added Support for SPINEL WORF

Requires ot-br-posix modifications

Requirees macro definition of `SL_CATALOG_WORF_PRESENT=1`

Currently, ncp_worf.cpp/hpp are in project root but are required to be put in gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/ncp/

## Modifying original ot-rcp application to support CPC

First operation is to move from spinel over uart to Silicon Labs CPC

Create a sample ot-rcp project for the target of your choosing

In my case BRD4180A

### Spinel over CPC

#### Silicon Labs Components changes

1. Requires removal of IOStream existing instance
2. Requires installation of component `CPC Sencondary` over to vcom

This will take over the previously deleted instance

3. Finally NCP CPC component is required

As per Silicon Labs documentation, this is limited to RCP arch support :

```
Description
This component provides Co-Processor Communication (CPC) support for the OpenThread stack. It requires that the OpenThread NCP component be included with the project. Currently only the OpenThread RCP stack is supported.
Quality
PRODUCTION
```

## Wake On RF Source code edits

* `config/sl_openthread_features_config.h`

```diff
diff --git a/config/sl_openthread_features_config.h b/config/sl_openthread_features_config.h
index 3b186b1..cf09941 100644
--- a/config/sl_openthread_features_config.h
+++ b/config/sl_openthread_features_config.h
@@ -188,7 +188,7 @@
 // </e>
 // <e>  Link Raw Service
 #ifndef OPENTHREAD_CONFIG_LINK_RAW_ENABLE
-#define OPENTHREAD_CONFIG_LINK_RAW_ENABLE           0
+#define OPENTHREAD_CONFIG_LINK_RAW_ENABLE           1^M
 #endif
 // </e>
 // <e>  MAC Filter
```

* `<sdk>/protocol/openthread/platform-abstraction/efr32/radio.c`

```diff
diff --git a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio.c b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio.c
index 26bb1f9..01093a5 100644
--- a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio.c
+++ b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio.c
@@ -94,6 +94,8 @@
 #include "sl_rail_util_ieee802154_fast_channel_switching_config.h"
 #endif // SL_CATALOG_RAIL_UTIL_IEEE802154_FAST_CHANNEL_SWITCHING_PRESENT
 
+#ifdef SL_CATALOG_WORF_PRESENT
+#include "wake-on-rf/magic_packet.h"
+#endif // SL_CATALOG_WORF_PRESENT
+
 //------------------------------------------------------------------------------
 // Enums, macros and static variables
 
@@ -1466,6 +1468,14 @@ void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
     otLogInfoPlat("PANID=%X index=%u IID=%d", aPanId, panIndex, iid);
     utilsSoftSrcMatchSetPanId(iid, aPanId);

+#ifdef SL_CATALOG_WORF_PRESENT 
+    if(aPanId != 0xFFFF){
+      static MagicPacketEnablePayload_t enablePayload_g;
+      enablePayload_g.panId = aPanId;
+      enablePayload_g.channel = 0;
+      enablePayload_g.borderRouter = true;
+      enableMagicPacketFilter(&enablePayload_g);
+    }
+#endif // SL_CATALOG_WORF_PRESENT
+
     status = RAIL_IEEE802154_SetPanId(gRailHandle, aPanId, panIndex);
     OT_ASSERT(status == RAIL_STATUS_NO_ERROR);
```

* `<sdk>/protocol/openthread/platform-abstraction/efr32/radio_extension.c`

```diff
diff --git a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio_extension.c b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio_extension.c
index 36aeceb..0c52ccc 100644
--- a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio_extension.c
+++ b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/efr32/radio_extension.c
@@ -67,6 +67,10 @@
 
 #endif // SL_CATALOG_OT_SIMULATION_PRESENT
 
+#ifdef SL_CATALOG_WORF_PRESENT
+#include "wake-on-rf/magic_packet.h"
+#endif // SL_CATALOG_WORF_PRESENT
+
 #include "common/code_utils.hpp"
 
 #ifdef SL_CATALOG_OPENTHREAD_ANT_DIV_PRESENT
@@ -521,3 +525,158 @@ otError otPlatRadioExtensionClearRadioCounters(void)
 }
 
 #endif // SL_CATALOG_OPENTHREAD_EFR32_EXT_PRESENT
+
+otError otPlatRadioExtensionGetWorfState(uint8_t *aWorfState)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    VerifyOrExit(aWorfState != NULL, error = OT_ERROR_INVALID_ARGS);
+    *aWorfState = (uint8_t)isMagicPacketFilterEnabled();
+#else
+    OT_UNUSED_VARIABLE(aWorfState);
+    ExitNow(error = OT_ERROR_NOT_IMPLEMENTED);
+#endif
+
+exit:
+    return error;
+}
+
+otError otPlatRadioExtensionGetWorfOptions(uint16_t *aWorfPanID, uint8_t *aWorfChannel, uint8_t *aWorfOptionsMask)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    VerifyOrExit(aWorfPanID != NULL, error = OT_ERROR_INVALID_ARGS);
+    VerifyOrExit(aWorfChannel != NULL, error = OT_ERROR_INVALID_ARGS);
+    VerifyOrExit(aWorfOptionsMask != NULL, error = OT_ERROR_INVALID_ARGS);
+  
+    MagicPacketEnablePayload_t worfOptions;
+
+    if(isMagicPacketFilterEnabled())
+    {
+        getMagicPacketFilterOptions(&worfOptions);
+
+        *aWorfPanID = worfOptions.panId;
+        *aWorfChannel = worfOptions.channel;
+        *aWorfOptionsMask = worfOptions.borderRouter;
+    } else 
+    {
+        error = OT_ERROR_INVALID_STATE;
+    }
+#else
+    OT_UNUSED_VARIABLE(aWorfPanID);
+    OT_UNUSED_VARIABLE(aWorfChannel);
+    OT_UNUSED_VARIABLE(aWorfOptionsMask);
+    ExitNow(error = OT_ERROR_NOT_IMPLEMENTED);
+#endif
+
+exit:
+    return error;
+}
+
+otError otPlatRadioExtensionGetWorfWakeTxOptions(uint8_t *aWorfTxFrameCounter, uint8_t *aWorfTtl, uint8_t *aWorfTxOptionsMask)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    VerifyOrExit(aWorfTxFrameCounter != NULL, error = OT_ERROR_INVALID_ARGS);
+    VerifyOrExit(aWorfTtl != NULL, error = OT_ERROR_INVALID_ARGS);
+    VerifyOrExit(aWorfTxOptionsMask != NULL, error = OT_ERROR_INVALID_ARGS);
+  
+    MagicPacketPayload_t worfTxOptions;
+
+    if(isMagicPacketFilterEnabled())
+    {
+        getMagicPacketFilterWakeTxOptions(&worfTxOptions);
+
+        *aWorfTxFrameCounter = worfTxOptions.frameCounter;
+        *aWorfTtl = worfTxOptions.timeToLive;
+        *aWorfTxOptionsMask = worfTxOptions.status;
+
+    } else 
+    {
+        error = OT_ERROR_INVALID_STATE;
+    }
+#else
+    OT_UNUSED_VARIABLE(aWorfTxFrameCounter);
+    OT_UNUSED_VARIABLE(aWorfTtl);
+    OT_UNUSED_VARIABLE(aWorfTxOptionsMask);
+    ExitNow(error = OT_ERROR_NOT_IMPLEMENTED);
+#endif
+
+exit:
+    return error;
+}
+
+otError otPlatRadioExtensionSetWorfState(uint8_t aWorfState)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    if(aWorfState)
+    {
+        MagicPacketEnablePayload_t enablePayload;
+
+        enablePayload.panId = 0xFFFF;
+        enablePayload.channel = 0x0B;
+        enablePayload.borderRouter = 0;
+
+        enableMagicPacketFilter(&enablePayload);
+    } else 
+    {
+        disableMagicPacketFilter();
+    }
+#else
+    OT_UNUSED_VARIABLE(aWorfState);
+    error = OT_ERROR_NOT_IMPLEMENTED;
+#endif
+    return error;
+}
+
+otError otPlatRadioExtensionSetWorfOptions(uint16_t aWorfPanID, uint8_t aWorfChannel, uint8_t aWorfOptionsMask)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    VerifyOrExit(((aWorfChannel >= 0x0B) &&  (aWorfChannel <= 0x1A)), error = OT_ERROR_INVALID_ARGS);
+  
+    MagicPacketEnablePayload_t worfOptions;//TODO: Check persistence after function call in pointers
+
+    if(isMagicPacketFilterEnabled())
+    {
+        disableMagicPacketFilter();
+    }
+
+    worfOptions.panId = aWorfPanID;
+    worfOptions.channel = aWorfChannel;
+    worfOptions.borderRouter = aWorfOptionsMask;
+  
+    enableMagicPacketFilter(&worfOptions);
+#else
+    OT_UNUSED_VARIABLE(aWorfPanID);
+    OT_UNUSED_VARIABLE(aWorfChannel);
+    OT_UNUSED_VARIABLE(aWorfOptionsMask);
+    ExitNow(error = OT_ERROR_NOT_IMPLEMENTED);
+#endif
+
+exit:
+    return error;
+}
+
+otError otPlatRadioExtensionSetWorfWakeTx(void)
+{
+    otError error = OT_ERROR_NONE;
+
+#ifdef SL_CATALOG_WORF_PRESENT
+    MagicPacketPayload_t worfTxPayload;
+
+    worfTxPayload.frameCounter = 0;
+    worfTxPayload.timeToLive = MAGIC_PACKET_DEFAULT_TTL;
+    worfTxPayload.status = 1;
+    error = (otError)sendMagicPacket(&worfTxPayload);
+#else
+    error = OT_ERROR_NOT_IMPLEMENTED;
+#endif
+    return error;
+}
```

* `<sdk>/protocol/openthread/platform-abstraction/efr32/radio_extension.h`

```diff
diff --git a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/radio_extension.h b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/radio_extension.h
index b7888b0..e48febd 100644
--- a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/radio_extension.h
+++ b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/radio_extension.h
@@ -680,6 +680,87 @@ otError otPlatRadioExtensionGetRadioCounters(efr32RadioCounters *aCounters);
  */
 otError otPlatRadioExtensionClearRadioCounters(void);
 
+/**
+ * Get the current state of the Wireless Coexistence Optimization and Responsiveness
+ * Framework (WORF) module.
+ *
+ * @param[out] aWorfState  Pointer to a variable to store the current WORF state.
+ *
+ * @retval OT_ERROR_NONE             The WORF state was successfully obtained.
+ * @retval OT_ERROR_INVALID_ARGS     The @p aWorfState pointer is NULL.
+ * @retval OT_ERROR_NOT_IMPLEMENTED  The WORF module is not supported.
+ */
+otError otPlatRadioExtensionGetWorfState(uint8_t *aWorfState);
+
+/**
+ * Get the current WORF (Wireless Coexistence Optimization and Responsiveness Framework) options.
+ *
+ * @param[out] aWorfPanID       Pointer to a variable to store the current WORF PAN ID.
+ * @param[out] aWorfChannel     Pointer to a variable to store the current WORF channel.
+ * @param[out] aWorfOptionsMask Pointer to a variable to store the current WORF options mask.
+ *
+ * @retval OT_ERROR_NONE             The WORF options were successfully obtained.
+ * @retval OT_ERROR_INVALID_STATE    The WORF module is not initialized.
+ * @retval OT_ERROR_INVALID_ARGS     One or more of the output pointers are NULL.
+ * @retval OT_ERROR_NOT_IMPLEMENTED  The WORF module is not supported.
+ */
+otError otPlatRadioExtensionGetWorfOptions(uint16_t *aWorfPanID, uint8_t *aWorfChannel, uint8_t *aWorfOptionsMask);
+
+/**
+ * Get the current WORF (Wireless Coexistence Optimization and Responsiveness Framework) TX options.
+ *
+ * @param[out] aWorfTxFrameCounter  Pointer to a variable to store the current WORF TX frame counter.
+ * @param[out] aWorfTtl             Pointer to a variable to store the current WORF TX TTL (Time To Live).
+ * @param[out] aWorfTxOptionsMask   Pointer to a variable to store the current WORF TX options mask.
+ *
+ * @retval OT_ERROR_NONE             The WORF TX options were successfully obtained.
+ * @retval OT_ERROR_INVALID_STATE    The WORF module is not initialized.
+ * @retval OT_ERROR_INVALID_ARGS     One or more of the output pointers are NULL.
+ * @retval OT_ERROR_NOT_IMPLEMENTED  The WORF module is not supported.
+ */
+otError otPlatRadioExtensionGetWorfWakeTxOptions(uint8_t *aWorfTxFrameCounter, uint8_t *aWorfTtl, uint8_t *aWorfTxOptionsMask);
+
+/**
+ * Set the current state of the Wireless Coexistence Optimization and Responsiveness
+ * Framework (WORF) module.
+ *
+ * @param[in] aWorfState  The new WORF state to set.
+ *
+ * @retval OT_ERROR_NONE             The WORF state was successfully set.
+ * @retval OT_ERROR_NOT_IMPLEMENTED  The WORF module is not supported.
+ */
+otError otPlatRadioExtensionSetWorfState(uint8_t aWorfState);
+
+/**
+ * Set the current WORF (Wireless Coexistence Optimization and Responsiveness Framework) options.
+ * Call is always overriden by otPlatRadioSetPanId in this implementation due to 
+ *     if(aPanId != 0xFFFF){
+      static MagicPacketEnablePayload_t enablePayload_g;
+      enablePayload_g.panId = aPanId;
+      enablePayload_g.channel = 0;
+      enablePayload_g.borderRouter = true;
+      enableMagicPacketFilter(&enablePayload_g);
+    }
+ *
+ * @param[in] aWorfPanID       The new WORF PAN ID to set. Must be aligned with OT Pan ID to work correctly.
+ * @param[in] aWorfChannel     The new WORF channel to set, overriden by OT Channel anyway.
+ * @param[in] aWorfOptionsMask The new WORF options mask to set.
+ *
+ * @retval OT_ERROR_NONE             The WORF options were successfully set.
+ * @retval OT_ERROR_NOT_IMPLEMENTED  The WORF module is not supported.
+ */
+otError otPlatRadioExtensionSetWorfOptions(uint16_t aWorfPanID, uint8_t aWorfChannel, uint8_t aWorfOptionsMask);
+
+/**
+ * Set the current WORF (Wireless Coexistence Optimization and Responsiveness Framework) TX options.
+ *
+ * This function sets the current WORF TX options, including the TX frame counter, TTL, and options mask.
+ *
+ * @retval OT_ERROR_NONE             The WORF TX options were successfully set.
+ * @retval MAGIC_PACKET_ERROR        Any error code from the worf module.
+ */
+otError otPlatRadioExtensionSetWorfWakeTx(void);
+
 /**
  * @}
  *
```

* `<sdk>/protocol/openthread/platform-abstraction/include/vendor_spinel.hpp`

```diff
diff --git a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/vendor_spinel.hpp b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/vendor_spinel.hpp
index 6c4a22e..95df1e9 100644
--- a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/vendor_spinel.hpp
+++ b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/include/vendor_spinel.hpp
@@ -36,64 +36,85 @@
 
 #include "lib/spinel/spinel.h"
 
-namespace ot {
-namespace Spinel {
-namespace Vendor {
-
-enum
+namespace ot
 {
-    SPINEL_PROP_VENDOR_ANTENNA = (SPINEL_PROP_VENDOR__BEGIN + 0),
-    SPINEL_PROP_VENDOR_COEX    = (SPINEL_PROP_VENDOR__BEGIN + 1),
-    SPINEL_PROP_VENDOR_TEST    = (SPINEL_PROP_VENDOR__BEGIN + 2),
-    SPINEL_PROP_VENDOR_EFR32   = (SPINEL_PROP_VENDOR__BEGIN + 3),
-};
-} // namespace Vendor
-} // namespace Spinel
+    namespace Spinel
+    {
+        namespace Vendor
+        {
 
-namespace Vendor {
-namespace Antenna {
-enum
-{
-    ANT_TX_MODE_COMMAND,
-    ANT_RX_MODE_COMMAND,
-    ANT_ACTIVE_PHY_COMMAND,
-};
+            enum
+            {
+                SPINEL_PROP_VENDOR_ANTENNA = (SPINEL_PROP_VENDOR__BEGIN + 0),
+                SPINEL_PROP_VENDOR_COEX = (SPINEL_PROP_VENDOR__BEGIN + 1),
+                SPINEL_PROP_VENDOR_TEST = (SPINEL_PROP_VENDOR__BEGIN + 2),
+                SPINEL_PROP_VENDOR_EFR32 = (SPINEL_PROP_VENDOR__BEGIN + 3),
+                SPINEL_PROP_VENDOR_WORF = (SPINEL_PROP_VENDOR__BEGIN + 4),
+            };
+        } // namespace Vendor
+    } // namespace Spinel
 
-} // namespace Antenna
+    namespace Vendor
+    {
+        namespace Antenna
+        {
+            enum
+            {
+                ANT_TX_MODE_COMMAND,
+                ANT_RX_MODE_COMMAND,
+                ANT_ACTIVE_PHY_COMMAND,
+            };
 
-namespace Coex {
-enum
-{
-    COEX_DP_STATE_COMMAND,
-    COEX_GPIO_INPUT_OVERRIDE_COMMAND,
-    COEX_ACTIVE_RADIO_COMMAND,
-    COEX_PHY_SELECT_TIMEOUT_COMMAND,
-    COEX_PTA_OPTIONS_COMMAND,
-    COEX_CONSTANT_OPTIONS_COMMAND,
-    COEX_PTA_STATE_COMMAND,
-    COEX_PWM_STATE_COMMAND,
-    COEX_COUNTERS_COMMAND,
-    COEX_RADIO_HOLDOFF_COMMAND,
-};
+        } // namespace Antenna
 
-} // namespace Coex
+        namespace Coex
+        {
+            enum
+            {
+                COEX_DP_STATE_COMMAND,
+                COEX_GPIO_INPUT_OVERRIDE_COMMAND,
+                COEX_ACTIVE_RADIO_COMMAND,
+                COEX_PHY_SELECT_TIMEOUT_COMMAND,
+                COEX_PTA_OPTIONS_COMMAND,
+                COEX_CONSTANT_OPTIONS_COMMAND,
+                COEX_PTA_STATE_COMMAND,
+                COEX_PWM_STATE_COMMAND,
+                COEX_COUNTERS_COMMAND,
+                COEX_RADIO_HOLDOFF_COMMAND,
+            };
 
-namespace Test {
-enum
-{
-    GEN_PTI_RADIO_CONFIG_COMMAND,
-    GEN_CCA_MODE_COMMAND,
-};
+        } // namespace Coex
 
-} // namespace Test
+        namespace Test
+        {
+            enum
+            {
+                GEN_PTI_RADIO_CONFIG_COMMAND,
+                GEN_CCA_MODE_COMMAND,
+            };
 
-namespace Efr32 {
-enum
-{
-    EFR32_RADIO_COUNTERS_COMMAND,
-};
+        } // namespace Test
+
+        namespace Efr32
+        {
+            enum
+            {
+                EFR32_RADIO_COUNTERS_COMMAND,
+            };
+
+        } // namespace Efr32
+
+        namespace Worf
+        {
+            enum
+            {
+                WORF_STATE_COMMAND,
+                WORF_OPTIONS_COMMAND,
+                WORF_WAKE_TX_OPTIONS_COMMAND,
+                WORF_WAKE_TX_COMMAND,
+            };
+        } // namespace Worf
 
-} // namespace Efr32
-} // namespace Vendor
+    } // namespace Vendor
 } // namespace ot
 #endif // _NCP_SPINEL_HPP
```

* `<sdk>/protocol/openthread/platform-abstraction/ncp/ncp_dispatcher.cpp`

```diff
diff --git a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/ncp/ncp_dispatcher.cpp b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/ncp/ncp_dispatcher.cpp
index 3e4d341..4d64d17 100644
--- a/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/ncp/ncp_dispatcher.cpp
+++ b/gecko_sdk_4.4.3/protocol/openthread/platform-abstraction/ncp/ncp_dispatcher.cpp
@@ -54,6 +54,10 @@
 #include "ncp_efr32.hpp"
 #endif // SL_CATALOG_OPENTHREAD_EFR32_CLI_PRESENT
 
+#ifdef SL_CATALOG_WORF_PRESENT
+#include "ncp_worf.hpp"
+#endif // SL_CATALOG_WORF_PRESENT
+
 #if OPENTHREAD_ENABLE_NCP_VENDOR_HOOK
 
 namespace SpinelProp    = ot::Spinel::Vendor;
@@ -130,6 +134,13 @@ otError NcpBase::VendorGetPropertyHandler(spinel_prop_key_t aPropKey)
         error = Vendor::Efr32::getEfr32Property(mDecoder, mEncoder);
         break;
     #endif // SL_CATALOG_OPENTHREAD_EFR32_CLI_PRESENT
+
+    #ifdef SL_CATALOG_WORF_PRESENT
+    case SpinelProp::SPINEL_PROP_VENDOR_WORF:
+        error = Vendor::Worf::getWorfProperty(mDecoder, mEncoder);
+        break;
+    #endif // SL_CATALOG_WORF_PRESENT
+
     default:
         error = OT_ERROR_NOT_FOUND;
         break;
@@ -180,6 +191,13 @@ otError NcpBase::VendorSetPropertyHandler(spinel_prop_key_t aPropKey)
         error = Vendor::Efr32::setEfr32Property(mDecoder);
         break;
     #endif // SL_CATALOG_OPENTHREAD_EFR32_CLI_PRESENT
+
+    #ifdef SL_CATALOG_WORF_PRESENT
+    case SpinelProp::SPINEL_PROP_VENDOR_WORF:
+        error = Vendor::Worf::setWorfProperty(mDecoder);
+        break;
+    #endif // SL_CATALOG_WORF_PRESENT
+
     default:
         error = OT_ERROR_NOT_FOUND;
         break;
```

* `<sdk>/util/third_party/openthread/src/ncp/ncp_base_radio.cpp`

```diff
diff --git a/gecko_sdk_4.4.3/util/third_party/openthread/src/ncp/ncp_base_radio.cpp b/gecko_sdk_4.4.3/util/third_party/openthread/src/ncp/ncp_base_radio.cpp
index 4a238f1..f4ea36c 100644
--- a/gecko_sdk_4.4.3/util/third_party/openthread/src/ncp/ncp_base_radio.cpp
+++ b/gecko_sdk_4.4.3/util/third_party/openthread/src/ncp/ncp_base_radio.cpp
@@ -43,6 +43,8 @@
 #include "instance/instance.hpp"
 #include "mac/mac_frame.hpp"
 
+#include "wake-on-rf/magic_packet.h"
+
 #if OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE
 
 namespace ot {
@@ -136,6 +138,8 @@ void NcpBase::LinkRawReceiveDone(otRadioFrame *aFrame, otError aError)
 
     header |= SPINEL_HEADER_IID(aFrame->mIid);
 
+    decodeMagicPacket(aFrame->mPsdu);
+
     // Append frame header
     SuccessOrExit(mEncoder.BeginFrame(header, SPINEL_CMD_PROP_VALUE_IS, SPINEL_PROP_STREAM_RAW));
 
```

### Bootloader support

XMODEM Bootloader is supported natively
