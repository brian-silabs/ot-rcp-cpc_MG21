/*
 *  Copyright (c) 2023, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file
 * @brief This file contains definitions for a spinel extension to support wake on rf commands.
 */

#include "ncp_coex.hpp"
#include "vendor_spinel.hpp"
#include "radio_extension.h"

#include "common/code_utils.hpp"

namespace WorfCmd = ot::Vendor::Worf;

namespace ot
{
    namespace Ncp
    {
        namespace Vendor
        {
            namespace Worf
            {

                static otError getWorfState(Spinel::Encoder &aEncoder);
                static otError setWorfState(Spinel::Decoder &aDecoder);

                static otError getWorfOptions(Spinel::Encoder &aEncoder);
                static otError setWorfOptions(Spinel::Decoder &aDecoder);

                static otError getWorfWakeTxOptions(Spinel::Encoder &aEncoder);

                static otError setWorfWakeTx(Spinel::Decoder &aDecoder);

                otError getWorfProperty(Spinel::Decoder &aDecoder, Spinel::Encoder &aEncoder)
                {
                    otError error = OT_ERROR_NOT_FOUND;
                    uint8_t cmdKey;

                    SuccessOrExit(aDecoder.ReadUint8(cmdKey));

                    switch (cmdKey)
                    {
                    case WorfCmd::WORF_STATE_COMMAND:
                        error = getWorfState(aEncoder);
                        break;
                    case WorfCmd::WORF_OPTIONS_COMMAND:
                        error = getWorfOptions(aEncoder);
                        break;
                    case WorfCmd::WORF_WAKE_TX_OPTIONS_COMMAND:
                        error = getWorfWakeTxOptions(aEncoder);
                        break;
                    }

                exit:
                    return error;
                }

                otError setWorfProperty(Spinel::Decoder &aDecoder)
                {
                    otError error = OT_ERROR_NOT_FOUND;
                    uint8_t cmdKey;

                    SuccessOrExit(aDecoder.ReadUint8(cmdKey));

                    switch (cmdKey)
                    {
                    case WorfCmd::WORF_STATE_COMMAND:
                        error = setWorfState(aDecoder);
                        break;
                    case WorfCmd::WORF_OPTIONS_COMMAND:
                        error = setWorfOptions(aDecoder);
                        break;
                    case WorfCmd::WORF_WAKE_TX_COMMAND:
                        error = setWorfWakeTx(aDecoder);
                        break;
                    }

                exit:
                    return error;
                }

                otError getWorfState(Spinel::Encoder &aEncoder)
                {
                    uint8_t worfState = 0;

                    IgnoreError(otPlatRadioExtensionGetWorfState(&worfState));

                    return (aEncoder.WriteUint8(worfState));
                }

                otError getWorfOptions(Spinel::Encoder &aEncoder)
                {
                    uint16_t worfPanId = 0;
                    uint8_t worfChannel = 0;
                    uint8_t worfOptionsMask = 0;

                    IgnoreError(otPlatRadioExtensionGetWorfOptions(&worfPanId, &worfChannel, &worfOptionsMask));

                    return (aEncoder.WriteUint8(dpPulse)); // TODO
                }

                otError getWorfWakeTxOptions(Spinel::Encoder &aEncoder)
                {
                    uint8_t worfTxFrameCounter = 0;
                    uint8_t worfTxTtl = 0;
                    uint8_t aWorfTxOptionsMask = 0;

                    IgnoreError(otPlatRadioExtensionGetWorfWakeTxOptions(&worfTxFrameCounter, &worfTxTtl, &aWorfTxOptionsMask));

                    return (aEncoder.WriteUint8(dpPulse)); // TODO
                }

                otError setWorfState(Spinel::Decoder &aDecoder)
                {
                    uint8_t worfState = 0;
                    otError error = OT_ERROR_NONE;

                    SuccessOrExit(error = aDecoder.ReadUint8(worfState));

                    error = otPlatRadioExtensionSetWorfState(worfState);

                exit:
                    return error;
                }

                otError setWorfOptions(Spinel::Decoder &aDecoder)
                {
                    uint16_t worfPanId = 0;
                    uint8_t worfChannel = 0;
                    uint8_t worfOptionsMask = 0;
                    otError error = OT_ERROR_NONE;

                    SuccessOrExit(error = aDecoder.ReadUint8(dpPulse)); // TODO

                    error = otPlatRadioExtensionSetWorfOptions(worfPanId, worfChannel, worfOptionsMask);

                exit:
                    return error;
                }

                otError setWorfWakeTx(Spinel::Decoder &aDecoder)
                {
                    otError error = OT_ERROR_NONE;

                    error = otPlatRadioExtensionSetWorfWakeTx();

                exit:
                    return error;
                }

            } // namespace Worf
        } // namespace Vendor
    } // namespace Ncp
} // namespace ot
