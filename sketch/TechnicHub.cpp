/*  This library allows to create and convert received messages
    from a LEGO Technic Hub with integrated BLE
    Copyright (C) 2020  hoharald 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "TechnicHub.h"

TechnicHubMessageBuilder::TechnicHubMessageBuilder() {
    memset(TechnicHubMessageBuilder::buffer, 0, sizeof(TechnicHubMessageBuilder::buffer));
};

uint8_t TechnicHubMessageBuilder::createMessage(TechnicHubMessageType messageType, uint8_t size) {
    TechnicHubMessageBuilder::_length = size + 3;
    TechnicHubMessageBuilder::buffer[0] = TechnicHubMessageBuilder::_length;
    TechnicHubMessageBuilder::buffer[1] = HUB_ID;
    TechnicHubMessageBuilder::buffer[2] = static_cast<uint8_t>(messageType);            
    return TechnicHubMessageBuilder::_length;
};

uint8_t TechnicHubMessageBuilder::createPortInformationRequest(uint8_t portId, TechnicHubPortInformationType informationType) {
    TechnicHubMessageBuilder::buffer[3] = portId;
    TechnicHubMessageBuilder::buffer[4] = (uint8_t)informationType;
    return createMessage(TechnicHubMessageType::portInformationRequest, 2);
};

uint8_t TechnicHubMessageBuilder::createPortModeInformationRequest(uint8_t portId, uint8_t mode, TechnicHubPortModeInformationType informationType) {
    TechnicHubMessageBuilder::buffer[3] = portId;
    TechnicHubMessageBuilder::buffer[4] = mode;
    TechnicHubMessageBuilder::buffer[5] = static_cast<uint8_t>(informationType);
    return createMessage(TechnicHubMessageType::portModeInformationRequest, 3);
};

uint8_t TechnicHubMessageBuilder::createPortInputFormatSetupSingle(uint8_t portId, uint8_t mode, uint32_t deltaInterval, boolean notificationEnabled) {
    TechnicHubMessageBuilder::buffer[3] = portId;
    TechnicHubMessageBuilder::buffer[4] = mode;
    memcpy(&buffer[5], &deltaInterval, 4);
    TechnicHubMessageBuilder::buffer[9] = notificationEnabled ? 0x01 : 0x00;
    return createMessage(TechnicHubMessageType::portInputFormatSetupSingle, 7);
};

uint8_t TechnicHubMessageBuilder::createVirtualPortConnect(uint8_t portIdA, uint8_t portIdB) {
    TechnicHubMessageBuilder::buffer[3] = 0x01;
    TechnicHubMessageBuilder::buffer[4] = portIdA;
    TechnicHubMessageBuilder::buffer[5] = portIdB;
    return createMessage(TechnicHubMessageType::virtualPortSetup, 3);
};

uint8_t TechnicHubMessageBuilder::createVirtualPortDisconnect(uint8_t portId) {
    TechnicHubMessageBuilder::buffer[3] = 0x00;
    TechnicHubMessageBuilder::buffer[4] = portId;
    return createMessage(TechnicHubMessageType::virtualPortSetup, 2);
}

uint8_t TechnicHubMessageBuilder::createPortOutput(uint8_t portId, boolean executeImmediately, boolean sendFeedback, TechnicHubPortOutputSubCommand subCommand, uint8_t size) {
    uint8_t s = executeImmediately ? 16 : 0;
    uint8_t c = sendFeedback ? 1 : 0;
    TechnicHubMessageBuilder::buffer[3] = portId;
    TechnicHubMessageBuilder::buffer[4] = s + c;
    TechnicHubMessageBuilder::buffer[5] = (uint8_t)subCommand;
    return createMessage(TechnicHubMessageType::portOutputCommand, 3 + size);
};

uint8_t TechnicHubMessageBuilder::createPortOutputStartPower(uint8_t portId, int8_t power) {
    TechnicHubMessageBuilder::buffer[6] = power;
    TechnicHubMessageBuilder::buffer[7] = power;
    return createPortOutput(portId, true, false, TechnicHubPortOutputSubCommand::startPower2, 2);
}

uint8_t TechnicHubMessageBuilder::_getUseProfileUint8(boolean useAccelerationProfile, boolean useDecelerationProfile) {
    uint8_t acc = useAccelerationProfile ? 2 : 0;
    uint8_t dec = useDecelerationProfile ? 1 : 0;
    return acc + dec;
}

uint8_t TechnicHubMessageBuilder::createPortOutputStartSpeed(uint8_t portId, int8_t speed, int8_t maxPower, boolean useAccelerationProfile, boolean useDecelerationProfile) {
    uint8_t profile = _getUseProfileUint8(useAccelerationProfile, useDecelerationProfile);
    TechnicHubMessageBuilder::buffer[6] = speed;
    TechnicHubMessageBuilder::buffer[7] = maxPower;
    TechnicHubMessageBuilder::buffer[8] = profile;
    return createPortOutput(portId, true, false, TechnicHubPortOutputSubCommand::startSpeed, 3);
};

uint8_t TechnicHubMessageBuilder::createPortOutputGoToAbsolutePosition(uint8_t portId, int32_t position, int8_t speed, int8_t maxPower, TechnicHubPortOutputCommandEndState endState, boolean useAccelerationProfile, boolean useDecelerationProfile) {
    uint8_t profile = _getUseProfileUint8(useAccelerationProfile, useDecelerationProfile);    
    memcpy(&buffer[6], &position, 4);
    TechnicHubMessageBuilder::buffer[10] = speed;
    TechnicHubMessageBuilder::buffer[11] = maxPower;
    TechnicHubMessageBuilder::buffer[12] = (uint8_t)endState;
    TechnicHubMessageBuilder::buffer[13] = profile;
    return createPortOutput(portId, true, false, TechnicHubPortOutputSubCommand::gotoAbsolutePosition, 8);
};

 TechnicHubMessageHeader TechnicHubMessageBuilder::_convertHeader() {
    return TechnicHubMessageHeader(TechnicHubMessageBuilder::buffer[0], TechnicHubMessageBuilder::buffer[1], static_cast<TechnicHubMessageType>(TechnicHubMessageBuilder::buffer[2]));
};

/**********************************************/
/* The following methods are work in progress */
/**********************************************/

boolean* TechnicHubMessageBuilder::_getPortModes(uint16_t modes) {
    // const result: boolean[] = [];
    // result.push((modes & 1) > 0);
    // result.push((modes & 2) > 0);
    // result.push((modes & 4) > 0);
    // result.push((modes & 8) > 0);
    // result.push((modes & 16) > 0);
    // result.push((modes & 32) > 0);
    // result.push((modes & 64) > 0);
    // result.push((modes & 128) > 0);
    // result.push((modes & 256) > 0);
    // result.push((modes & 512) > 0);
    // result.push((modes & 1024) > 0);
    // result.push((modes & 2048) > 0);
    // result.push((modes & 4096) > 0);
    // result.push((modes & 8192) > 0);
    // result.push((modes & 16384) > 0);
    // result.push((modes & 32768) > 0);
    // return result;
    boolean result[1] = {false};
    return result;
};

TechnicHubPortModes TechnicHubMessageBuilder::_convertPortMode(TechnicHubPortInformation message) {
    // const capabilities = data.getUint8(5);
    // const totalModeCount = data.getUint8(6);
    // const inputModes = data.getUint16(7, true);
    // const outputModes = data.getUint16(9, true);
    // const result = new TechnicHubPortModes(message);
    // result.capabilities = {} as ITechnicHubPortCapabilities;
    // result.capabilities.output = (capabilities & 0x01) > 0;
    // result.capabilities.input = (capabilities & 0x02) > 0;
    // result.capabilities.logicalCombinable = (capabilities & 0x04) > 0;
    // result.capabilities.logicalSynchronizable = (capabilities & 0x08) > 0;

    // result.totalModeCount = totalModeCount;

    // result.inputModes = this._getPortModes(inputModes);
    // result.outputModes = this._getPortModes(outputModes);
    return TechnicHubPortModes();
};

TechnicHubPortModeCombinations TechnicHubMessageBuilder::_convertPortModeCombinations(TechnicHubPortInformation message) {
    // const result = new TechnicHubPortModeCombinations(message);
    // result.combinations = [] as  Array<boolean[]>;
    // for (let i = 0; i < data.byteLength - 5; i += 2) {
    //     const modes = data.getUint16(5 + i, true);
    //     result.combinations.push(this._getPortModes(modes));
    // }
    return TechnicHubPortModeCombinations(TechnicHubPortModeCombinations(message));
};

// static convertPortInformation(data: DataView):
// ITechnicHubPortModes |
// ITechnicHubPortModeCombinations {
//     const header = this._convertHeader(data);
//     const message = header as ITechnicHubPortInformation;
//     message.portId = data.getUint8(3);
//     message.informationType = data.getUint8(4) as TechnicHubPortInformationType;
//     switch (message.informationType) {
//         case TechnicHubPortInformationType.modeInformation:
//             return this._convertPortMode(message, data);
//         case TechnicHubPortInformationType.possibleModeCombinations:
//             return this._convertPortModeCombinations(message, data);
//         default:
//             return null;
//     }
// }

// private static _getPortMapping(mapping: number): ITechnicHubPortModeInformationMapping {
//     const result = {} as ITechnicHubPortModeInformationMapping;
//     result.outputMapping.dis = ((mapping & 4) > 0);
//     result.outputMapping.rel = ((mapping & 8) > 0);
//     result.outputMapping.abs = ((mapping & 16) > 0);
//     result.outputMapping.supportsFunctionalMapping20 = ((mapping & 64) > 0);
//     result.outputMapping.supportsNullValue = ((mapping & 128) > 0);

//     result.inputMapping.dis = ((mapping & 1024) > 0);
//     result.inputMapping.rel = ((mapping & 2048) > 0);
//     result.inputMapping.abs = ((mapping & 4096) > 0);
//     result.inputMapping.supportsFunctionalMapping20 = ((mapping & 16384) > 0);
//     result.inputMapping.supportsNullValue = ((mapping & 32768) > 0);
//     return result;
// }

// static _convertPortModeInformationName(message: ITechnicHubPortModeInformation, data: DataView): TechnicHubPortModeInformationName {
//     const result = new TechnicHubPortModeInformationName(message);
//     const buffer = new ArrayBuffer(data.byteLength - 6);
//     const array = new Uint8Array(buffer);
//     array.set(new Uint8Array(data.buffer.slice(6)));
//     result.name = new TextDecoder('utf-8').decode(array);
//     return result;
// }

// static _convertPortModeInformationRaw(message: ITechnicHubPortModeInformation, data: DataView): TechnicHubPortModeInformationRaw {
//     const result = new TechnicHubPortModeInformationRaw(message);
//     result.rawMin = data.getFloat32(6, true);
//     result.rawMax = data.getFloat32(10, true);
//     return result;
// }

// static convertPortModeInformationPct(message: ITechnicHubPortModeInformation, data: DataView): TechnicHubPortModeInformationPct {
//     const result = new TechnicHubPortModeInformationPct(message);
//     result.pctMin = data.getFloat32(6, true);
//     result.pctMax = data.getFloat32(10, true);
//     return result;
// }

// static _convertPortModeInformationSi(message: ITechnicHubPortModeInformation, data: DataView): TechnicHubPortModeInformationSi {
//     const result = new TechnicHubPortModeInformationSi(message);
//     result.siMin = data.getFloat32(6, true);
//     result.siMax = data.getFloat32(10, true);
//     return result;
// }

// static _convertPortModeInformationSymbol(message: ITechnicHubPortModeInformation, data: DataView): TechnicHubPortModeInformationSymbol {
//     const result = new TechnicHubPortModeInformationSymbol(message);
//     const buffer = new ArrayBuffer(data.byteLength - 6);
//     const array = new Uint8Array(buffer);
//     array.set(new Uint8Array(data.buffer.slice(6)));
//     result.symbol = new TextDecoder('utf-8').decode(array);
//     return result;
// }

//     static convertPortModeInformation(data: DataView):
//     ITechnicHubPortModeInformationName |
//     ITechnicHubPortModeInformationRaw |
//     ITechnicHubPortModeInformationPtc |
//     ITechnicHubPortModeInformationSi |
//     ITechnicHubPortModeInformationSymbol |
//     ITechnicHubPortModeInformationMapping |
//     ITechnicHubPortModeInformationMotorBias |
//     ITechnicHubPortModeInformationValueFormat {
//         const header = this._convertHeader(data);
//         const message = header as ITechnicHubPortModeInformation;
//         message.portId = data.getUint8(3);
//         message.mode = data.getUint8(4);
//         message.informationType = data.getUint8(5) as TechnicHubPortModeInformationType;
//         switch (message.informationType) {
//             case TechnicHubPortModeInformationType.name:
//                 return this._convertPortModeInformationName(message, data);
//             case TechnicHubPortModeInformationType.raw:
//                 return this._convertPortModeInformationRaw(message, data);
//             case TechnicHubPortModeInformationType.pct:
//                 return this.convertPortModeInformationPct(message, data);
//             case TechnicHubPortModeInformationType.si:
//                 return this._convertPortModeInformationSi(message, data);
//             case TechnicHubPortModeInformationType.symbol:
//                 return this._convertPortModeInformationSymbol(message, data);
//             // case TechnicHubPortModeInformationType.mapping:
//             //     return this.convertPortModeInformationMapping(message, data);
//             // case TechnicHubPortModeInformationType.motorBias:
//             //     return this.convertPortModeInformationMotorBias(message, data);
//             // case TechnicHubPortModeInformationType.capabilityBits:
//             //     return this.convertPortModeInformationCapabilityBits(message, data);
//             // case TechnicHubPortModeInformationType.valueFormat:
//             //     return this.convertPortModeInformationValueFormat(message, data);
//             default:
//                 return null;
//         }
//     }
// }
