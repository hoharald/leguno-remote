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
    
#ifndef _TECHNIC_HUB_H_
#define _TECHNIC_HUB_H_

#include <Arduino.h>

enum class TechnicHubValueType: uint8_t {
    _n8bit = 0x00,
    _n16bit = 0x01,
    _n32bit = 0x02,
    _float = 0x03
};

enum class TechnicHubPortOutputCommandEndState: uint8_t {
    _float = 0,
    _hold = 126,
    _break = 127
};

enum class TechnicHubMessageType: uint8_t {
    hubProperties = 0x01,
    hubActions = 0x02,
    hubAlert = 0x03,
    hubAttachedIO = 0x04,
    genericErrorMessage = 0x05,
    hwNetworkCommands = 0x08,
    fwUpdateGoIntoBootMode = 0x10,
    fwUpdateLockMemory = 0x11,
    fwUpdateLockStatusRequest = 0x12,
    fwLockStatus = 0x13,
    portInformationRequest = 0x21,
    portModeInformationRequest = 0x22,
    portInputFormatSetupSingle = 0x41,
    portInputFormatSetupCombinedMode = 0x42,
    portInformation = 0x43,
    portModeInformation = 0x44,
    portValueSingle = 0x45,
    portValueCombinedMode = 0x46,
    portInputFormatSingle = 0x47,
    portInputFormatCombinedMode = 0x48,
    virtualPortSetup = 0x61,
    portOutputCommand = 0x81,
    portOutputCommandFeedback = 0x82
};

enum class TechnicHubPortOutputSubCommand: uint8_t {
    startPower2 = 0x02,
    setAccTime = 0x05,
    setDecTime = 0x06,
    startSpeed = 0x07,
    startSpeed2 = 0x08,
    startSpeedForTime = 0x09,
    startSpeedForTime2 = 0x0A,
    startSpeedForDegrees = 0x0B,
    startSpeedForDegrees2 = 0x0C,
    gotoAbsolutePosition = 0x0D,
    gotoAbsolutePosition2 = 0x0E,
    presetEncoder2 = 0x14
};

enum class TechnicHubAlertType: uint8_t {
    lowVoltage = 0x01,
    highCurrent = 0x02,
    lowSignalStrength = 0x03,
    overPowerCondition = 0x04
};

enum class TechnicHubAlertOperation: uint8_t {
    enableUpdates = 0x00,
    alert = 0xFF
};

enum class TechnicHubAlertPayload: uint8_t {
    statusOk = 0x00,
    disableUpdates = 0x02,
    requestUpdates = 0x03,
    update = 0x04
};

enum class TechnicHubErrorCodes: uint8_t {
    ack = 0x00,
    mack = 0x02,
    bufferOverflow = 0x03,
    timeout = 0x04,
    commandNotRecognized = 0x05,
    invalidUse = 0x06,
    overCurrent = 0x07,
    internalError = 0x08,
};

enum class TechnicHubPortType: uint8_t {
    motor = 0x0001,
    systemTrainMotor = 0x0002,
    button = 0x0005,
    ledLight = 0x0008,
    voltage = 0x0014,
    current = 0x0015,
    piezoTone = 0x0016,
    rgbLight = 0x0017,
    externalTiltSensor = 0x0022,
    motionSensor = 0x0023,
    visionSensor = 0x0025,
    externalMotorWithTacho = 0x0026,
    internalMotorWithTacho = 0x0027,
    internalTilt = 0x0028
};

enum class TechnicHubPortInformationType: uint8_t {
    portValue = 0x00,
    modeInformation = 0x01,
    possibleModeCombinations = 0x02
};

enum class TechnicHubPortModeInformationType: uint8_t {
    name = 0x00,
    raw = 0x01,
    pct = 0x02,
    si = 0x03,
    symbol = 0x04,
    mapping = 0x05,
    internal = 0x06,
    motorBias = 0x07,
    capabilityBits = 0x08,
    valueFormat = 0x80
};

enum class TechnicHubPortEvent: uint8_t {
    detached = 0x00,
    attached = 0x01,
    attachedVirtual = 0x02
};

enum class TechnicHubState: uint8_t {
    idle = 0,
    virtualPortSetup = 10,
    waitForIoAttachedEvents = 20,
    portInformationRequest = 30,
    waitForPortInformation = 40,
    portModeCombinationRequest = 50,
    waitForPortModeCombination = 60,
    portModeInformationRequest = 70,
    waitForPortModeInformation = 80,
    portSetup = 90,
    notificationSetup = 100,
};


// struct TechnicHubPort {
//     byte portId;
//     TechnicHubPortEvent event;
//     TechnicHubPortType type;
//     byte hardwareRevision;
//     byte softwareRevision;
//     byte portIdA;
//     byte portIdB;
//     TechnicHubPortModes modes;
//     TechnicHubPortModeCombinations modeCombinations;
//     TechnicHubPortModeViewModel modeInformation[];
// };

// class TechnicHubPortModeViewModel {
//     byte mode;
//     String name;
//     String symbol;
//     int rawMin;
//     int rawMax;
//     int pctMin;
//     int pctMax;
//     int siMin;
//     int siMax;
// };

class TechnicHubMessageHeader {
    public:
        TechnicHubMessageHeader(uint8_t length, uint8_t hubId, TechnicHubMessageType messageType);
        virtual ~TechnicHubMessageHeader();
        uint8_t length;
        uint8_t hubId;
        TechnicHubMessageType messageType;
};

class TechnicHubPortInformationRequest : public TechnicHubMessageHeader {
    public:
        uint8_t portId;
        TechnicHubPortInformationType informationType;
};

class TechnicHubPortInformation : public TechnicHubMessageHeader {
    public:
        TechnicHubPortInformation(uint8_t portId);
        uint8_t portId;
        TechnicHubPortInformationType informationType;
};

class TechnicHubPortCapabilities {
    public:
        boolean output;
        boolean input;
        boolean logicalCombinable;
        boolean logicalSynchronizable;
};

class TechnicHubPortModes : public TechnicHubPortInformation {
    public:
        TechnicHubPortModes();
        TechnicHubPortCapabilities capabilities;
        uint8_t totalModeCount;
        boolean inputModes[16];
        boolean outputModes[16];

        TechnicHubPortModes(TechnicHubPortInformation head);
};

class TechnicHubPortModeCombinations : public TechnicHubPortInformation {
    public:
        boolean combinations[16];
        TechnicHubPortModeCombinations(TechnicHubPortInformation head);
};

class TechnicHubPortModeInformation : public TechnicHubMessageHeader {
    public:
        uint8_t portId;
        uint8_t mode;
        TechnicHubPortModeInformationType informationType;
};

class TechnicHubPortModeInformationName : public TechnicHubPortModeInformation {
    public:
        String name;
        TechnicHubPortModeInformationName(TechnicHubPortModeInformation head);
};

class TechnicHubPortModeInformationRaw : public TechnicHubPortModeInformation {
    public:
        uint8_t rawMin;
        uint8_t rawMax;
        TechnicHubPortModeInformationRaw(TechnicHubPortModeInformation head);
};

class TechnicHubPortModeInformationPct : public TechnicHubPortModeInformation {
    public:
        uint8_t pctMin;
        uint8_t pctMax;
        TechnicHubPortModeInformationPct(TechnicHubPortModeInformation head);
};

class TechnicHubPortModeInformationSi : public TechnicHubPortModeInformation {
    public:
        uint8_t siMin;
        uint8_t siMax;
        TechnicHubPortModeInformationSi(TechnicHubPortModeInformation head);
};


class TechnicHubPortModeInformationSymbol : public TechnicHubPortModeInformation {
    public:
        String symbol;
        TechnicHubPortModeInformationSymbol(TechnicHubPortModeInformation head);
};

class TechnicHubPortMapping {
    public:
        boolean supportsNullValue;
        boolean supportsFunctionalMapping20;
        boolean abs;
        boolean rel;
        boolean dis;
};

class TechnicHubPortModeInformationMapping : public TechnicHubPortModeInformation {
    public:
        TechnicHubPortMapping inputMapping;
        TechnicHubPortMapping outputMapping;
};

class TechnicHubPortModeInformationMotorBias : public TechnicHubPortModeInformation {
    uint8_t bias;
};

class TechnicHubPortModeInformationCapabilityBits : public TechnicHubPortModeInformation {
    //was an Uint8Array
    boolean capabilityBits[16];
};

class TechnicHubValueFormat {
    int count;
    TechnicHubValueType valueType;
    int totalFigures;
    int decimals;
};

class TechnicHubPortModeInformationValueFormat : public TechnicHubPortModeInformation {
    TechnicHubValueFormat valueFormat;
};

class TechnicHubMessageBuilder {
    const static byte HUB_ID = 0x00;

    private:
        inline static uint8_t _length = 0;
        static uint8_t _getUseProfileUint8(boolean useAccelerationProfile, boolean useDecelerationProfile);
        static TechnicHubMessageHeader _convertHeader();
        static boolean * _getPortModes(uint16_t modes);
        static TechnicHubPortModes _convertPortMode(TechnicHubPortInformation message);
        static TechnicHubPortModeCombinations _convertPortModeCombinations(TechnicHubPortInformation message);
        static TechnicHubPortModeInformationMapping _getPortMapping(uint16_t mapping);
        static TechnicHubPortModeInformationName _convertPortModeInformationName(TechnicHubPortModeInformation message);
        static TechnicHubPortModeInformationRaw _convertPortModeInformationRaw(TechnicHubPortModeInformation message);
        static TechnicHubPortModeInformationPct _convertPortModeInformationPct(TechnicHubPortModeInformation message);
        static TechnicHubPortModeInformationSi _convertPortModeInformationSi(TechnicHubPortModeInformation message);
        static TechnicHubPortModeInformationSymbol _convertPortModeInformationSymbol(TechnicHubPortModeInformation message);

    public:
        TechnicHubMessageBuilder();
        virtual ~TechnicHubMessageBuilder();
        inline static byte buffer[64];
        static uint8_t createMessage(TechnicHubMessageType messageType, uint8_t size); 
        static uint8_t createPortInformationRequest(uint8_t portId, TechnicHubPortInformationType informationType);
        static uint8_t createPortModeInformationRequest(uint8_t portId, uint8_t mode, TechnicHubPortModeInformationType informationType);
        static uint8_t createPortInputFormatSetupSingle(uint8_t portId, uint8_t mode, uint32_t deltaInterval, boolean notificationEnabled);
        static uint8_t createVirtualPortConnect(uint8_t portIdA, uint8_t portIdB);
        static uint8_t createVirtualPortDisconnect(uint8_t portId);
        static uint8_t createPortOutput(uint8_t portId, boolean executeImmediately, boolean sendFeedback, TechnicHubPortOutputSubCommand subCommand, uint8_t size);
        static uint8_t createPortOutputStartPower(uint8_t portId, int8_t power);
        static uint8_t createPortOutputStartSpeed(uint8_t portId, int8_t speed, int8_t maxPower, boolean useAccelerationProfile, boolean useDecelerationProfile);
        static uint8_t createPortOutputGoToAbsolutePosition(uint8_t portId, int32_t position, int8_t speed, int8_t maxPower, TechnicHubPortOutputCommandEndState endState, boolean useAccelerationProfile, boolean useDecelerationProfile);
};

#endif
