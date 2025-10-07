MODULE_NAME='mPanasonicDisplay' (
                                    dev vdvObject,
                                    dev dvPort
                                )

(***********************************************************)
#DEFINE USING_NAV_MODULE_BASE_CALLBACKS
#DEFINE USING_NAV_MODULE_BASE_PROPERTY_EVENT_CALLBACK
#DEFINE USING_NAV_MODULE_BASE_PASSTHRU_EVENT_CALLBACK
#DEFINE USING_NAV_STRING_GATHER_CALLBACK
#DEFINE USING_NAV_LOGIC_ENGINE_EVENT_CALLBACK
#DEFINE USING_NAV_DEVICE_PRIORITY_QUEUE_SEND_NEXT_ITEM_EVENT_CALLBACK
#DEFINE USING_NAV_DEVICE_PRIORITY_QUEUE_FAILED_RESPONSE_EVENT_CALLBACK
#include 'NAVFoundation.LogicEngine.axi'
#include 'NAVFoundation.ModuleBase.axi'
#include 'NAVFoundation.SnapiHelpers.axi'
#include 'NAVFoundation.Math.axi'
#include 'NAVFoundation.SocketUtils.axi'
#include 'NAVFoundation.ArrayUtils.axi'
#include 'NAVFoundation.TimelineUtils.axi'
#include 'NAVFoundation.ErrorLogUtils.axi'
#include 'NAVFoundation.DevicePriorityQueue.axi'
#include 'NAVFoundation.Encoding.axi'
#include 'NAVFoundation.Cryptography.Md5.axi'

/*
 _   _                       _          ___     __
| \ | | ___  _ __ __ _  __ _| |_ ___   / \ \   / /
|  \| |/ _ \| '__/ _` |/ _` | __/ _ \ / _ \ \ / /
| |\  | (_) | | | (_| | (_| | ||  __// ___ \ V /
|_| \_|\___/|_|  \__, |\__,_|\__\___/_/   \_\_/
                 |___/

MIT License

Copyright (c) 2023 Norgate AV Services Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

(***********************************************************)
(*          DEVICE NUMBER DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_DEVICE

(***********************************************************)
(*               CONSTANT DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_CONSTANT

constant long TL_SOCKET_CHECK               = 1

constant long TL_SOCKET_CHECK_INTERVAL[]    = { 3000 }

constant integer REQUIRED_POWER_ON          = 1
constant integer REQUIRED_POWER_OFF         = 2

constant integer ACTUAL_POWER_ON            = 1
constant integer ACTUAL_POWER_OFF           = 2

constant integer INPUT_SLOT_1               = 1
constant integer INPUT_SLOT_1A              = 2
constant integer INPUT_SLOT_1B              = 3
constant integer INPUT_SLOT_2               = 4
constant integer INPUT_SLOT_2A              = 5
constant integer INPUT_SLOT_2B              = 6
constant integer INPUT_SLOT_3               = 7
constant integer INPUT_PC_1                 = 8
constant integer INPUT_DVI_1                = 9
constant integer INPUT_HDMI_1               = 10
constant integer INPUT_HDMI_2               = 11
constant integer INPUT_HDMI_3               = 12
constant integer INPUT_HDMI_4               = 13
constant integer INPUT_USB_1                = 14
constant integer INPUT_USB_2                = 15
constant integer INPUT_USB_3                = 16
constant integer INPUT_USB_4                = 17
constant integer INPUT_DIGITAL_LINK_1       = 18

constant char INPUT_SNAPI_PARAMS[][NAV_MAX_CHARS]   =   {
                                                            'SLOT,1',
                                                            'SLOT,1,A',
                                                            'SLOT,1,B',
                                                            'SLOT,2',
                                                            'SLOT,2,A',
                                                            'SLOT,2,B',
                                                            'SLOT,3',
                                                            'PC,1',
                                                            'DVI,1',
                                                            'HDMI,1',
                                                            'HDMI,2',
                                                            'HDMI,3',
                                                            'HDMI,4',
                                                            'USB,1',
                                                            'USB,2',
                                                            'USB,3',
                                                            'USB,4',
                                                            'DIGITAL_LINK,1'
                                                        }

constant char INPUT_COMMANDS[][NAV_MAX_CHARS]   =   {
                                                        'SL1',
                                                        'SL1A',
                                                        'SL1B',
                                                        'SL2',
                                                        'SL2A',
                                                        'SL2B',
                                                        'SL3',
                                                        'PC1',
                                                        'DV1',
                                                        'HM1',
                                                        'HM2',
                                                        'HM3',
                                                        'HM4',
                                                        'UC1',
                                                        'UC2',
                                                        'UC3',
                                                        'UC4',
                                                        'DL1'
                                                    }

constant integer GET_POWER  = 1
constant integer GET_INPUT  = 2
constant integer GET_MUTE   = 3
constant integer GET_VOLUME = 4

constant integer REQUIRED_MUTE_ON   = 1
constant integer REQUIRED_MUTE_OFF  = 2

constant integer ACTUAL_MUTE_ON     = 1
constant integer ACTUAL_MUTE_OFF    = 2

constant integer MAX_VOLUME = 100
constant integer MIN_VOLUME = 0

constant integer MODE_SERIAL       = 1
constant integer MODE_IP_DIRECT    = 2
constant integer MODE_IP_INDIRECT  = 3

constant char MODE_HEADER[][NAV_MAX_CHARS]    = { {NAV_STX_CHAR}, {'00'}, {NAV_STX_CHAR} }
constant char MODE_DELIMITER[][NAV_MAX_CHARS]    = { {NAV_ETX_CHAR}, {NAV_CR_CHAR}, {NAV_ETX_CHAR} }

constant integer DEFAULT_IP_PORT    = 1024

(***********************************************************)
(*              DATA TYPE DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_TYPE

(***********************************************************)
(*               VARIABLE DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_VARIABLE

volatile _NAVModule module
volatile _NAVLogicEngine engine
volatile _NAVDevicePriorityQueue queue

volatile _NAVDisplay object

volatile integer pollSequence = GET_POWER

volatile char id[3] = ''

volatile char baudRate[NAV_MAX_CHARS]    = '9600'

volatile integer mode = MODE_SERIAL

volatile char secureCommandRequired
volatile char connectionStarted
volatile char md5Seed[255]

volatile _NAVCredential credential = { 'dispadmin', '@Panasonic' }

(***********************************************************)
(*               LATCHING DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_LATCHING

(***********************************************************)
(*       MUTUALLY EXCLUSIVE DEFINITIONS GO BELOW           *)
(***********************************************************)
DEFINE_MUTUALLY_EXCLUSIVE

(***********************************************************)
(*        SUBROUTINE/FUNCTION DEFINITIONS GO BELOW         *)
(***********************************************************)
(* EXAMPLE: DEFINE_FUNCTION <RETURN_TYPE> <NAME> (<PARAMETERS>) *)
(* EXAMPLE: DEFINE_CALL '<NAME>' (<PARAMETERS>) *)

define_function SendString(char payload[]) {
    payload = "payload, MODE_DELIMITER[mode]"

    if (secureCommandRequired && ModeIsIp(mode)) {
        payload = "NAVHexToString(NAVMd5GetHash(GetMd5Message(credential, md5Seed))), payload"
    }

    if (ModeIsIp(mode)) {
        NAVErrorLog(NAV_LOG_LEVEL_DEBUG,
                    NAVFormatStandardLogMessage(NAV_STANDARD_LOG_MESSAGE_TYPE_STRING_TO,
                                                dvPort,
                                                payload))
    }

    send_string dvPort, "payload"
}


define_function char[NAV_MAX_BUFFER] BuildProtocol(char message[]) {
    char payload[NAV_MAX_BUFFER]

    payload = MODE_HEADER[mode]

    if (length_array(id)) {
        payload = "payload, 'AD94;RAD:', id, ';'"
    }

    return "payload, message"
}


define_function SendQuery(integer query) {
    if (queue.Busy) {
        return
    }

    switch (query) {
        case GET_POWER:     { EnqueueQueryItem(BuildProtocol("'QPW'")) }
        case GET_INPUT:     { EnqueueQueryItem(BuildProtocol("'QMI'")) }
        case GET_MUTE:      { EnqueueQueryItem(BuildProtocol("'QAM'")) }
        case GET_VOLUME:    { EnqueueQueryItem(BuildProtocol("'QAV'")) }
        default:            { SendQuery(GET_POWER) }
    }
}


define_function CommunicationTimeOut(integer timeout) {
    cancel_wait 'TimeOut'

    module.Device.IsCommunicating = true
    UpdateFeedback()

    wait (timeout * 10) 'TimeOut' {
        module.Device.IsCommunicating = false
        UpdateFeedback()
    }
}


define_function Reset() {
    module.Device.SocketConnection.IsConnected = false
    module.Device.IsCommunicating = false
    module.Device.IsInitialized = false
    UpdateFeedback()

    connectionStarted = false

    NAVLogicEngineStop(engine)
}



define_function SetPower(integer state) {
    switch (state) {
        case REQUIRED_POWER_ON:     { EnqueueCommandItem(BuildProtocol("'PON'")) }
        case REQUIRED_POWER_OFF:    { EnqueueCommandItem(BuildProtocol("'POF'")) }
    }
}


define_function SetInput(integer input) {
    EnqueueCommandItem(BuildProtocol("'IMS:', INPUT_COMMANDS[input]"))
}


define_function SetVolume(sinteger volume) {
    EnqueueCommandItem(BuildProtocol("'AVL:', format('%03d', volume)"))
}


define_function RampVolume(integer direction) {
    switch (direction) {
        case VOL_UP: {
            if (object.Volume.Level.Actual < MAX_VOLUME) {
                EnqueueCommandItem(BuildProtocol("'AUU'"))
            }
        }
        case VOL_DN: {
            if (object.Volume.Level.Actual > MIN_VOLUME) {
                EnqueueCommandItem(BuildProtocol("'AUD'"))
            }
        }
    }
}


define_function SetMute(integer state) {
    switch (state) {
        case REQUIRED_MUTE_ON:  { EnqueueCommandItem(BuildProtocol("'AMT:1'")) }
        case REQUIRED_MUTE_OFF: { EnqueueCommandItem(BuildProtocol("'AMT:0'")) }
    }
}


define_function integer ModeIsIp(integer mode) {
    return mode == MODE_IP_DIRECT || mode == MODE_IP_INDIRECT
}


define_function MaintainSocketConnection() {
    if (module.Device.SocketConnection.IsConnected) {
        return
    }

    ModuleLog(0, "'Attempting to connect to ', module.Device.SocketConnection.Address, ':', itoa(module.Device.SocketConnection.Port)")

    NAVClientSocketOpen(dvPort.PORT,
                        module.Device.SocketConnection.Address,
                        module.Device.SocketConnection.Port,
                        IP_TCP)
}


define_function char[NAV_MAX_BUFFER] GetMd5Message(_NAVCredential credential, char md5Seed[]) {
    return "credential.Username, ':', credential.Password, ':', md5Seed"
}


define_function EnqueueCommandItem(char item[]) {
    NAVDevicePriorityQueueEnqueue(queue, item, true)
}


define_function EnqueueQueryItem(char item[]) {
    NAVDevicePriorityQueueEnqueue(queue, item, false)
}


#IF_DEFINED USING_NAV_DEVICE_PRIORITY_QUEUE_SEND_NEXT_ITEM_EVENT_CALLBACK
define_function NAVDevicePriorityQueueSendNextItemEventCallback(char item[]) {
    SendString(item)
}
#END_IF


#IF_DEFINED USING_NAV_DEVICE_PRIORITY_QUEUE_FAILED_RESPONSE_EVENT_CALLBACK
define_function NAVDevicePriorityQueueFailedResponseEventCallback(_NAVDevicePriorityQueue queue) {
    module.Device.IsCommunicating = false
    UpdateFeedback()
}
#END_IF


#IF_DEFINED USING_NAV_LOGIC_ENGINE_EVENT_CALLBACK
define_function NAVLogicEngineEventCallback(_NAVLogicEngineEvent args) {
    if (!connectionStarted && ModeIsIp(mode)) {
        return;
    }

    if (secureCommandRequired && !length_array(md5Seed) && ModeIsIp(mode)) {
        return;
    }

    if (!module.Device.SocketConnection.IsConnected && ModeIsIp(mode)) {
        return;
    }

    if (queue.Busy) {
        return;
    }

    switch (args.Name) {
        case NAV_LOGIC_ENGINE_EVENT_QUERY: {
            SendQuery(pollSequence)
            return
        }
        case NAV_LOGIC_ENGINE_EVENT_ACTION: {
            if (module.CommandBusy) {
                return
            }

            if (object.PowerState.Required && (object.PowerState.Required == object.PowerState.Actual)) { object.PowerState.Required = 0; return }
            if (object.Input.Required && (object.Input.Required == object.Input.Actual)) { object.Input.Required = 0; return }
            if (object.Volume.Mute.Required && (object.Volume.Mute.Required == object.Volume.Mute.Actual)) { object.Volume.Mute.Required = 0; return }

            if (object.PowerState.Required && (object.PowerState.Required != object.PowerState.Actual)) {
                SetPower(object.PowerState.Required)
                module.CommandBusy = true
                wait 80 module.CommandBusy = false
                pollSequence = GET_POWER
                return
            }

            if (object.Input.Required && (object.PowerState.Actual == ACTUAL_POWER_ON) && (object.Input.Required != object.Input.Actual)) {
                SetInput(object.Input.Required)
                module.CommandBusy = true
                wait 10 module.CommandBusy = false
                pollSequence = GET_INPUT
                return
            }


            if (object.Volume.Mute.Required && (object.PowerState.Actual == ACTUAL_POWER_ON) && (object.Volume.Mute.Required != object.Volume.Mute.Actual)) {
                SetMute(object.Volume.Mute.Required)
                module.CommandBusy = true
                wait 10 module.CommandBusy = false
                pollSequence = GET_MUTE;
                return
            }

            if ([vdvObject, VOL_UP]) { RampVolume(VOL_UP) }
            if ([vdvObject, VOL_DN]) { RampVolume(VOL_DN) }
        }
    }
}
#END_IF


define_function char[NAV_MAX_BUFFER] GetError(integer error) {
    switch (error) {
        case 401:   { return 'Command cannot be executed' }
        case 402:   { return 'Invalid parameter' }
        default:    { return 'Unknown error' }
    }
}


define_function ProcessPowerResponse(char data[]) {
    switch (atoi(data)) {
        case false: {
            object.PowerState.Actual = ACTUAL_POWER_OFF
        }
        case true: {
            object.PowerState.Actual = ACTUAL_POWER_ON

            select {
                active (!object.Input.Initialized): {
                    pollSequence = GET_INPUT
                }
                active (!object.Volume.Level.Initialized): {
                    pollSequence = GET_VOLUME
                }
                active (!object.Volume.Mute.Initialized): {
                    pollSequence = GET_MUTE
                }
                active (true): {
                    pollSequence = GET_INPUT
                }
            }
        }
    }

    UpdateFeedback()
}


define_function ProcessInputResponse(char data[]) {
    object.Input.Actual = NAVFindInArrayString(INPUT_COMMANDS, data)
    object.Input.Initialized = true
    pollSequence = GET_POWER
}


define_function ProcessVolumeResponse(char data[]) {
    sinteger level

    level = atoi(data)

    if (object.Volume.Level.Actual != level) {
        object.Volume.Level.Actual = level
        send_level vdvObject, VOL_LVL, object.Volume.Level.Actual * 255 / MAX_VOLUME
        send_string vdvObject, "'VOLUME-ABS,', itoa(object.Volume.Level.Actual)"
        send_string vdvObject, "'VOLUME-', itoa(object.Volume.Level.Actual * 255 / MAX_VOLUME)"
    }

    object.Volume.Level.Initialized = true
    pollSequence = GET_POWER
}


define_function ProcessMuteResponse(char data[]) {
    switch (atoi(data)) {
        case false: {
            object.Volume.Mute.Actual = ACTUAL_MUTE_OFF
        }
        case true: {
            object.Volume.Mute.Actual = ACTUAL_MUTE_ON
        }
    }

    UpdateFeedback()

    object.Volume.Mute.Initialized = true;
    pollSequence = GET_POWER;
}


#IF_DEFINED USING_NAV_STRING_GATHER_CALLBACK
define_function NAVStringGatherCallback(_NAVStringGatherResult args) {
    stack_var char data[NAV_MAX_BUFFER]
    stack_var char delimiter[NAV_MAX_CHARS]

    data = args.Data
    delimiter = args.Delimiter

    if (ModeIsIp(mode)) {
        NAVErrorLog(NAV_LOG_LEVEL_DEBUG,
                    NAVFormatStandardLogMessage(NAV_STANDARD_LOG_MESSAGE_TYPE_PARSING_STRING_FROM,
                                                dvPort,
                                                data))
    }

    data = NAVStripRight(data, length_array(delimiter))

    select {
        active (NAVStartsWith(data, 'NTCONTROL')): {// || NAVStartsWith(data, 'PDPCONTROL')): {
            data = NAVStripLeft(data, 10);

            secureCommandRequired = atoi(remove_string(data, ' ', 1));

            if (secureCommandRequired) {
                md5Seed = data;
            }

            connectionStarted = true;
        }
        active (NAVStartsWith(data, MODE_HEADER[mode])): {
            stack_var char last[NAV_MAX_BUFFER]

            last = NAVDevicePriorityQueueGetLastMessage(queue)

            remove_string(data, MODE_HEADER[mode], 1)

            if (NAVStartsWith(data, 'ER')) {
                pollSequence = GET_POWER

                // ER401 - Command cannot be executed
                // ER402 - Invalid parameter

                // Common reasons for ER401:
                // - The projector is in a state where the command cannot be executed
                // --- Some commands cannot be executed when the projector is in standby
                // --- Sending 'QIN' (Get Input) when the projector is in standby will return ER401
                // --- Sending 'QSE' (Get Aspect) when the projector isn't displaying an image will return ER401

                remove_string(data, 'ER', 1)

                ModuleLog(NAV_LOG_LEVEL_ERROR,
                            "'Error: Command [', last, MODE_DELIMITER[mode], '] failed with error code ', data, ': ',
                                GetError(atoi(data))")

                NAVDevicePriorityQueueGoodResponse(queue)
                return
            }

            // Check if the message is addressed to this unit
            if (NAVContains(data, 'AD94;RAD:')) {
                stack_var char thisId[3]

                remove_string(data, ':', 1)

                thisId = NAVStripRight(remove_string(data, ';', 1), 1)

                if (thisId != id) {
                    pollSequence = GET_POWER
                    NAVDevicePriorityQueueGoodResponse(queue)
                    return
                }
            }

            switch (mode) {
                case MODE_SERIAL:
                case MODE_IP_INDIRECT: {
                    stack_var char cmd[NAV_MAX_CHARS]

                    cmd = NAVStripRight(remove_string(data, ':', 1), 1)

                    switch (cmd){
                        case 'QPW': {
                            ProcessPowerResponse(data)
                        }
                        case 'QMI': {
                            ProcessInputResponse(data)
                        }
                        case 'QAV': {
                            ProcessVolumeResponse(data)
                        }
                        case 'QAM': {
                            ProcessMuteResponse(data)
                        }
                    }
                }
                case MODE_IP_DIRECT: {
                    // Ignore if not a response to a query
                    if (!NAVStartsWith(last, 'Q')) {
                        NAVDevicePriorityQueueGoodResponse(queue)
                        return
                    }

                    select {
                        active (NAVContains(last, 'QPW')): {
                            ProcessPowerResponse(data)
                        }
                        active (NAVContains(last, 'QAM')): {
                            ProcessMuteResponse(data)
                        }
                        active (NAVContains(last, 'QAV')): {
                            ProcessVolumeResponse(data)
                        }
                        active (NAVContains(last, 'QMI')): {
                            ProcessInputResponse(data)
                        }
                    }
                }
            }

            NAVDevicePriorityQueueGoodResponse(queue)
        }
    }
}


#IF_DEFINED USING_NAV_MODULE_BASE_PROPERTY_EVENT_CALLBACK
define_function NAVModulePropertyEventCallback(_NAVModulePropertyEvent event) {
    if (event.Device != vdvObject) {
        return
    }

    switch (event.Name) {
        case NAV_MODULE_PROPERTY_EVENT_IP_ADDRESS: {
            module.Device.SocketConnection.Address = NAVTrimString(event.Args[1])
            ModuleLog(0, "'Setting IP Address to ', module.Device.SocketConnection.Address")
            NAVTimelineStart(TL_SOCKET_CHECK,
                                TL_SOCKET_CHECK_INTERVAL,
                                TIMELINE_ABSOLUTE,
                                TIMELINE_REPEAT)
        }
        case 'IP_PORT': {
            module.Device.SocketConnection.Port = atoi(event.Args[1])
            ModuleLog(0, "'Setting IP Port to ', itoa(module.Device.SocketConnection.Port)")
        }
        case 'COMM_MODE': {
            switch (event.Args[1]) {
                case 'SERIAL': {
                    mode = MODE_SERIAL
                    ModuleLog(0, "'Setting Comm Mode to Serial'")
                }
                case 'IP_DIRECT': {
                    mode = MODE_IP_DIRECT
                    ModuleLog(0, "'Setting Comm Mode to IP Direct'")
                }
                case 'IP_INDIRECT': {
                    mode = MODE_IP_INDIRECT
                    ModuleLog(0, "'Setting Comm Mode to IP Indirect'")
                }
            }
        }
        case NAV_MODULE_PROPERTY_EVENT_ID: {
            id = format('%03d', atoi(event.Args[1]))
        }
        case NAV_MODULE_PROPERTY_EVENT_BAUDRATE: {
            baudRate = event.Args[1]

            if ((mode == MODE_SERIAL) && device_id(event.Device)) {
                NAVCommand(event.Device, "'SET BAUD ', baudRate, ',N,8,1 485 DISABLE'")
            }
        }
        case NAV_MODULE_PROPERTY_EVENT_USERNAME: {
            credential.Username = NAVTrimString(event.Args[1])
        }
        case NAV_MODULE_PROPERTY_EVENT_PASSWORD: {
            credential.Password = NAVTrimString(event.Args[1])
        }
    }
}
#END_IF


#IF_DEFINED USING_NAV_MODULE_BASE_PASSTHRU_EVENT_CALLBACK
define_function NAVModulePassthruEventCallback(_NAVModulePassthruEvent event) {
    if (event.Device != vdvObject) {
        return
    }

    SendString(event.Payload)
}
#END_IF


define_function HandleSnapiMessage(_NAVSnapiMessage message, tdata data) {
    switch (message.Header) {
        case 'POWER': {
            switch (message.Parameter[1]) {
                case 'ON': {
                    object.PowerState.Required = REQUIRED_POWER_ON
                }
                case 'OFF': {
                    object.PowerState.Required = REQUIRED_POWER_OFF
                    object.Input.Required = 0
                }
            }
        }
        case 'MUTE': {
            if (object.PowerState.Actual != ACTUAL_POWER_ON) {
                return
            }

            switch (message.Parameter[1]) {
                case 'ON': {
                    object.Volume.Mute.Required = REQUIRED_MUTE_ON
                }
                case 'OFF': {
                    object.Volume.Mute.Required = REQUIRED_MUTE_OFF
                }
            }
        }
        case 'VOLUME': {
            switch (message.Parameter[1]) {
                case 'ABS': {
                    SetVolume(atoi(message.Parameter[2]))
                    pollSequence = GET_VOLUME
                }
                default: {
                    SetVolume(atoi(message.Parameter[1]) * 100 / 255)
                    pollSequence = GET_VOLUME
                }
            }
        }
        case 'INPUT': {
            stack_var integer input
            stack_var char inputCommand[NAV_MAX_CHARS]

            NAVTrimStringArray(message.Parameter)
            inputCommand = NAVArrayJoinString(message.Parameter, ',')

            input = NAVFindInArrayString(INPUT_SNAPI_PARAMS, inputCommand)

            if (input <= 0) {
                ModuleLog(NAV_LOG_LEVEL_WARNING, "'Invalid input: ', inputCommand")

                return
            }

            object.PowerState.Required = REQUIRED_POWER_ON
            object.Input.Required = input
        }
    }
}


define_function UpdateFeedback() {
    [vdvObject, NAV_IP_CONNECTED]	= (module.Device.SocketConnection.IsConnected)
    [vdvObject, DEVICE_COMMUNICATING] = (module.Device.IsCommunicating)
    [vdvObject, DATA_INITIALIZED] = (module.Device.IsInitialized)

    [vdvObject, VOL_MUTE_FB] = (object.Volume.Mute.Actual == ACTUAL_MUTE_ON)
    [vdvObject, POWER_FB] = (object.PowerState.Actual == ACTUAL_POWER_ON)
}


define_function ModuleLog(long level, char message[]) {
    if (level == 0) {
        NAVLog("__NAME__, ' [[', NAVDeviceToString(vdvObject), ']:[', NAVDeviceToString(dvPort), ']] => ', message")
        return
    }

    NAVErrorLog(level,
                "__NAME__, ' [[', NAVDeviceToString(vdvObject), ']:[', NAVDeviceToString(dvPort), ']] => ', message")
}


(***********************************************************)
(*                STARTUP CODE GOES BELOW                  *)
(***********************************************************)
DEFINE_START {
    NAVModuleInit(module)
    NAVLogicEngineInit(engine)
    NAVDevicePriorityQueueInit(queue)

    create_buffer dvPort, module.RxBuffer.Data
    module.Device.SocketConnection.Socket = dvPort.PORT
    module.Device.SocketConnection.Port = DEFAULT_IP_PORT
}

(***********************************************************)
(*                THE EVENTS GO BELOW                      *)
(***********************************************************)
DEFINE_EVENT

data_event[dvPort] {
    online: {
        if (data.device.number != 0) {
            NAVCommand(data.device, "'SET BAUD ', baudRate, ',N,8,1 485 DISABLE'")
            NAVCommand(data.device, "'B9MOFF'")
            NAVCommand(data.device, "'CHARD-0'")
            NAVCommand(data.device, "'CHARDM-0'")
            NAVCommand(data.device, "'HSOFF'")
        }

        if (data.device.number == 0) {
            module.Device.SocketConnection.IsConnected = true
            UpdateFeedback()
        }

        NAVLogicEngineStart(engine)
    }
    string: {
        CommunicationTimeOut(30)

        if (data.device.number == 0) {
            NAVErrorLog(NAV_LOG_LEVEL_DEBUG,
                        NAVFormatStandardLogMessage(NAV_STANDARD_LOG_MESSAGE_TYPE_STRING_FROM,
                                                    data.device,
                                                    data.text))
        }

        select {
            active(true): {
                NAVStringGather(module.RxBuffer, MODE_DELIMITER[mode])
            }
        }
    }
    offline: {
        if (data.device.number == 0) {
            NAVClientSocketClose(data.device.port)
            Reset()
        }
    }
    onerror: {
        if (data.device.number == 0) {
            Reset()
        }

        ModuleLog(NAV_LOG_LEVEL_ERROR, "'OnError: ', NAVGetSocketError(type_cast(data.number))")
    }
}


data_event[vdvObject] {
    online: {
        NAVCommand(data.device, "'PROPERTY-RMS_MONITOR_ASSET_PROPERTY,MONITOR_ASSET_DESCRIPTION,Monitor'")
        NAVCommand(data.device, "'PROPERTY-RMS_MONITOR_ASSET_PROPERTY,MONITOR_ASSET_MANUFACTURER_URL,www.panasonic.com'")
        NAVCommand(data.device, "'PROPERTY-RMS_MONITOR_ASSET_PROPERTY,MONITOR_ASSET_MANUFACTURER_NAME,Panasonic'")
    }
    command: {
        stack_var _NAVSnapiMessage message

        NAVParseSnapiMessage(data.text, message)

        HandleSnapiMessage(message, data)
    }
}


channel_event[vdvObject, 0] {
    on: {
        switch (channel.channel) {
            case POWER: {
                if (object.PowerState.Required) {
                    switch (object.PowerState.Required) {
                        case REQUIRED_POWER_ON: {
                            object.PowerState.Required = REQUIRED_POWER_OFF
                            object.Input.Required = 0
                        }
                        case REQUIRED_POWER_OFF: {
                            object.PowerState.Required = REQUIRED_POWER_ON
                        }
                    }
                }
                else {
                    switch (object.PowerState.Actual) {
                        case ACTUAL_POWER_ON: {
                            object.PowerState.Required = REQUIRED_POWER_OFF
                            object.Input.Required = 0
                        }
                        case ACTUAL_POWER_OFF: {
                            object.PowerState.Required = REQUIRED_POWER_ON
                        }
                    }
                }
            }
            case PWR_ON: {
                object.PowerState.Required = REQUIRED_POWER_ON
            }
            case PWR_OFF: {
                object.PowerState.Required = REQUIRED_POWER_OFF
                object.Input.Required = 0
            }
            case VOL_MUTE: {
                if (object.PowerState.Actual == ACTUAL_POWER_ON) {
                    if (object.Volume.Mute.Required) {
                        switch (object.Volume.Mute.Required) {
                            case REQUIRED_MUTE_ON: {
                                object.Volume.Mute.Required = REQUIRED_MUTE_OFF
                            }
                            case REQUIRED_MUTE_OFF: {
                                object.Volume.Mute.Required = REQUIRED_MUTE_ON
                            }
                        }
                    }
                    else {
                        switch (object.Volume.Mute.Actual) {
                            case ACTUAL_MUTE_ON: {
                                object.Volume.Mute.Required = REQUIRED_MUTE_OFF
                            }
                            case ACTUAL_MUTE_OFF: {
                                object.Volume.Mute.Required = REQUIRED_MUTE_ON
                            }
                        }
                    }
                }
            }
        }
    }
}


timeline_event[TL_SOCKET_CHECK] {
    MaintainSocketConnection()
}


timeline_event[TL_NAV_LOGIC_ENGINE] {
    NAVLogicEngineDrive(engine, timeline)
}


timeline_event[TL_NAV_DEVICE_PRIORITY_QUEUE_FAILED_RESPONSE] {
    NAVDevicePriorityQueueFailedResponse(queue)
}


(***********************************************************)
(*                     END OF PROGRAM                      *)
(*        DO NOT PUT ANY CODE BELOW THIS COMMENT           *)
(***********************************************************)
