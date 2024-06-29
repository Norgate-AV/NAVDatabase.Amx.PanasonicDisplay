MODULE_NAME='mPanasonicDisplay' (
                                    dev vdvObject,
                                    dev dvPort
                                )

(***********************************************************)
#include 'NAVFoundation.ModuleBase.axi'
#include 'NAVFoundation.Math.axi'
#include 'NAVFoundation.SocketUtils.axi'
#include 'NAVFoundation.ArrayUtils.axi'
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

constant long TL_DRIVE    = 1
constant long TL_IP_CLIENT_CHECK = 2

constant integer REQUIRED_POWER_ON    = 1
constant integer REQUIRED_POWER_OFF    = 2

constant integer ACTUAL_POWER_ON    = 1
constant integer ACTUAL_POWER_OFF    = 2

constant integer REQUIRED_INPUT_SLOT_1    = 1
constant integer REQUIRED_INPUT_SLOT_1A    = 2
constant integer REQUIRED_INPUT_SLOT_1B    = 3
constant integer REQUIRED_INPUT_SLOT_2    = 4
constant integer REQUIRED_INPUT_SLOT_2A    = 5
constant integer REQUIRED_INPUT_SLOT_2B    = 6
constant integer REQUIRED_INPUT_SLOT_3    = 7
constant integer REQUIRED_INPUT_PC_1    = 8
constant integer REQUIRED_INPUT_HDMI_1    = 9
constant integer REQUIRED_INPUT_DVI_1    = 10
constant integer REQUIRED_INPUT_HDMI_2    = 11
constant integer REQUIRED_INPUT_HDMI_3    = 12
constant integer REQUIRED_INPUT_HDMI_4    = 13

constant integer ACTUAL_INPUT_SLOT_1    = 1
constant integer ACTUAL_INPUT_SLOT_1A    = 2
constant integer ACTUAL_INPUT_SLOT_1B    = 3
constant integer ACTUAL_INPUT_SLOT_2    = 4
constant integer ACTUAL_INPUT_SLOT_2A    = 5
constant integer ACTUAL_INPUT_SLOT_2B    = 6
constant integer ACTUAL_INPUT_SLOT_3    = 7
constant integer ACTUAL_INPUT_PC_1    = 8
constant integer ACTUAL_INPUT_HDMI_1    = 9
constant integer ACTUAL_INPUT_DVI_1    = 10
constant integer ACTUAL_INPUT_HDMI_2    = 11
constant integer ACTUAL_INPUT_HDMI_3    = 12
constant integer ACTUAL_INPUT_HDMI_4    = 13

constant char INPUT_COMMANDS[][NAV_MAX_CHARS]   =   {
                                                        'SL1',
                                                        'SL1A',
                                                        'SL1B',
                                                        'SL2',
                                                        'SL2A',
                                                        'SL2B',
                                                        'SL3',
                                                        'PC1',
                                                        'HM1',
                                                        'DV1',
                                                        'HM2',
                                                        'HM3',
                                                        'HM4'
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

constant integer COMM_MODE_SERIAL       = 1
constant integer COMM_MODE_IP_DIRECT    = 2
constant integer COMM_MODE_IP_INDIRECT  = 3

constant char COMM_MODE_HEADER[][NAV_MAX_CHARS]    = { {NAV_STX_CHAR}, {'00'}, {NAV_STX_CHAR} }
constant char COMM_MODE_DELIMITER[][NAV_MAX_CHARS]    = { {NAV_ETX_CHAR}, {NAV_CR_CHAR}, {NAV_ETX_CHAR} }

constant integer DEFAULT_TCP_PORT    = 1024

(***********************************************************)
(*              DATA TYPE DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_TYPE

(***********************************************************)
(*               VARIABLE DEFINITIONS GO BELOW             *)
(***********************************************************)
DEFINE_VARIABLE

volatile _NAVDisplay uDisplay

volatile long ltIPClientCheck[] = { 3000 }

volatile integer iLoop
volatile integer iPollSequence = GET_POWER

volatile integer iRequiredPower
volatile integer iRequiredInput
volatile integer iRequiredMute
volatile sinteger siRequiredVolume = -1

volatile long ltDrive[] = { 200 }

volatile integer iSemaphore
volatile char cRxBuffer[NAV_MAX_BUFFER]

volatile integer iPowerBusy

volatile integer iCommandBusy
volatile integer iCommandLockOut

volatile char cID[3] = ''

volatile integer iRebuildReady = false

volatile _NAVSocketConnection uIPConnection

volatile integer iInputInitialized
volatile integer iVolumeInitialized
volatile integer iAudioMuteInitialized

volatile integer iCommMode = COMM_MODE_SERIAL
volatile integer iCommModeIP    = COMM_MODE_IP_DIRECT

volatile integer iSecureCommandRequired
volatile integer iConnectionStarted
volatile char cMD5RandomNumber[255]
volatile char cMD5StringToEncode[255]

volatile char cUserName[NAV_MAX_CHARS] = 'dispadmin'
volatile char cPassword[NAV_MAX_CHARS] = '@Panasonic'

volatile char cBaudRate[NAV_MAX_CHARS]    = '9600'

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

define_function SendStringRaw(char payload[]) {
    NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'String To ', NAVConvertDPSToAscii(dvPort), '-[', payload, ']'")
    send_string dvPort, "payload"
}


define_function SendString(char cmd[]) {
    char cPayload[NAV_MAX_BUFFER]

    cPayload = COMM_MODE_HEADER[iCommMode]

    if (length_array(cID)) {
        cPayload = "cPayload, 'AD94;RAD:', cID, ';'"
    }

    cPayload = "cPayload, cmd, COMM_MODE_DELIMITER[iCommMode]"

    if (iSecureCommandRequired && CommModeIsIP(iCommMode)) {
        cPayload = "NAVMd5GetHash(cMD5StringToEncode), cPayload"
    }

    SendStringRaw(cPayload)
}


define_function PassthruSend(char cmd[]) {
    char cPayload[NAV_MAX_BUFFER]

    cPayload = "COMM_MODE_HEADER[iCommMode], cmd, COMM_MODE_DELIMITER[iCommMode]"

    if (iSecureCommandRequired && CommModeIsIP(iCommMode)) {
        cPayload = "NAVMd5GetHash(cMD5StringToEncode), cPayload"
    }

    SendStringRaw(cPayload)
}


define_function integer CommModeIsIP(integer mode) {
    return mode == COMM_MODE_IP_DIRECT || mode == COMM_MODE_IP_INDIRECT
}


define_function SendQuery(integer query) {
    switch (query) {
        case GET_POWER:     { SendString("'QPW'") }
        case GET_INPUT:     { SendString("'QMI'") }
        case GET_MUTE:      { SendString("'QAM'") }
        case GET_VOLUME:    { SendString("'QAV'") }
    }
}


define_function TimeOut() {
    cancel_wait 'CommsTimeOut'

    wait 300 'CommsTimeOut' {
        [vdvObject, DEVICE_COMMUNICATING] = false

        if (CommModeIsIP(iCommMode)) {
            //uIPConnection.IsConnected = false
            NAVClientSocketClose(dvPort.PORT)
        }
    }
}


define_function SetPower(integer state) {
    switch (state) {
        case REQUIRED_POWER_ON:     { SendString("'PON'") }
        case REQUIRED_POWER_OFF:    { SendString("'POF'") }
    }
}


define_function SetInput(integer input) {
    SendString("'IMS:', INPUT_COMMANDS[input]")
}


define_function SetVolume(sinteger volume) {
    SendString("'AVL:', format('%03d', volume)")
}


define_function RampVolume(integer direction) {
    switch (direction) {
        case VOL_UP: {
            if (uDisplay.Volume.Level.Actual < MAX_VOLUME) {
                SendString("'AUU'")
            }
        }
        case VOL_DN: {
            if (uDisplay.Volume.Level.Actual > MIN_VOLUME) {
                SendString("'AUD'")
            }
        }
    }
}


define_function SetMute(integer state) {
    switch (state) {
        case REQUIRED_MUTE_ON:  { SendString("'AMT:1'") }
        case REQUIRED_MUTE_OFF: { SendString("'AMT:0'") }
    }
}


define_function char[NAV_MAX_CHARS] GetCommMode(integer mode) {
    switch (mode) {
        case COMM_MODE_SERIAL:      { return 'Serial' }
        case COMM_MODE_IP_DIRECT:   { return 'IP_Direct' }
        case COMM_MODE_IP_INDIRECT: { return 'IP_Indirect' }
        default:                    { return 'Unknown'}
    }
}


define_function Process() {
    stack_var char cTemp[NAV_MAX_BUFFER]

    if (iSemaphore) {
        return
    }

    iSemaphore = true
    //NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'Processing String From ', NAVConvertDPSToAscii(dvaCommObjects[iCommMode]), ' in Comm Mode[', itoa(iCommMode), ']', '-[', cRxBuffer[iCommMode], ']'")
    while (length_array(cRxBuffer) && NAVContains(cRxBuffer, COMM_MODE_DELIMITER[iCommMode])) {
        cTemp = remove_string(cRxBuffer, COMM_MODE_DELIMITER[iCommMode], 1)

        if (iCommModeIP == COMM_MODE_IP_DIRECT && iConnectionStarted) {
            NAVClientSocketClose(dvPort.PORT)
            //MaintainIPConnection()
        }

        if (!length_array(cTemp)) {
            continue
        }

        NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'Parsing String From ', NAVConvertDPSToAscii(dvPort), ' in Comm Mode[', GetCommMode(iCommMode), ']', '-[', cTemp, ']'")

        cTemp = NAVStripCharsFromRight(cTemp, 1)    //Remove delimiter

        select {
            active (NAVStartsWith(cTemp, 'NTCONTROL')): {// || NAVStartsWith(cTemp, 'PDPCONTROL')): {
                //Connection Started
                cTemp = NAVStripCharsFromLeft(cTemp, 10);
                //remove_string(cTemp, ' ', 1)
                iSecureCommandRequired = atoi(remove_string(cTemp, ' ', 1));

                if (iSecureCommandRequired) {
                    cMD5RandomNumber = cTemp;
                    //if (len(password) > 0 && len(user_name) > 0) {
                        cMD5StringToEncode = "cUserName, ':', cPassword, ':', cMD5RandomNumber"
                    //}//else {
                        //sMD5StringToEncode = DEFAULT_USER_NAME + ":" + DEFAULT_PASSWORD + ":" + sMD5RandomNumber;
                    //}
                }

                iConnectionStarted = 1;
                iLoop = 0;
                Drive();
            }
            active (NAVStartsWith(cTemp, COMM_MODE_HEADER[iCommMode])): {
                remove_string(cTemp, COMM_MODE_HEADER[iCommMode], 1)

                if (NAVStartsWith(cTemp, 'ER')) {
                    iPollSequence = GET_POWER
                    continue
                }

                if (NAVContains(cTemp, 'AD94;RAD:')) {
                    stack_var char cTempID[3]

                    remove_string(cTemp, ':', 1)

                    cTempID = NAVStripCharsFromRight(remove_string(cTemp, ';', 1), 1)

                    if (cTempID != cID) {
                        iPollSequence = GET_POWER
                        continue
                    }
                }

                switch (iCommMode) {
                    case COMM_MODE_SERIAL:
                    case COMM_MODE_IP_INDIRECT: {
                        stack_var char cAttribute[NAV_MAX_CHARS]

                        cAttribute = NAVStripCharsFromRight(remove_string(cTemp, ':', 1), 1)

                        select {
                            active (cAttribute == 'QPW'): {
                                stack_var integer iTemp

                                iTemp = atoi(cTemp)

                                switch (iTemp) {
                                    case false: uDisplay.PowerState.Actual = ACTUAL_POWER_OFF
                                    case true: uDisplay.PowerState.Actual = ACTUAL_POWER_ON
                                }

                                select {
                                    active (!iInputInitialized): {
                                        iPollSequence = GET_INPUT
                                    }
                                    active (!iVolumeInitialized): {
                                        iPollSequence = GET_VOLUME
                                    }
                                    active (!iAudioMuteInitialized): {
                                        iPollSequence = GET_MUTE
                                    }
                                }
                            }
                            active (cAttribute == 'QMI'): {
                                uDisplay.Input.Actual = NAVFindInArraySTRING(INPUT_COMMANDS, cTemp)
                                iInputInitialized = true
                                iPollSequence = GET_POWER
                            }
                            active (cAttribute == 'QAV'): {
                                stack_var sinteger siTemp

                                siTemp = atoi(cTemp)

                                if (uDisplay.Volume.Level.Actual != siTemp) {
                                    uDisplay.Volume.Level.Actual = siTemp
                                    send_level vdvObject, 1, uDisplay.Volume.Level.Actual * 255 / MAX_VOLUME
                                }

                                iVolumeInitialized = true
                                iPollSequence = GET_POWER
                            }
                            active (cAttribute == 'QAM'): {
                                stack_var integer iTemp

                                iTemp = atoi(cTemp)

                                switch (iTemp) {
                                    case false: uDisplay.Volume.Mute.Actual = ACTUAL_MUTE_OFF
                                    case true: uDisplay.Volume.Mute.Actual = ACTUAL_MUTE_ON
                                }

                                iAudioMuteInitialized = true
                                iPollSequence = GET_POWER
                            }
                        }
                    }
                    case COMM_MODE_IP_DIRECT: {
                        select {
                            active (NAVContains(cTemp, 'IMS') > 0): {
                                //Input Switch Ack
                            }
                            active (NAVContains(cTemp, 'AVL') > 0): {
                                //Volume Change Ack
                            }
                            active (NAVContains(cTemp, 'DGE') > 0): {
                                //Auto Setup Ack
                            }
                            active (NAVContains(cTemp, 'OSP') > 0): {
                                //OSD Ack
                            }
                            active (NAVContains(cTemp, 'VMT') > 0): {
                                //Video Mute Ack
                            }
                            active (NAVContains(cTemp, 'AMT') > 0): {
                                //Audio Mute Ack
                            }
                            active (NAVContains(cTemp, 'PON') > 0): {
                                //Power On Ack
                            }
                            active (NAVContains(cTemp, 'POF') > 0): {
                                //Power Off Ack
                            }
                            active (NAVContains(cTemp, 'UD1') > 0): {
                                //???
                            }
                            active (true): {
                                switch (iPollSequence) {
                                    case GET_POWER: {
                                        if (length_array(cTemp) = 1) {
                                            switch (atoi(cTemp)) {
                                                case 0: { uDisplay.PowerState.Actual = ACTUAL_POWER_OFF; }    //Power Off
                                                case 1: {
                                                    uDisplay.PowerState.Actual = ACTUAL_POWER_ON;    //Power On
                                                    select {
                                                        active (!iInputInitialized): { iPollSequence = GET_INPUT; }
                                                        active (!iAudioMuteInitialized): { iPollSequence = GET_MUTE; }
                                                        active (!iVolumeInitialized): { iPollSequence = GET_VOLUME; }
                                                        active (true): { iPollSequence = GET_INPUT; }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    case GET_MUTE: {
                                        stack_var integer iTemp

                                        if (length_array(cTemp) = 1) {
                                            iTemp = atoi(cTemp);

                                            switch (iTemp) {
                                                case 0: { uDisplay.Volume.Mute.Actual = ACTUAL_MUTE_OFF; }
                                                case 1: { uDisplay.Volume.Mute.Actual = ACTUAL_MUTE_ON; }
                                            }

                                            iAudioMuteInitialized = 1;
                                            iPollSequence = GET_POWER;
                                        }
                                    }
                                    case GET_VOLUME: {
                                        stack_var sinteger siTemp

                                        siTemp = atoi(cTemp);

                                        if (siTemp != uDisplay.Volume.Level.Actual) {
                                            uDisplay.Volume.Level.Actual = siTemp;
                                            send_level vdvObject, 1, uDisplay.Volume.Level.Actual * 255 / MAX_VOLUME
                                        }

                                        iVolumeInitialized = 1;
                                        iPollSequence = GET_POWER;
                                    }
                                    case GET_INPUT: {
                                        uDisplay.Input.Actual = NAVFindInArraySTRING(INPUT_COMMANDS, cTemp)
                                        iInputInitialized = true
                                        iPollSequence = GET_POWER
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    iSemaphore = false
}


define_function Drive() {
    if (!iConnectionStarted && CommModeIsIP(iCommMode) && iCommModeIP == COMM_MODE_IP_DIRECT) {
        return;
    }

    if (iSecureCommandRequired && !length_array(cMD5StringToEncode) && CommModeIsIP(iCommMode) && iCommModeIP == COMM_MODE_IP_DIRECT) {
        return;
    }

    if (!uIPConnection.IsConnected && CommModeIsIP(iCommMode)) {
        return;
    }

    iLoop++

    switch (iLoop) {
        case 1:
        case 6:
        case 11:
        case 16: { SendQuery(iPollSequence); return }
        case 21: { iLoop = 1; return }
        default: {
            if (iCommandLockOut) { return }

            if (iRequiredPower && (iRequiredPower == uDisplay.PowerState.Actual)) { iRequiredPower = 0; return }
            if (iRequiredInput && (iRequiredInput == uDisplay.Input.Actual)) { iRequiredInput = 0; return }
            if (iRequiredMute && (iRequiredMute == uDisplay.Volume.Mute.Actual)) { iRequiredMute = 0; return }

            if (iRequiredPower && (iRequiredPower != uDisplay.PowerState.Actual)) {
                iCommandBusy = true
                SetPower(iRequiredPower)
                iCommandLockOut = true
                wait 80 iCommandLockOut = false
                iPollSequence = GET_POWER
                return
            }

            if (iRequiredInput && (uDisplay.PowerState.Actual == ACTUAL_POWER_ON) && (iRequiredInput != uDisplay.Input.Actual)) {
                iCommandBusy = true
                SetInput(iRequiredInput)
                iCommandLockOut = true
                wait 10 iCommandLockOut = false
                iPollSequence = GET_INPUT
                return
            }


            if (iRequiredMute && (uDisplay.PowerState.Actual == ACTUAL_POWER_ON) && (iRequiredMute != uDisplay.Volume.Mute.Actual)) {
                iCommandBusy = true
                SetMute(iRequiredMute);
                iCommandLockOut = true
                wait 10 iCommandLockOut = false
                iPollSequence = GET_MUTE;
                return
            }

            // if (siRequiredVolume >= 0) {// && (uDisplay.PowerState.Actual == ACTUAL_POWER_ON)) {// && [vdvObject, DEVICE_COMMUNICATING]) {
            //     iCommandBusy = true
            //     SetVolume(siRequiredVolume);
            //     siRequiredVolume = -1
            //     iPollSequence = GET_VOLUME;
            //     return
            // }

            if ([vdvObject, VOL_UP]) { RampVolume(VOL_UP) }
            if ([vdvObject, VOL_DN]) { RampVolume(VOL_DN) }
        }
    }
}


define_function MaintainIPConnection() {
    if (uIPConnection.IsConnected) {
        return
    }

    NAVClientSocketOpen(dvPort.PORT, uIPConnection.Address, uIPConnection.Port, IP_TCP)
}


(***********************************************************)
(*                STARTUP CODE GOES BELOW                  *)
(***********************************************************)
DEFINE_START {
    create_buffer dvPort, cRxBuffer

    uIPConnection.Port = DEFAULT_TCP_PORT
}

(***********************************************************)
(*                THE EVENTS GO BELOW                      *)
(***********************************************************)
DEFINE_EVENT

data_event[dvPort] {
    online: {
        if (data.device.number != 0) {
            NAVCommand(data.device, "'SET BAUD ', cBaudRate, ', N, 8, 1 485 DISABLE'")
            NAVCommand(data.device, "'B9MOFF'")
            NAVCommand(data.device, "'CHARD-0'")
            NAVCommand(data.device, "'CHARDM-0'")
            NAVCommand(data.device, "'HSOFF'")
        }

        if (data.device.number == 0) {
            uIPConnection.IsConnected = true

            // iCommMode = iCommModeIP

            // NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'PANASONIC_PLASMA_IP_ONLINE<', NAVStringSurroundWith(NAVDeviceToString(dvPortIP), '[', ']'), '>'")
        }

        NAVTimelineStart(TL_DRIVE, ltDrive, TIMELINE_ABSOLUTE, TIMELINE_REPEAT)
    }
    string: {
        [vdvObject, DEVICE_COMMUNICATING] = true
        [vdvObject, DATA_INITIALIZED] = true

        TimeOut()

        NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'String From ', NAVConvertDPSToAscii(data.device), '-[', data.text, ']'")

        if (!iSemaphore) { Process() }
    }
    offline: {
        if (data.device.number == 0) {
            uIPConnection.IsConnected = false
            NAVClientSocketClose(data.device.port)
            iConnectionStarted = false
            // iCommMode = COMM_MODE_SERIAL
            // NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'PANASONIC_PLASMA_IP_OFFLINE<', NAVStringSurroundWith(NAVDeviceToString(dvPortIP), '[', ']'), '>'")

            MaintainIPConnection()
        }
    }
    onerror: {
        if (data.device.number == 0) {
            uIPConnection.IsConnected = false
            //NAVClientSocketClose(data.device.port)
            // iCommMode = COMM_MODE_SERIAL
            // NAVErrorLog(NAV_LOG_LEVEL_DEBUG, "'PANASONIC_PLASMA_IP_ONERROR<', NAVStringSurroundWith(NAVDeviceToString(dvPortIP), '[', ']'), '>'")
        }
    }
}


data_event[vdvObject] {
    command: {
        stack_var char cCmdHeader[NAV_MAX_CHARS]
        stack_var char cCmdParam[3][NAV_MAX_CHARS]

        NAVErrorLog(NAV_LOG_LEVEL_DEBUG, NAVFormatStandardLogMessage(NAV_STANDARD_LOG_MESSAGE_TYPE_COMMAND_FROM, data.device, data.text))

        cCmdHeader = DuetParseCmdHeader(data.text)
        cCmdParam[1] = DuetParseCmdParam(data.text)
        cCmdParam[2] = DuetParseCmdParam(data.text)
        cCmdParam[3] = DuetParseCmdParam(data.text)

        switch (cCmdHeader) {
            case 'PROPERTY': {
                switch (cCmdParam[1]) {
                    case 'IP_ADDRESS': {
                        uIPConnection.Address = cCmdParam[2]
                        NAVTimelineStart(TL_IP_CLIENT_CHECK, ltIPClientCheck, TIMELINE_ABSOLUTE, TIMELINE_REPEAT)
                    }
                    case 'IP_PORT': {
                        uIPConnection.Port = atoi(cCmdParam[2])
                    }
                    case 'ID': {
                        cID = format('%03d', atoi(cCmdParam[2]))
                    }
                    case 'COMM_MODE': {
                        switch (cCmdParam[2]) {
                            case 'SERIAL': {
                                iCommMode = COMM_MODE_SERIAL
                            }
                            case 'IP_DIRECT': {
                                iCommModeIP = COMM_MODE_IP_DIRECT
                                iCommMode = COMM_MODE_IP_DIRECT
                            }
                            case 'IP_INDIRECT': {
                                iCommModeIP = COMM_MODE_IP_INDIRECT
                                iCommMode = COMM_MODE_IP_INDIRECT
                            }
                        }
                    }
                    case 'BAUD_RATE': {
                        cBaudRate = cCmdParam[2]

                        if ((iCommMode == COMM_MODE_SERIAL) && device_id(dvPort)) {
                            send_command dvPort, "'SET BAUD ', cBaudRate, ', N, 8, 1 485 DISABLE'"
                        }
                    }
                    case 'USER_NAME': {
                        cUserName = cCmdParam[2]

                        if (length_array(cMD5RandomNumber)) {
                            cMD5StringToEncode = "cUserName, ':', cPassword, ':', cMD5RandomNumber"
                        }
                    }
                    case 'PASSWORD': {
                        cPassword = cCmdParam[2]

                        if (length_array(cMD5RandomNumber)) {
                            cMD5StringToEncode = "cUserName, ':', cPassword, ':', cMD5RandomNumber"
                        }
                    }
                }
            }
            case 'REBUILD': { iRebuildReady = true }
            case 'PASSTHRU': { PassthruSend(cCmdParam[1]) }
            case 'POWER': {
                switch (cCmdParam[1]) {
                    case 'ON': { iRequiredPower = REQUIRED_POWER_ON; Drive() }
                    case 'OFF': { iRequiredPower = REQUIRED_POWER_OFF; iRequiredInput = 0; Drive() }
                }
            }
            case 'VOLUME': {
                switch (cCmdParam[1]) {
                    case 'ABS': {
                        siRequiredVolume = atoi(cCmdParam[2]); Drive();
                        SetVolume(siRequiredVolume);
                        iPollSequence = GET_VOLUME
                    }
                    default: {
                        siRequiredVolume = NAVScaleValue(atoi(cCmdParam[1]), 255, (MAX_VOLUME - MIN_VOLUME), 0); Drive();
                        SetVolume(siRequiredVolume);
                        iPollSequence = GET_VOLUME
                    }
                }
            }
            case 'INPUT': {
                switch (cCmdParam[1]) {
                    case 'VGA': {
                        switch (cCmdParam[2]) {
                            case '1': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_PC_1; Drive() }
                        }
                    }
                    case 'SLOT': {
                        switch (cCmdParam[2]) {
                            case '1': {
                                switch (cCmdParam[3]) {
                                    case 'A': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_1A; Drive() }
                                    case 'B': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_1B; Drive() }
                                    default: { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_1; Drive() }
                                }
                            }
                            case '2': {
                                switch (cCmdParam[3]) {
                                    case 'A': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_2A; Drive() }
                                    case 'B': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_2B; Drive() }
                                    default: { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_2; Drive() }
                                }
                            }
                            case '3': {
                                switch (cCmdParam[3]) {
                                    default: { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_SLOT_3; Drive() }
                                }
                            }
                        }
                    }
                    case 'HDMI': {
                        switch (cCmdParam[2]) {
                            case '1': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_HDMI_1; Drive() }
                            case '2': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_HDMI_2; Drive() }
                            case '3': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_HDMI_3; Drive() }
                            case '4': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_HDMI_4; Drive() }
                        }
                    }
                    case 'DVI': {
                        switch (cCmdParam[2]) {
                            case '1': { iRequiredPower = REQUIRED_POWER_ON; iRequiredInput = REQUIRED_INPUT_DVI_1; Drive() }
                        }
                    }
                }
            }
        }
    }
}


channel_event[vdvObject, 0] {
    on: {
        switch (channel.channel) {
            case POWER: {
                if (iRequiredPower) {
                    switch (iRequiredPower) {
                        case REQUIRED_POWER_ON: { iRequiredPower = REQUIRED_POWER_OFF; iRequiredInput = 0; Drive() }
                        case REQUIRED_POWER_OFF: { iRequiredPower = REQUIRED_POWER_ON; Drive() }
                    }
                }
                else {
                    switch (uDisplay.PowerState.Actual) {
                        case ACTUAL_POWER_ON: { iRequiredPower = REQUIRED_POWER_OFF; iRequiredInput = 0; Drive() }
                        case ACTUAL_POWER_OFF: { iRequiredPower = REQUIRED_POWER_ON; Drive() }
                    }
                }
            }
            case PWR_ON: { iRequiredPower = REQUIRED_POWER_ON; Drive() }
            case PWR_OFF: { iRequiredPower = REQUIRED_POWER_OFF; iRequiredInput = 0; Drive() }
            //case PIC_MUTE: { SetShutter(![vdvObject, PIC_MUTE_FB]) }
            case VOL_MUTE: {
                if (uDisplay.PowerState.Actual == ACTUAL_POWER_ON) {
                    if (iRequiredMute) {
                        switch (iRequiredMute) {
                            case REQUIRED_MUTE_ON: { iRequiredMute = REQUIRED_MUTE_OFF; Drive() }
                            case REQUIRED_MUTE_OFF: { iRequiredMute = REQUIRED_MUTE_ON; Drive() }
                        }
                    }
                    else {
                        switch (uDisplay.Volume.Mute.Actual) {
                            case ACTUAL_MUTE_ON: { iRequiredMute = REQUIRED_MUTE_OFF; Drive() }
                            case ACTUAL_MUTE_OFF: { iRequiredMute = REQUIRED_MUTE_ON; Drive() }
                        }
                    }
                }
            }
        }
    }
}


timeline_event[TL_DRIVE] { Drive() }


timeline_event[TL_IP_CLIENT_CHECK] { MaintainIPConnection() }


timeline_event[TL_NAV_FEEDBACK] {
    [vdvObject, VOL_MUTE_FB] = (uDisplay.Volume.Mute.Actual == ACTUAL_MUTE_ON)
    [vdvObject, POWER_FB] = (uDisplay.PowerState.Actual == ACTUAL_POWER_ON)
}


(***********************************************************)
(*                     END OF PROGRAM                      *)
(*        DO NOT PUT ANY CODE BELOW THIS COMMENT           *)
(***********************************************************)
