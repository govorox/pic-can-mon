#include "app.h"
#include "can232.h"

INT8U lw232CanSpeedSelection = CAN_500KBPS;
INT8U lw232CanChannelMode = LW232_STATUS_CAN_CLOSED;
INT8U lw232LastErr = LW232_OK;

INT8U lw232AutoStart = LW232_AUTOSTART_OFF;
INT8U lw232AutoPoll  = LW232_AUTOPOLL_OFF;
INT8U lw232TimeStamp = LW232_TIMESTAMP_OFF;

INT32U lw232CanId = 0;

INT8U lw232Buffer[8];
INT8U lw232PacketLen = 0;

INT8U lw232Message[LW232_FRAME_MAX_SIZE];

const INT32U lw232CanBaudRates[] //PROGMEM
= { CAN_10KBPS, CAN_20KBPS, CAN_50KBPS, 
    CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, 
    CAN_500KBPS, CAN_500KBPS /*CAN_800KBPS*/,
    CAN_1000KBPS, CAN_83K3BPS };

const INT32U lw232SerialBaudRates[] //PROGMEM
= { 230400, 115200, 57600, 38400, 19200, 9600, 2400 };

#define SYS_PRINT_CHAR(c) SYS_PRINT("%c", c)


void printFullByte(INT8U b) {
    SYS_PRINT("%02X", b);
}

void printNibble(INT8U b) {
    SYS_PRINT("%1X", b & 0x0F);
}


INT8U parseNibble(INT8U hex) {
    INT8U ret = 0;
    if (hex >= '0' && hex <= '9') {
        ret = hex - '0';
    } else if (hex >= 'a' && hex <= 'f') {
        ret = hex - 'a' + 10;
    } else if (hex >= 'A' && hex <= 'F') {
        ret = hex - 'A' + 10;
    } // else error, return 0
    return ret;
}

INT8U parseFullByte(INT8U H, INT8U L) {
    return (parseNibble(H) << 4) + parseNibble(L);
}

INT8U parseNibbleWithLimit(INT8U hex, INT8U limit) {
    INT8U ret = parseNibble(hex);
    if (ret < limit)
        return ret;
    else
        return 0;
}

//----------------------------------------------

INT32U parseCanStdId(char * cmd) {
    lw232CanId = (((INT32U)parseNibble(cmd[1])) << 8)
        + (((INT32U)parseNibble(cmd[2])) << 4)
        + (((INT32U)parseNibble(cmd[3])));
    return lw232CanId;
}

INT32U parseCanExtId(char * cmd) {
    lw232CanId = (((INT32U)parseNibble(cmd[1])) << 20)
        + (((INT32U)parseNibble(cmd[2])) << 16)
        + (((INT32U)parseNibble(cmd[3])) << 12)
        + (((INT32U)parseNibble(cmd[4])) << 8)
        + (((INT32U)parseNibble(cmd[5])) << 4)
        + (((INT32U)parseNibble(cmd[6])));
    return lw232CanId;
}

bool isExtendedFrame() { return false; }

bool checkPassFilter(INT32U addr) { return true; }



INT8U checkReceive() 
{
    CAN_CHANNEL_EVENT channelEvent;
    channelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_CHANNEL1);
    if((channelEvent & (CAN_RX_CHANNEL_NOT_EMPTY | CAN_RX_CHANNEL_FULL)) != 0) {
        
        SYS_MESSAGE("\r\nRX\r\n");
        
        return CAN_MSGAVAIL;
    }
    return CAN_NOMSG; 
}

INT8U readMsgBufID(INT32U *ID, INT8U *len, INT8U buf[]) {
    
    INT8U ret = CAN_FAIL;
    
    CAN_CHANNEL_EVENT channelEvent;
    channelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_CHANNEL1);

    //if (channelEvent != 0)
    //    SYS_PRINT("\r\nCAN EVENT:%d", channelEvent);

    if((channelEvent & (CAN_RX_CHANNEL_NOT_EMPTY | CAN_RX_CHANNEL_FULL)) != 0)
    {
        // This means that either Receive Channel  is not empty
        // or the Channel is full.
        
        int i;
        CAN_RX_MSG_BUFFER *RxMessage;     
        RxMessage = (CAN_RX_MSG_BUFFER *)PLIB_CAN_ReceivedMessageGet(CAN_ID_1, CAN_CHANNEL1);
        if(RxMessage != NULL)
        {
            SYS_PRINT("\r\nSID:%04X L=%d ", RxMessage->msgSID.sid, RxMessage->msgEID.data_length_code);
            
            *ID = RxMessage->msgSID.sid;
            *len = RxMessage->msgEID.data_length_code;
            for (i=0; i<RxMessage->msgEID.data_length_code; i++) {
                buf[i] = RxMessage->data[i];
                SYS_PRINT("%02X",buf[i]);
            }
            SYS_PRINT("\r\n");
            
            //Process the message received

            /* Call the PLIB_CAN_ChannelUpdate function to let
            the CAN module know that the message processing
            is done. */
            PLIB_CAN_ChannelUpdate(CAN_ID_1, CAN_CHANNEL1);

            ret = CAN_OK;
        }
        
        PLIB_CAN_ChannelEventClear(CAN_ID_1, CAN_CHANNEL1, channelEvent);
    }
    return ret; 
}

INT8U receiveSingleFrame() { 
    
    INT8U ret = LW232_OK;
    INT8U idx = 0;
    if (CAN_OK == readMsgBufID(&lw232CanId, &lw232PacketLen, lw232Buffer)) {
        if (lw232CanId > 0x1FFFFFFF) {
            ret = LW232_ERR; // address if totally wrong
        }
        else if (checkPassFilter(lw232CanId)) {// do we want to skip some addresses?
            if (isExtendedFrame()) {
                SYS_PRINT_CHAR(LW232_TR29);
                printFullByte(HIGH_BYTE(HIGH_WORD(lw232CanId)));
                printFullByte(LOW_BYTE(HIGH_WORD(lw232CanId)));
                printFullByte(HIGH_BYTE(LOW_WORD(lw232CanId)));
                printFullByte(LOW_BYTE(LOW_WORD(lw232CanId)));
            }
            else {
                SYS_PRINT_CHAR(LW232_TR11);
                printNibble(HIGH_BYTE(LOW_WORD(lw232CanId)));
                printFullByte(LOW_BYTE(LOW_WORD(lw232CanId)));
            }
            //write data len
            printNibble(lw232PacketLen);
            //write data
            for (idx = 0; idx < lw232PacketLen; idx++) {
                printFullByte(lw232Buffer[idx]);
            }
            //write timestamp if needed
            if (lw232TimeStamp != LW232_TIMESTAMP_OFF) {
                INT32U time = 0; //millis();
                if (lw232TimeStamp == LW232_TIMESTAMP_ON_NORMAL) { 
                    // standard LAWICEL protocol. two bytes.
                    time %= 60000;  
                } else {
                    // non standard protocol - 4 bytes timestamp
                    printFullByte(HIGH_BYTE(HIGH_WORD(time)));
                    printFullByte(LOW_BYTE(HIGH_WORD(time)));
                }
                printFullByte(HIGH_BYTE(LOW_WORD(time)));
                printFullByte(LOW_BYTE(LOW_WORD(time)));
            }
        }
    }
    else {
        ret = LW232_ERR;
    }
    return ret;
}


INT8U openCanBus() {
    DRV_CAN0_Open();
    return 0;
}

INT8U closeCanBus() {
    DRV_CAN0_Close();
    return 0;
}

INT8U checkError(INT8U * err) { return 0; };
    
INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U rtr, INT8U len, INT8U *buf)
{ 
    SYS_PRINT("TX id:%X l:%d %02X", id, len, buf[0]);
    if (DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL0, id, len, buf))
        return CAN_OK;
    return CAN_FAIL;
}

//==============================================

void CAN_232_Poll() {
    
}

void CAN232_Command(char * command) {
    //SYS_PRINT(">%s", command);
    
    INT8U ret = LW232_OK;
    INT8U idx = 0;
    INT8U err = 0;

    lw232LastErr = LW232_OK;

    switch (command[0]) {
        case LW232_CMD_SETUP:
            // Sn[CR] Setup with standard CAN bit-rates where n is 0-9.
            if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                idx = parseNibbleWithLimit(command[1], LW232_CAN_BAUD_NUM);
                lw232CanSpeedSelection = lw232CanBaudRates[idx];
                
            }
            else {
                ret = LW232_ERR;
            }
            break;
        case LW232_CMD_SETUP_BTR:
            // sxxyy[CR] Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
            ret = LW232_ERR; break;
        case LW232_CMD_OPEN:
            // O[CR] Open the CAN channel in normal mode (sending & receiving).
            if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                lw232CanChannelMode = LW232_STATUS_CAN_OPEN_NORMAL;
                ret = openCanBus();
            }
            else {
                ret = LW232_ERR;
            }
            break;
        case LW232_CMD_LISTEN:
            // L[CR] Open the CAN channel in listen only mode (receiving).
            if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                lw232CanChannelMode = LW232_STATUS_CAN_OPEN_LISTEN;
                ret = openCanBus();
            }
            else {
                ret = LW232_ERR;
            }
        break;
        case LW232_CMD_CLOSE:
            // C[CR] Close the CAN channel.
            if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED) {
                lw232CanChannelMode = LW232_STATUS_CAN_CLOSED;
                closeCanBus();
            }
            else {
                ret = LW232_ERR;
            }
        break;
        case LW232_CMD_TX11:
            // tiiildd...[CR] Transmit a standard (11bit) CAN frame.
            if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
                lw232CanId = parseCanStdId(command);
                lw232PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH);
                for (idx=0; idx < lw232PacketLen; idx++) {
                    lw232Buffer[idx] = parseFullByte(command[LW232_OFFSET_STD_PKT_DATA + idx * 2], 
                                                     command[LW232_OFFSET_STD_PKT_DATA + idx * 2 + 1]);
                }
                if (CAN_OK != sendMsgBuf(lw232CanId, 0, 0, lw232PacketLen, lw232Buffer)) {
                    ret = LW232_ERR;
                }
                else if (lw232AutoPoll) {
                    ret = LW232_OK_SMALL;
                }
            }
            else {
                ret = LW232_ERR;
            }
        break;
    case LW232_CMD_TX29:
        // Tiiiiiiiildd...[CR] Transmit an extended (29bit) CAN frame
        if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            lw232CanId = parseCanExtId(command);
            lw232PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            for (idx=0; idx < lw232PacketLen; idx++) {
                lw232Buffer[idx] = parseFullByte(command[LW232_OFFSET_EXT_PKT_DATA + idx * 2],
                                                 command[LW232_OFFSET_EXT_PKT_DATA + idx * 2 + 1]);
            }
            if (CAN_OK != sendMsgBuf(lw232CanId, 1, 0, lw232PacketLen, lw232Buffer)) {
                ret = LW232_ERR;
            } else if (lw232AutoPoll) {
                ret = LW232_OK_BIG;
            }
        }
        break;
    case LW232_CMD_RTR11:
        // riiil[CR] Transmit an standard RTR (11bit) CAN frame.
        if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            parseCanStdId(command);
            lw232PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            if (CAN_OK != sendMsgBuf(lw232CanId, 0, 1, lw232PacketLen, lw232Buffer)) {
                ret = LW232_ERR;
            }
            else if (lw232AutoPoll) {
                ret = LW232_OK_SMALL;
            }
        }
        else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_RTR29:
        // Riiiiiiiil[CR] Transmit an extended RTR (29bit) CAN frame.
        if (lw232CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            parseCanExtId(command);
            lw232PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            if (CAN_OK != sendMsgBuf(lw232CanId, 1, 1, lw232PacketLen, lw232Buffer)) {
                ret = LW232_ERR;
            }
            else if (lw232AutoPoll) {
                ret = LW232_OK_SMALL; // not a typo. strangely can232_v3.pdf tells to return "z[CR]", not "Z[CR]" as in 29bit. todo: check if it is error in pdf???
            }
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_POLL_ONE:
        // P[CR] Poll incomming FIFO for CAN frames (single poll)
        if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232AutoPoll == LW232_AUTOPOLL_OFF) {
            if (CAN_MSGAVAIL == checkReceive()) {
                ret = receiveSingleFrame();
            }
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_POLL_MANY:
        // A[CR] Polls incomming FIFO for CAN frames (all pending frames)
        if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232AutoPoll == LW232_AUTOPOLL_OFF) {
            while (CAN_MSGAVAIL == checkReceive()) {
                ret = ret ^ receiveSingleFrame();
                if (ret != CAN_OK)
                    break;
                SYS_PRINT("%c", LW232_CR);
            }
            if (ret == CAN_OK)
                SYS_PRINT("%c", LW232_ALL);
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_FLAGS:
        // F[CR] Read Status Flags.
        // LAWICEL CAN232 and CANUSB have some specific errors which differ from MCP2515/MCP2551 errors. We just return MCP2515 error.
        SYS_PRINT("%c", LW232_FLAG);
        if (checkError(&err) == CAN_OK) 
            err = 0;
        printFullByte(err & 0xFF); //MCP_EFLG_ERRORMASK);
        break;
    case LW232_CMD_AUTOPOLL:
        // Xn[CR] Sets Auto Poll/Send ON/OFF for received frames.
        if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
            lw232AutoPoll = (command[1] == LW232_ON_ONE) ? LW232_AUTOPOLL_ON : LW232_AUTOPOLL_OFF;
            //todo: save to eeprom
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_FILTER:
        // Wn[CR] Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
        ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_ACC_CODE:
        // Mxxxxxxxx[CR] Sets Acceptance Code Register (ACn Register of SJA1000). // we use MCP2515,
        ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_ACC_MASK:
        // mxxxxxxxx[CR] Sets Acceptance Mask Register (AMn Register of SJA1000).
        ret = LW232_ERR_NOT_IMPLEMENTED; break;
    case LW232_CMD_UART:
        // Un[CR] Setup UART with a new baud rate where n is 0-6.
        idx = parseNibbleWithLimit(command[1], LW232_UART_BAUD_NUM);
        //Serial.begin(lw232SerialBaudRates[idx]);
        break;
    case LW232_CMD_VERSION1:
    case LW232_CMD_VERSION2:
        // V[CR] Get Version number of both CAN232 hardware and software
        SYS_PRINT(LW232_LAWICEL_VERSION_STR);
        break;
    case LW232_CMD_SERIAL:
        // N[CR] Get Serial number of the CAN232.
        SYS_PRINT(LW232_LAWICEL_SERIAL_NUM);
        break;
    case LW232_CMD_TIMESTAMP:
        // Zn[CR] Sets Time Stamp ON/OFF for received frames only. Z0 - OFF, Z1 - Lawicel's timestamp 2 bytes, Z2 - arduino timestamp 4 bytes.
        if (lw232CanChannelMode == LW232_STATUS_CAN_CLOSED) {
            // lw232TimeStamp = (command[1] == LW232_ON_ONE); 
            if (command[1] == LW232_ON_ONE) {
                lw232TimeStamp = LW232_TIMESTAMP_ON_NORMAL;
            }
            else if (command[1] == LW232_ON_TWO) {
                lw232TimeStamp = LW232_TIMESTAMP_ON_EXTENDED;
            }
            else {
                lw232TimeStamp = LW232_TIMESTAMP_OFF;
            }
        }
        else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_AUTOSTART:
        // Qn[CR] Auto Startup feature (from power on).
        if (lw232CanChannelMode != LW232_STATUS_CAN_CLOSED) {
            if (command[1] == LW232_ON_ONE) {
                lw232AutoStart = LW232_AUTOSTART_ON_NORMAL;
            }
            else if (command[1] == LW232_ON_TWO) {
                lw232AutoStart = LW232_AUTOSTART_ON_LISTEN;
            }
            else {
                lw232AutoStart = LW232_AUTOSTART_OFF;
            }
            //todo: save to eeprom
        }
        else {
            ret = LW232_ERR;
        }
        break;
    default:
        ret = LW232_ERR_UNKNOWN_CMD;
    }

    
    lw232LastErr = ret; //parseAndRunCommand();

    switch (lw232LastErr) {
    case LW232_OK:
        SYS_PRINT_CHAR(LW232_RET_ASCII_OK);
        break;
    case LW232_OK_SMALL:
        SYS_PRINT_CHAR(LW232_RET_ASCII_OK_SMALL);
        SYS_PRINT_CHAR(LW232_RET_ASCII_OK);
        break;
    case LW232_OK_BIG:
        SYS_PRINT("%c", LW232_RET_ASCII_OK_BIG);
        SYS_PRINT("%c", LW232_RET_ASCII_OK);
        break;
    case LW232_ERR_NOT_IMPLEMENTED:
        // Choose behavior: will it fail or not when not implemented command comes in. Some can monitors might be affected by this selection.
        SYS_PRINT("%c", LW232_RET_ASCII_ERROR);
        //SYS_PRINT(LW232_RET_ASCII_OK); 
        break;
    default:
        SYS_PRINT("%c", LW232_RET_ASCII_ERROR);
    }
}
