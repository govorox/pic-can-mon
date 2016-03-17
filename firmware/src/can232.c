#include "app.h"
#include "can232.h"

uint8_t lw232_CanSpeedSelection = CAN_500KBPS;
uint8_t lw232_CanChannelMode = LW232_STATUS_CAN_CLOSED;
uint8_t lw232_LastErr = LW232_OK;

uint8_t lw232_AutoStart = LW232_AUTOSTART_OFF;
uint8_t lw232_AutoPoll  = LW232_AUTOPOLL_OFF;
uint8_t lw232_TimeStamp = LW232_TIMESTAMP_OFF;

uint32_t lw232_CanId = 0;

uint8_t lw232_Buffer[8];
uint8_t lw232_PacketLen = 0;

uint8_t lw232_Message[LW232_FRAME_MAX_SIZE];

const uint32_t lw232_CanBaudRates[] //PROGMEM
= { CAN_10KBPS, CAN_20KBPS, CAN_50KBPS, 
    CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, 
    CAN_500KBPS, CAN_500KBPS /*CAN_800KBPS*/,
    CAN_1000KBPS, CAN_83K3BPS };

const uint32_t lw232_SerialBaudRates[] //PROGMEM
= { 230400, 115200, 57600, 38400, 19200, 9600, 2400 };

//#define printChar(c) SYS_PRINT("%c", c)

void printChar(char c) {
    SYS_PRINT("%c", c);
}

void printFullByte(uint8_t b) {
    SYS_PRINT("%02X", b);
}

void printNibble(uint8_t b) {
    SYS_PRINT("%1X", b & 0x0F);
}


uint8_t parseNibble(uint8_t hex) {
    uint8_t ret = 0;
    if (hex >= '0' && hex <= '9') {
        ret = hex - '0';
    } else if (hex >= 'a' && hex <= 'f') {
        ret = hex - 'a' + 10;
    } else if (hex >= 'A' && hex <= 'F') {
        ret = hex - 'A' + 10;
    } // else error, return 0
    return ret;
}

uint8_t parseFullByte(uint8_t H, uint8_t L) {
    return (parseNibble(H) << 4) + parseNibble(L);
}

uint8_t parseNibbleWithLimit(uint8_t hex, uint8_t limit) {
    uint8_t ret = parseNibble(hex);
    if (ret < limit)
        return ret;
    else
        return 0;
}

//----------------------------------------------

uint32_t parseCanStdId(char * cmd) {
    lw232_CanId = (((uint32_t)parseNibble(cmd[1])) << 8)
        + (((uint32_t)parseNibble(cmd[2])) << 4)
        + (((uint32_t)parseNibble(cmd[3])));
    return lw232_CanId;
}

uint32_t parseCanExtId(char * cmd) {
    lw232_CanId = (((uint32_t)parseNibble(cmd[1])) << 20)
        + (((uint32_t)parseNibble(cmd[2])) << 16)
        + (((uint32_t)parseNibble(cmd[3])) << 12)
        + (((uint32_t)parseNibble(cmd[4])) << 8)
        + (((uint32_t)parseNibble(cmd[5])) << 4)
        + (((uint32_t)parseNibble(cmd[6])));
    return lw232_CanId;
}

bool isExtendedFrame() { return false; }

bool checkPassFilter(uint32_t addr) { return true; }



uint8_t checkReceive() 
{
    CAN_CHANNEL_EVENT channelEvent;
    channelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_CHANNEL1);
    if((channelEvent & (CAN_RX_CHANNEL_NOT_EMPTY | CAN_RX_CHANNEL_FULL)) != 0) {
        
        //SYS_MESSAGE("\r\nRX\r\n");
        
        return CAN_MSGAVAIL;
    }
    return CAN_NOMSG; 
}

uint8_t readMsgBufID(uint32_t *ID, uint8_t *len, uint8_t buf[]) {
    
    uint8_t ret = CAN_FAIL;
    
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
#ifdef TRACE
            SYS_PRINT("\r\nSID:%04X L=%d ", RxMessage->msgSID.sid, RxMessage->msgEID.data_length_code);
#endif      
            *ID = RxMessage->msgSID.sid;
            *len = RxMessage->msgEID.data_length_code;
            for (i=0; i<RxMessage->msgEID.data_length_code; i++) {
                buf[i] = RxMessage->data[i];
#ifdef TRACE
                SYS_PRINT("%02X",buf[i]);
#endif
            }
#ifdef TRACE
            SYS_PRINT("\r\n");
#endif
            
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

uint8_t receiveSingleFrame() { 
    
    uint8_t ret = LW232_OK;
    uint8_t idx = 0;
    if (CAN_OK == readMsgBufID(&lw232_CanId, &lw232_PacketLen, lw232_Buffer)) {
        if (lw232_CanId > 0x1FFFFFFF) {
            ret = LW232_ERR; // address if totally wrong
        }
        else if (checkPassFilter(lw232_CanId)) {// do we want to skip some addresses?
            if (isExtendedFrame()) {
                printChar(LW232_TR29);
                printFullByte(HIGH_BYTE(HIGH_WORD(lw232_CanId)));
                printFullByte(LOW_BYTE(HIGH_WORD(lw232_CanId)));
                printFullByte(HIGH_BYTE(LOW_WORD(lw232_CanId)));
                printFullByte(LOW_BYTE(LOW_WORD(lw232_CanId)));
            }
            else {
                printChar(LW232_TR11);
                printNibble(HIGH_BYTE(LOW_WORD(lw232_CanId)));
                printFullByte(LOW_BYTE(LOW_WORD(lw232_CanId)));
            }
            //write data len
            printNibble(lw232_PacketLen);
            //write data
            for (idx = 0; idx < lw232_PacketLen; idx++) {
                printFullByte(lw232_Buffer[idx]);
            }
            //write timestamp if needed
            if (lw232_TimeStamp != LW232_TIMESTAMP_OFF) {
                uint32_t time = 0; //millis();
                if (lw232_TimeStamp == LW232_TIMESTAMP_ON_NORMAL) { 
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


uint8_t openCanBus() {
    DRV_CAN0_Open();
    return 0;
}

uint8_t closeCanBus() {
    DRV_CAN0_Close();
    return 0;
}

uint8_t checkError(uint8_t * err) { return 0; };
    
uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf)
{ 
    //SYS_PRINT("TX id:%X l:%d %02X", id, len, buf[0]);
    if (DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL0, id, len, buf))
        return CAN_OK;
    return CAN_FAIL;
}

//==============================================================================
// This function performs background tasks:
// - receives and displays frames in auto
// poll mode
//------------------------------------------------------------------------------
void CAN232_Tasks() {
    if (lw232_CanChannelMode != LW232_STATUS_CAN_CLOSED
        && lw232_AutoPoll == LW232_AUTOPOLL_ON) 
    {
        int recv = 0;
        while (CAN_MSGAVAIL == checkReceive() && recv++<5) {
            //printChar('+');
            if (CAN_OK == receiveSingleFrame()) {
                printChar(LW232_CR);
            }
        }
    }
}

//==============================================================================
// Parses and executes a command
// Returns: LW232_OK in case of success, otherwise - specific error code
//------------------------------------------------------------------------------
void CAN232_Command(char * command) {
    //SYS_PRINT(">%s", command);
    
    uint8_t ret = LW232_OK;
    uint8_t idx = 0;
    uint8_t err = 0;

    lw232_LastErr = LW232_OK;

    switch (command[0]) {
        case LW232_CMD_SETUP:
            // Sn[CR] Setup with standard CAN bit-rates where n is 0-9.
            if (lw232_CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                idx = parseNibbleWithLimit(command[1], LW232_CAN_BAUD_NUM);
                lw232_CanSpeedSelection = lw232_CanBaudRates[idx];
                
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
            if (lw232_CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                lw232_CanChannelMode = LW232_STATUS_CAN_OPEN_NORMAL;
                ret = openCanBus();
            }
            else {
                ret = LW232_ERR;
            }
            break;
        case LW232_CMD_LISTEN:
            // L[CR] Open the CAN channel in listen only mode (receiving).
            if (lw232_CanChannelMode == LW232_STATUS_CAN_CLOSED) {
                lw232_CanChannelMode = LW232_STATUS_CAN_OPEN_LISTEN;
                ret = openCanBus();
            }
            else {
                ret = LW232_ERR;
            }
        break;
        case LW232_CMD_CLOSE:
            // C[CR] Close the CAN channel.
            if (lw232_CanChannelMode != LW232_STATUS_CAN_CLOSED) {
                lw232_CanChannelMode = LW232_STATUS_CAN_CLOSED;
                closeCanBus();
            }
            else {
                ret = LW232_ERR;
            }
        break;
        case LW232_CMD_TX11:
            // tiiildd...[CR] Transmit a standard (11bit) CAN frame.
            if (lw232_CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
                lw232_CanId = parseCanStdId(command);
                lw232_PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH);
                for (idx=0; idx < lw232_PacketLen; idx++) {
                    lw232_Buffer[idx] = parseFullByte(command[LW232_OFFSET_STD_PKT_DATA + idx * 2], 
                                                     command[LW232_OFFSET_STD_PKT_DATA + idx * 2 + 1]);
                }
                if (CAN_OK != sendMsgBuf(lw232_CanId, 0, 0, lw232_PacketLen, lw232_Buffer)) {
                    ret = LW232_ERR;
                }
                else if (lw232_AutoPoll) {
                    ret = LW232_OK_SMALL;
                }
            }
            else {
                ret = LW232_ERR;
            }
        break;
    case LW232_CMD_TX29:
        // Tiiiiiiiildd...[CR] Transmit an extended (29bit) CAN frame
        if (lw232_CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            lw232_CanId = parseCanExtId(command);
            lw232_PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            for (idx=0; idx < lw232_PacketLen; idx++) {
                lw232_Buffer[idx] = parseFullByte(command[LW232_OFFSET_EXT_PKT_DATA + idx * 2],
                                                 command[LW232_OFFSET_EXT_PKT_DATA + idx * 2 + 1]);
            }
            if (CAN_OK != sendMsgBuf(lw232_CanId, 1, 0, lw232_PacketLen, lw232_Buffer)) {
                ret = LW232_ERR;
            } else if (lw232_AutoPoll) {
                ret = LW232_OK_BIG;
            }
        }
        break;
    case LW232_CMD_RTR11:
        // riiil[CR] Transmit an standard RTR (11bit) CAN frame.
        if (lw232_CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            parseCanStdId(command);
            lw232_PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_STD_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            if (CAN_OK != sendMsgBuf(lw232_CanId, 0, 1, lw232_PacketLen, lw232_Buffer)) {
                ret = LW232_ERR;
            }
            else if (lw232_AutoPoll) {
                ret = LW232_OK_SMALL;
            }
        }
        else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_RTR29:
        // Riiiiiiiil[CR] Transmit an extended RTR (29bit) CAN frame.
        if (lw232_CanChannelMode == LW232_STATUS_CAN_OPEN_NORMAL) {
            parseCanExtId(command);
            lw232_PacketLen = parseNibbleWithLimit(command[LW232_OFFSET_EXT_PKT_LEN], LW232_FRAME_MAX_LENGTH);
            if (CAN_OK != sendMsgBuf(lw232_CanId, 1, 1, lw232_PacketLen, lw232_Buffer)) {
                ret = LW232_ERR;
            }
            else if (lw232_AutoPoll) {
                ret = LW232_OK_SMALL; // not a typo. strangely can232_v3.pdf tells to return "z[CR]", not "Z[CR]" as in 29bit. todo: check if it is error in pdf???
            }
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_POLL_ONE:
        // P[CR] Poll incomming FIFO for CAN frames (single poll)
        if (lw232_CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232_AutoPoll == LW232_AUTOPOLL_OFF) {
            if (CAN_MSGAVAIL == checkReceive()) {
                ret = receiveSingleFrame();
            }
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_POLL_MANY:
        // A[CR] Polls incomming FIFO for CAN frames (all pending frames)
        if (lw232_CanChannelMode != LW232_STATUS_CAN_CLOSED && lw232_AutoPoll == LW232_AUTOPOLL_OFF) {
            while (CAN_MSGAVAIL == checkReceive()) {
                ret = ret ^ receiveSingleFrame();
                if (ret != CAN_OK)
                    break;
                printChar(LW232_CR);
            }
            if (ret == CAN_OK)
                printChar(LW232_ALL);
        } else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_FLAGS:
        // F[CR] Read Status Flags.
        // LAWICEL CAN232 and CANUSB have some specific errors which differ from MCP2515/MCP2551 errors. We just return MCP2515 error.
        printChar(LW232_FLAG);
        if (checkError(&err) == CAN_OK) 
            err = 0;
        printFullByte(err & 0xFF); //MCP_EFLG_ERRORMASK);
        break;
    case LW232_CMD_AUTOPOLL:
        // Xn[CR] Sets Auto Poll/Send ON/OFF for received frames.
        if (lw232_CanChannelMode == LW232_STATUS_CAN_CLOSED) {
            lw232_AutoPoll = (command[1] == LW232_ON_ONE) ? LW232_AUTOPOLL_ON : LW232_AUTOPOLL_OFF;
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
        SYS_PRINT("%d",lw232_SerialBaudRates[idx]);
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
        if (lw232_CanChannelMode == LW232_STATUS_CAN_CLOSED) {
            // lw232_TimeStamp = (command[1] == LW232_ON_ONE); 
            if (command[1] == LW232_ON_ONE) {
                lw232_TimeStamp = LW232_TIMESTAMP_ON_NORMAL;
            }
            else if (command[1] == LW232_ON_TWO) {
                lw232_TimeStamp = LW232_TIMESTAMP_ON_EXTENDED;
            }
            else {
                lw232_TimeStamp = LW232_TIMESTAMP_OFF;
            }
        }
        else {
            ret = LW232_ERR;
        }
        break;
    case LW232_CMD_AUTOSTART:
        // Qn[CR] Auto Startup feature (from power on).
        if (lw232_CanChannelMode != LW232_STATUS_CAN_CLOSED) {
            if (command[1] == LW232_ON_ONE) {
                lw232_AutoStart = LW232_AUTOSTART_ON_NORMAL;
            }
            else if (command[1] == LW232_ON_TWO) {
                lw232_AutoStart = LW232_AUTOSTART_ON_LISTEN;
            }
            else {
                lw232_AutoStart = LW232_AUTOSTART_OFF;
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

    
    lw232_LastErr = ret; //parseAndRunCommand();

    switch (lw232_LastErr) {
    case LW232_OK:
        printChar(LW232_RET_ASCII_OK);
        break;
    case LW232_OK_SMALL:
        printChar(LW232_RET_ASCII_OK_SMALL);
        printChar(LW232_RET_ASCII_OK);
        break;
    case LW232_OK_BIG:
        printChar(LW232_RET_ASCII_OK_BIG);
        printChar(LW232_RET_ASCII_OK);
        break;
    case LW232_ERR_NOT_IMPLEMENTED:
        // Choose behavior: will it fail or not when not implemented command comes in. Some can monitors might be affected by this selection.
        printChar(LW232_RET_ASCII_ERROR);
        //SYS_PRINT(LW232_RET_ASCII_OK); 
        break;
    default:
        printChar(LW232_RET_ASCII_ERROR);
    }
}
