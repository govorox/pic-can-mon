/* 
 * File:   can232.h
 * Author: vadim
 *
 * Created on 16 March 2016, 12:16
 */

#ifndef CAN232_H
#define	CAN232_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
#define LW232_LAWICEL_VERSION_STR     "V1013"
#define LW232_LAWICEL_SERIAL_NUM      "NA123"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//   Commands not supported/not implemented:
//     s, W, M, m, U
//      
//   Commands modified:
//     S - supports not declared 83.3 rate straight away (S9) fork with 83.3 support, easy to add.
//     F - returns error flags
//     Z - extra Z2 option enables 4 byte timestamp vs standard 2 byte (60000ms max)
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                   
//                          CODE   SUPPORTED   SYNTAX               DESCRIPTION     
//
#define LW232_CMD_SETUP     'S' //   YES+      Sn[CR]               Setup with standard CAN bit-rates where n is 0-8.
                                //                                  S0 10Kbit          S4 125Kbit         S8 1Mbit
                                //                                  S1 20Kbit          S5 250Kbit         S9 83.3Kbit
                                //                                  S2 50Kbit          S6 500Kbit
                                //                                  S3 100Kbit         S7 800Kbit
#define LW232_CMD_SETUP_BTR 's' //    -        sxxyy[CR]            Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
#define LW232_CMD_OPEN      'O' //   YES       O[CR]                Open the CAN channel in normal mode (sending & receiving).
#define LW232_CMD_LISTEN    'L' //   YES       L[CR]                Open the CAN channel in listen only mode (receiving).
#define LW232_CMD_CLOSE     'C' //   YES       C[CR]                Close the CAN channel.
#define LW232_CMD_TX11      't' //   YES       tiiildd...[CR]       Transmit a standard (11bit) CAN frame.
#define LW232_CMD_TX29      'T' //   YES       Tiiiiiiiildd...[CR]  Transmit an extended (29bit) CAN frame
#define LW232_CMD_RTR11     'r' //   YES       riiil[CR]            Transmit an standard RTR (11bit) CAN frame.
#define LW232_CMD_RTR29     'R' //   YES       Riiiiiiiil[CR]       Transmit an extended RTR (29bit) CAN frame.
#define LW232_CMD_POLL_ONE  'P' //   YES       P[CR]                Poll incomming FIFO for CAN frames (single poll)
#define LW232_CMD_POLL_MANY 'A' //   YES       A[CR]                Polls incomming FIFO for CAN frames (all pending frames)
#define LW232_CMD_FLAGS     'F' //   YES+      F[CR]                Read Status Flags.
#define LW232_CMD_AUTOPOLL  'X' //   YES       Xn[CR]               Sets Auto Poll/Send ON/OFF for received frames.
#define LW232_CMD_FILTER    'W' //    -        Wn[CR]               Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
#define LW232_CMD_ACC_CODE  'M' //    -        Mxxxxxxxx[CR]        Sets Acceptance Code Register (ACn Register of SJA1000). // we use MCP2515, not supported
#define LW232_CMD_ACC_MASK  'm' //    -        mxxxxxxxx[CR]        Sets Acceptance Mask Register (AMn Register of SJA1000). // we use MCP2515, not supported
#define LW232_CMD_UART      'U' //   YES       Un[CR]               Setup UART with a new baud rate where n is 0-6.
#define LW232_CMD_VERSION1  'V' //   YES       v[CR]                Get Version number of both CAN232 hardware and software
#define LW232_CMD_VERSION2  'v' //   YES       V[CR]                Get Version number of both CAN232 hardware and software
#define LW232_CMD_SERIAL    'N' //   YES       N[CR]                Get Serial number of the CAN232.
#define LW232_CMD_TIMESTAMP 'Z' //   YES+      Zn[CR]               Sets Time Stamp ON/OFF for received frames only. EXTENSION to LAWICEL: Z2 - millis() timestamp w/o standard 60000ms cycle
#define LW232_CMD_AUTOSTART 'Q' //   YES  todo     Qn[CR]               Auto Startup feature (from power on). 

#define LOW_BYTE(x)     ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x)    ((unsigned char)(((x)>>8)&0xFF))
#define LOW_WORD(x)     ((unsigned short)((x)&0xFFFF))
#define HIGH_WORD(x)    ((unsigned short)(((x)>>16)&0xFFFF))

#ifndef INT32U
#define INT32U uint32_t
#endif

#ifndef INT16U
#define INT16U uint16_t
#endif

#ifndef INT8U
#define INT8U uint8_t
#endif

#define LW232_OK                      0x00
#define LW232_OK_SMALL                0x01
#define LW232_OK_BIG                  0x02
#define LW232_ERR                     0x03
#define LW232_ERR_NOT_IMPLEMENTED     0x04
#define LW232_ERR_UNKNOWN_CMD         0x05

#define LW232_FILTER_SKIP			  0x00
#define LW232_FILTER_PROCESS	      0x01

//#define LW232_IS_OK(x) ((x)==LW232_OK ||(x)==LW232_OK_NEW ? TRUE : FALSE)

#define LW232_CR    '\r'
#define LW232_ALL   'A'
#define LW232_FLAG  'F'
#define LW232_TR11  't'
#define LW232_TR29  'T'

#define LW232_RET_ASCII_OK             0x0D
#define LW232_RET_ASCII_ERROR          0x07
#define LW232_RET_ASCII_OK_SMALL       'z'
#define LW232_RET_ASCII_OK_BIG         'Z'

#define LW232_STATUS_CAN_CLOSED        0x00
#define LW232_STATUS_CAN_OPEN_NORMAL   0x01
#define LW232_STATUS_CAN_OPEN_LISTEN   0x01

#define LW232_FRAME_MAX_LENGTH         0x08
#define LW232_FRAME_MAX_SIZE           (sizeof("Tiiiiiiiildddddddddddddddd\r")+1)

#define LW232_OFF                      '0'
#define LW232_ON_ONE                   '1'
#define LW232_ON_TWO                   '2'

#define LW232_AUTOPOLL_OFF             0x00
#define LW232_AUTOPOLL_ON              0x01

#define LW232_AUTOSTART_OFF            0x00
#define LW232_AUTOSTART_ON_NORMAL      0x01
#define LW232_AUTOSTART_ON_LISTEN      0x02

#define LW232_TIMESTAMP_OFF            0x00
#define LW232_TIMESTAMP_ON_NORMAL      0x01
#define LW232_TIMESTAMP_ON_EXTENDED    0x02

#define LW232_OFFSET_STD_PKT_LEN       0x04
#define LW232_OFFSET_STD_PKT_DATA      0x05
#define LW232_OFFSET_EXT_PKT_LEN       0x09
#define LW232_OFFSET_EXT_PKT_DATA      0x0A


#define LW232_DEFAULT_BAUD_RATE        115200
#define LW232_CAN_BAUD_NUM             0x0a
#define LW232_UART_BAUD_NUM            0x07


#define CAN_5KBPS    1
#define CAN_10KBPS   2
#define CAN_20KBPS   3
#define CAN_31K25BPS 4
#define CAN_33KBPS   5
#define CAN_40KBPS   6
#define CAN_50KBPS   7
#define CAN_80KBPS   8
#define CAN_83K3BPS  9
#define CAN_95KBPS   10
#define CAN_100KBPS  11
#define CAN_125KBPS  12
#define CAN_200KBPS  13
#define CAN_250KBPS  14
#define CAN_500KBPS  15
#define CAN_1000KBPS 16

#define CAN_OK                  (0)
#define CAN_FAILINIT            (1)
#define CAN_FAILTX              (2)
#define CAN_MSGAVAIL            (3)
#define CAN_NOMSG               (4)
#define CAN_CTRLERROR           (5)
#define CAN_GETTXBFTIMEOUT      (6)
#define CAN_SENDMSGTIMEOUT      (7)
#define CAN_FAIL                (0xff)

#define CAN_MAX_CHAR_IN_MESSAGE (8)

// Function prototypes
    
void CAN232_Command(char * command);
void CAN232_Tasks();
    

#ifdef	__cplusplus
}
#endif

#endif	/* CAN232_H */

