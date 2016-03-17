/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "can232.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
ssize_t SYS_CONSOLE_ReadX(const SYS_MODULE_INDEX index, int fd, void *buf, size_t count )
{
    size_t  numBytes = 0;
    char* pReadByte = (char*)buf;
    do
    {
        if( !DRV_USART0_ReceiverBufferIsEmpty() )
        {
            *pReadByte = DRV_USART0_ReadByte();

            numBytes++;
            pReadByte++;
        }
    } while( numBytes < count );
    return numBytes;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */


void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

static char cmdBuffer[64];
static int cmdTail = 0;
static bool echoInput = false;

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            SYS_CONSOLE_Flush( SYS_CONSOLE_INDEX_0 );
            
            //ssize_t nr;
            //char myBuffer[] = "\r\npic-CAN-mon";
            //nr = SYS_CONSOLE_Write( SYS_CONSOLE_INDEX_0, STDOUT_FILENO, myBuffer, strlen(myBuffer) );
            //if (nr != strlen(myBuffer))
            //{
            //    // Handle error
            //}
            SYS_PRINT("\r\npicCANmon");
            appData.state = APP_STATE_IDLE;
            break;
        }
        
        case APP_STATE_IDLE:
        {
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G,
                                 PORTS_BIT_POS_14);
           /* 
            CAN_CHANNEL_EVENT channelEvent;
            channelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_CHANNEL1);
            if((channelEvent & (CAN_RX_CHANNEL_NOT_EMPTY | CAN_RX_CHANNEL_FULL)) != 0) {
                SYS_MESSAGE("\r\nCAN RX\r\n");
                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G,
                                 PORTS_BIT_POS_12);
            }
            */
            
            CAN232_Tasks();
            
            ssize_t nr = 0;
            do {
                nr = SYS_CONSOLE_ReadX( SYS_CONSOLE_INDEX_0, STDIN_FILENO,
                                       &cmdBuffer[cmdTail], 0 );
                if (nr > 0)
                {
                    cmdTail += nr;
                    if (cmdTail >= sizeof(cmdBuffer))
                        cmdTail = 0;
                    //cmdBuffer[cmdTail] = 0;

                    appData.state = APP_STATE_INPUT;

                    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G,
                                         PORTS_BIT_POS_15);
                }
            } while (nr > 0);
            
            break;
        }
        
        case APP_STATE_INPUT:
        {
            appData.state = APP_STATE_IDLE;
            
            if (cmdTail > 0) {
                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G,
                                 PORTS_BIT_POS_13);
                
                char ch = cmdBuffer[cmdTail-1];
                if (ch < ' ') {
                    if (ch == '\r' || ch == '\n') {
                        cmdBuffer[cmdTail] = 0;
                        appData.state = APP_STATE_COMMAND;
                    }
                    cmdTail = 0;
                }
                else 
                {
                    // echo back
                    if (echoInput)
                        SYS_CONSOLE_Write( SYS_CONSOLE_INDEX_0, STDOUT_FILENO,
                                           &cmdBuffer[cmdTail-1], 1);
                }
                
            }
            break;
        }
        
        case APP_STATE_COMMAND: {
            
            //SYS_PRINT("\r\nCMD:%s", cmdBuffer);
            CAN232_Command(cmdBuffer);          
            
            cmdBuffer[0] = 0;   // Reset command buffer
            cmdTail = 0;
            
            appData.state = APP_STATE_IDLE;
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
