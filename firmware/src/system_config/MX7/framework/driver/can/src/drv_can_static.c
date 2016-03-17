/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_static.c

  Summary:
    CAN driver implementation for the static single instance driver.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "../drv_can_static.h"


static CAN_TX_MSG_BUFFER  *drv_Message0;
static CAN_TX_MSG_BUFFER  can_message_buffer0[2*2*16];


// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CAN0_Initialize(void)
{	

    /* Switch the CAN module ON */
    PLIB_CAN_Enable(CAN_ID_1);
    
    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(CAN_ID_1, CAN_CONFIGURATION_MODE);
    while(PLIB_CAN_OperationModeGet(CAN_ID_1) != CAN_CONFIGURATION_MODE);

    PLIB_CAN_PhaseSegment2LengthFreelyProgrammableEnable(CAN_ID_1);

    //Set the Baud rate to 500 kbps
    PLIB_CAN_PropagationTimeSegmentSet(CAN_ID_1, 1-1);
    PLIB_CAN_PhaseSegment1LengthSet(CAN_ID_1, 3-1);
    PLIB_CAN_PhaseSegment2LengthSet(CAN_ID_1, 3-1);
    PLIB_CAN_SyncJumpWidthSet(CAN_ID_1, 1-1);
    PLIB_CAN_BaudRatePrescaleSet(CAN_ID_1, 0x9); // set to 1 higher then ECAN tool


    /* Assign the buffer area to the CAN module.
      In this case assign enough memory for 2
      channels, each with 8 message buffers.*/
     PLIB_CAN_MemoryBufferAssign(CAN_ID_1, can_message_buffer0);

     /* Configure CAN_ID_1 Channel for CAN_TX_RTR_ENABLED operation. Allocate 1 message buffer, and assign low medium priority for transmissions. */			  
     PLIB_CAN_ChannelForTransmitSet(CAN_ID_1, CAN_CHANNEL0, 1, CAN_TX_RTR_ENABLED, CAN_LOW_MEDIUM_PRIORITY);
     /* Configure CAN_ID_1 Channel for CAN_RX_FULL_RECEIVE operation. Allocate 1 message buffer, and assign low medium priority for transmissions. */			
     PLIB_CAN_ChannelForReceiveSet(CAN_ID_1, CAN_CHANNEL1, 1, CAN_RX_FULL_RECEIVE);
     PLIB_CAN_FilterToChannelLink(CAN_ID_1, CAN_FILTER0, CAN_FILTER_MASK0, CAN_CHANNEL1);
     PLIB_CAN_ChannelEventEnable(CAN_ID_1, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY);


     PLIB_CAN_FilterConfigure(CAN_ID_1, CAN_FILTER0, 0x101, CAN_SID);
     PLIB_CAN_FilterEnable(CAN_ID_1, CAN_FILTER0);

     PLIB_CAN_FilterMaskConfigure(CAN_ID_1, CAN_FILTER_MASK0, 0x0, CAN_SID, CAN_FILTER_MASK_IDE_TYPE);

    /* Switch the CAN module to Normal mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(CAN_ID_1, CAN_NORMAL_MODE);
    while(PLIB_CAN_OperationModeGet(CAN_ID_1) != CAN_NORMAL_MODE);

}

void DRV_CAN0_DeInitialize(void)
{

    /* Switch the CAN module to Disable mode. Wait until the switch is complete */
    PLIB_CAN_OperationModeSelect(CAN_ID_1, CAN_DISABLE_MODE);
    while(PLIB_CAN_OperationModeGet(CAN_ID_1) != CAN_DISABLE_MODE);	

}

void DRV_CAN0_Open(void)
{
   /* Switch the CAN module ON */
    PLIB_CAN_Enable(CAN_ID_1);
}

void DRV_CAN0_Close(void)
{
   /* Switch the CAN module OFF */
    PLIB_CAN_Disable(CAN_ID_1);
}

bool DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{

    int count = 0;
    drv_Message0 = (CAN_TX_MSG_BUFFER *) &can_message_buffer0[channelNum];
    drv_Message0->msgSID.sid     = address;
    drv_Message0->msgEID.ide     = 0;
    drv_Message0->msgEID.data_length_code     = DLC;

    while(count < DLC)
    {
    	drv_Message0->data[count++] = *message++;
    }

    //Update CAN module and then transmit data on the bus;
    PLIB_CAN_ChannelUpdate(CAN_ID_1, channelNum);
    PLIB_CAN_TransmitChannelFlush(CAN_ID_1, channelNum);
    return(true);
   
}

bool DRV_CAN0_ChannelMessageReceive(CAN_CHANNEL channelNum, int address, uint8_t DLC, uint8_t* message)
{
    int EchoDLC = 0;
    CAN_RX_MSG_BUFFER *RxMessage;     
    RxMessage = (CAN_RX_MSG_BUFFER *)PLIB_CAN_ReceivedMessageGet(CAN_ID_1, channelNum);

    if(RxMessage != NULL)
    {
        
        if (RxMessage->msgEID.data_length_code > 0)
        {
        
            if(RxMessage->msgSID.sid == address)
            {
                while(EchoDLC < RxMessage->msgEID.data_length_code)
                { 
                     *message++ = RxMessage->data[EchoDLC++];
                }
            }
        }

        //Process the message received

        /* Call the PLIB_CAN_ChannelUpdate function to let
        the CAN module know that the message processing
        is done. */
        PLIB_CAN_ChannelUpdate(CAN_ID_1, channelNum);

        return( true );
    }
    return( false );
}

/*******************************************************************************
 End of File
*/
