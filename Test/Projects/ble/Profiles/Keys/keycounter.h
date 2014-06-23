/**************************************************************************************************
  Filename:       keycounter.h
  Revised:        $Date: 2010-10-01 14:14:58 -0700 (Fri, 01 Oct 2010) $
  Revision:       $Revision: 23960 $

  Description:    This file contains the Simple Keys Profile header file.


  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef KEYCOUNTER_H
#define KEYCOUNTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define KC_KEY_ATTR                   0  // RW uint8 - Profile Attribute value
 
// KC Service UUID
#define KC_SERV_UUID                  0xFFE2
    
// Key Counter UUID
#define KC_KEYPRESSED_UUID            0xFFE3

// Key Values
#define SK_KEY_LEFT                   0x01
#define SK_KEY_RIGHT                  0x02

// Key Counter Profile Services bit fields
#define KC_SERVICE                    0x00000004
   
#define HEARTRATE_MEAS_NOTI_ENABLED         1
#define HEARTRATE_MEAS_NOTI_DISABLED        2

/*********************************************************************
 * TYPEDEFS
 */
typedef void (*keyCounterServiceCB_t)(uint8 event);
  
  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * SK_AddService- Initializes the Simple Key service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t KC_AddService( uint32 services );
  
/*
 * SK_SetParameter - Set a Simple Key Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t KC_SetParameter( uint8 param, uint8 len, void *pValue );
  
/*
 * SK_GetParameter - Get a Simple Key Profile parameter.
 *
 *    param - Profile parameter ID
 *    pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t KC_GetParameter( uint8 param, void *pValue );

extern void BLENotify();
/*********************************************************************
*********************************************************************/

/*********************************************************************
 * @fn          KeyCounter_MeasNotify
 *
 * @brief       Send a notification containing a key
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t KeyCounter_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

/*
 * KeyCounter_Register - Register a callback function with the
 *          Key Counter Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void KeyCounter_Register( keyCounterServiceCB_t pfnServiceCB );


#ifdef __cplusplus
}
#endif

#endif /* KEYCOUNTER_H */
