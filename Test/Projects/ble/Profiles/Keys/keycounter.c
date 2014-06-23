/**************************************************************************************************
  Filename:       keycounter.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    keycounter Profile


  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "keycounter.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        5

// Position of key measurement value in attribute array
#define KEY_COUNTER_POS            2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// SK Service UUID: 0x1800?
CONST uint8 kcServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(KC_SERV_UUID), HI_UINT16(KC_SERV_UUID)
};

// Key Pressed UUID: 0x1801?
CONST uint8 keyCounterUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(KC_KEYPRESSED_UUID), HI_UINT16(KC_KEYPRESSED_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static keyCounterServiceCB_t keyCounterServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// SK Service attribute
static CONST gattAttrType_t kcService = { ATT_BT_UUID_SIZE, kcServUUID };

// Keys Pressed Characteristic Properties
static uint8 kcCharProps = GATT_PROP_NOTIFY;

// Key Pressed State Characteristic
 uint32 kcKeyCounter = 0;

// Key Pressed Characteristic Configs
static gattCharCfg_t kcConfig[GATT_MAX_NUM_CONN];

// Key Pressed Characteristic User Description
static uint8 skCharUserDesp[16] = "Key Counter\0";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simplekeysAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Keys Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&kcService                       /* pValue */
  },

    // Characteristic Declaration for Keys
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &kcCharProps 
    },

      // Characteristic Value- Key Pressed
      { 
        { ATT_BT_UUID_SIZE, keyCounterUUID },
        0, 
        0, 
        (uint8 *)&kcKeyCounter 
      },

      // Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)kcConfig 
      },

      // Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        skCharUserDesp 
      },      
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 kc_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t kc_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void kc_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// SK Service Callbacks
CONST gattServiceCBs_t kcCBs =
{
  kc_ReadAttrCB,  // Read callback function pointer
  kc_WriteAttrCB, // Write callback function pointer
  NULL            // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      KC_AddService
 *
 * @brief   Initializes the Key Counter service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t KC_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, kcConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( kc_HandleConnStatusCB );  
  
  
  if ( services & KC_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simplekeysAttrTbl, 
                                          GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                          &kcCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      KeyCounter_Register
 *
 * @brief   Register a callback function with the Key Counter Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void KeyCounter_Register( keyCounterServiceCB_t pfnServiceCB )
{
  keyCounterServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      KC_SetParameter
 *
 * @brief   Set a Key Counter Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t KC_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case KC_KEY_ATTR:
      if ( len == sizeof ( uint32 ) ) 
      {
        kcKeyCounter = *((uint32*)pValue);
        
        // See if Notification/Indication has been enabled
        /*GATTServApp_ProcessCharCfg( kcConfig, (uint8 *)&kcKeyCounter, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );*/
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      KC_GetParameter
 *
 * @brief   Get a KC Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   pValue - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t KC_GetParameter( uint8 param, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case KC_KEY_ATTR:
      *((uint32*)pValue) = kcKeyCounter;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          kc_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 kc_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
 
  // Make sure it's not a blob operation (no attributes in the profile are long
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles this type for reads

      // simple keys characteristic does not have read permissions, but because it
      //   can be sent as a notification, it must be included here
      case KC_KEYPRESSED_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      kc_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t kc_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        
        if ( status == SUCCESS )
        {
          uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

          (*keyCounterServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
                                HEARTRATE_MEAS_NOTI_DISABLED :
                                HEARTRATE_MEAS_NOTI_ENABLED );
        }
        
        
        break;
       
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn          kc_HandleConnStatusCB
 *
 * @brief       Key Counter Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void kc_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, kcConfig );
    }
  }
}


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
bStatus_t KeyCounter_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, kcConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = simplekeysAttrTbl[KEY_COUNTER_POS].handle;
  
    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}




void BLENotify()
{
  
  GATTServApp_ProcessCharCfg( kcConfig, (uint8 *)&kcKeyCounter,
                                    FALSE, simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
}
