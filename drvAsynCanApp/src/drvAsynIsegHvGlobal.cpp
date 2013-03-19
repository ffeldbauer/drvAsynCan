//******************************************************************************
//! Copyright (C) 2012 Florian Feldbauer
//!
//! This program is free software; you can redistribute it and/or modify
//! it under the terms of the GNU General Public License as published by
//! the Free Software Foundation; either version 3 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//! GNU General Public License for more details.
//!
//! You should have received a copy of the GNU General Public License
//! along with this program; if not, write to the Free Software
//! Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//!
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   Asyn driver for ISEG EHS/EDS high voltage modules using the RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/* EPICS includes */
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

/* ASYN includes */
#include "asynDriver.h"
#include "asynGenericPointerSyncIO.h"
#include "asynStandardInterfaces.h"

#include "drvAsynIsegHvGlobal.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynIsegHvGlobalDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          If pasynUser->reason is equal to P_EmergencyOff this funciton
//!          switched all channels connected to CanPort off w/o ramp.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//!
//! @sa      drvAsynIsegHvGlobal::drvAsynIsegHvGlobal
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHvGlobal::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  /* Set the parameter in the parameter library. */
  status = (asynStatus) setIntegerParam( function, value );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, deviceName_, functionName, function, value );

  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = 0x004;
  pframe->can_dlc = 6;
  pframe->data[0] = 0xe8;
  pframe->data[1] = 0x00;
  pframe->data[2] = 0x60;
  pframe->data[3] = 0x01;
  pframe->data[4] = 0x00;
  pframe->data[5] = 0x20;

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, Could not send can frame.\033[0m", 
                   driverName, deviceName_, functionName, status, function );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->write().
//!
//!          If pasynUser->reason is equal to P_SwitchOnOff this function
//!          switches all channels connected to CanPort on / off.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//!
//! @sa      drvAsynIsegHvGlobal::drvAsynIsegHvGlobal
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHvGlobal::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  /* Set the parameter in the parameter library. */
  status = (asynStatus) setUIntDigitalParam( function, value, mask );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if ( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, functionName, function, value, mask );

  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = 0x004;
  pframe->can_dlc = 6;
  pframe->data[0] = 0xe8;
  pframe->data[1] = 0x00;
  pframe->data[2] = 0x60;
  pframe->data[3] = 0x01;
  pframe->data[4] = 0x00;
  pframe->data[5] = ( value ? 0x08 : 0x00 );

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Could not send can frame.\033[0m", 
                   driverName, deviceName_, functionName, function );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynIsegHvGlobal class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//------------------------------------------------------------------------------
drvAsynIsegHvGlobal::drvAsynIsegHvGlobal( const char *portName, const char *CanPort ) 
  : asynPortDriver( portName, 
                    1, /* maxAddr */ 
                    NUM_ISEGHVGLOBAL_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynIsegHvGlobal";
  asynStatus status = asynSuccess;

  deviceName_  = epicsStrDup( portName );

  createParam( P_ISEGHV_EMO_STRING,            asynParamInt32,         &P_EmergencyOff );
  createParam( P_ISEGHV_SWITCH_STRING,         asynParamUInt32Digital, &P_SwitchOnOff );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
             driverName, deviceName_, functionName, CanPort );
    return;
  }
  
  // Get inital value for Switch-Parameter
  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = ( 1 << 9 ) | 1;
  pframe->can_dlc = 3;
  pframe->data[0] = 0x40;
  pframe->data[1] = 0x01;
  pframe->data[2] = 0x00;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m %s:%s: Init %s: No reply from device within 1 s! \033[0m \n",
             driverName, functionName, P_ISEGHV_SWITCH_STRING );
    return;
  }
  if ( pframe->data[4] & 0x08 )
    setUIntDigitalParam( 0, P_SwitchOnOff, 1, 1 );
  else
    setUIntDigitalParam( 0, P_SwitchOnOff, 0, 1 );

}

//******************************************************************************
//! EOF
//******************************************************************************
