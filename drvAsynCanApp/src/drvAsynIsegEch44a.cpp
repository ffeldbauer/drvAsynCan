//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Ruhr-Universitaet Bochum, Lehrstuhl fuer Experimentalphysik I
//
// This file is part of drvAsynCan
//
// drvAsynCan is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// drvAsynCan is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// brief   Asyn driver for ISEG ECH44A Crate Controller
//
// version 1.0.0; May 16, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <linux/can.h>

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

#include "drvAsynIsegEch44a.h"
#include "ReadPoller.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynIsegEch44aDriver";
static const epicsUInt32 _can_id = 0x600;

//_____ F U N C T I O N S ______________________________________________________
static void myInterruptCallbackGenericPointer( void *userPvt,
                                               asynUser *pasynUser,
                                               void *pointer ) {
  drvAsynIsegEch44a* interface = static_cast<drvAsynIsegEch44a*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

//------------------------------------------------------------------------------
//! @brief   Callback for generic pointer interface
//!
//! This function works as callback for the asynGenericPointer Interface
//! and is called each time a can frame with id == can_id_ was received
//! by the lower level driver.
//! Depending on the the 1st two bytes the corresponding parameter will be
//! updated.
//!
//! @param   [in]  pointer    Address of the received can frame struct
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//!
//! @sa      drvAsynIsegEch44aChannel::drvAsynIsegEch44aChannel
//------------------------------------------------------------------------------
void drvAsynIsegEch44a::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  can_frame_t* pframe = (can_frame_t*)pointer;

  if ( pframe->can_id != ( _can_id | 1 )) return;

  // Send "log-on" message
  can_frame_t frame;
  frame.can_id  = _can_id;
  frame.can_dlc = 2;
  frame.data[0] = 0xd8;
  frame.data[1] = 0x01;
  _pasynUser->timeout = 0.5;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    std::cerr << printTimestamp() << " " << driverName << ":" <<  _deviceName
              << ":asynReadHandler"
              << ": pasynManager->queueLockPort: status=" << status
              << std::endl;
    return;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &frame );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    std::cerr << printTimestamp() << " " << driverName << ":" <<  _deviceName << ":asynReadHandler"
              << ": pasynManager->queueUnlockPort: status=" << status
              << std::endl;
    return;
  }
  if ( asynSuccess != status ){
    std::cerr << printTimestamp() << " " << driverName << ":" <<  _deviceName << ":asynReadHandler"
              << ": pasynGenericPointer->write: status=" << status
              << std::endl;
    return ;
  }
  return;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->read().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEch44a::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  const char* functionName = "readInt32";

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = _can_id | 1;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;

  _pasynUser->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  if ( asynSuccess != status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  } 
  status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return unlockStatus;
  }
  
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, _deviceName, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( pframe.can_id  != _can_id                 ||   \
       pframe.can_dlc != ( it->second.dlc + 1 )  ||   \
       pframe.data[0] != it->second.data0        ||   \
       pframe.data[1] != it->second.data1 ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x %02x, where %08x %d %02x %02x... was expected", 
                   driverName, _deviceName, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2], pframe.data[3],
                   _can_id, ( it->second.dlc + 2 ), it->second.data0, it->second.data1 );
    return asynError;
  }
  status = setIntegerParam( function, pframe.data[2] );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, functionName, status, function, *value );
    return status;
  }
  
  status = (asynStatus) getIntegerParam( function, value );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%d", 
                   driverName, _deviceName, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, _deviceName, functionName, function, *value );
  return status;


}
  
//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEch44a::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  const char* functionName = "writeInt32";
  
  //  if ( function != P_CrateOnOff ) return asynSuccess;
  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = _can_id;
  pframe.can_dlc = it->second.dlc + 1;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = ( value ) ? 1 : 0;

  _pasynUser->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return unlockStatus;
  }
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, _deviceName, functionName, status, function,
                   _pasynUser->errorMessage );
    return asynError;
  }

  /* Set the parameter in the parameter library. */
  status = (asynStatus) setIntegerParam( function, value );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, _deviceName, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, _deviceName, functionName, function, value );

  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEch44a::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readUInt32Digital";

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = _can_id | 1;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;

  _pasynUser->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  if ( asynSuccess != status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  } 
  status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return unlockStatus;
  }
  
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, _deviceName, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( pframe.can_id  != _can_id                 ||   \
       pframe.can_dlc != ( it->second.dlc + 4 )  ||   \
       pframe.data[0] != it->second.data0        ||   \
       pframe.data[1] != it->second.data1 ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x %02x, where %08x %d %02x %02x... was expected", 
                   driverName, _deviceName, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2], pframe.data[3],
                   _can_id, ( it->second.dlc + 2 ), it->second.data0, it->second.data1 );
    return asynError;
  }
  can_data_t helper;
  helper.can[3] = pframe.data[2];
  helper.can[2] = pframe.data[3];
  helper.can[1] = pframe.data[4];
  helper.can[0] = pframe.data[5];
  status = setUIntDigitalParam( function, helper.uval32, mask );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, functionName, status, function, *value, mask );
    return status;
  }

  status = (asynStatus) getUIntDigitalParam( function, value, mask );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%u mask=%u", 
                   driverName, _deviceName, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%u, mask=%u\n", 
               driverName, _deviceName, functionName, function, *value, mask );
  return status;

}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->write().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEch44a::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  if ( function != P_CrateControl ) return asynSuccess;
  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_data_t helper;
  helper.uval32 = value;

  can_frame_t pframe;
  pframe.can_id = _can_id;
  pframe.can_dlc = it->second.dlc + 4;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = helper.can[3];
  pframe.data[3] = helper.can[2];
  pframe.data[4] = helper.can[1];
  pframe.data[5] = helper.can[0];

  _pasynUser->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return unlockStatus;
  }
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, _deviceName, functionName, status, function,
                   _pasynUser->errorMessage );
    return asynError;
  }

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

  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynFloat64->read().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of value to read
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEch44a::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readFloat64";

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = _can_id | 1;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;

  _pasynUser->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  if ( asynSuccess != status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return status;
  } 
  status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, _deviceName, functionName, status, function );
    return unlockStatus;
  }
  
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, _deviceName, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( pframe.can_id  != _can_id                 ||   \
       pframe.can_dlc != ( it->second.dlc + 4 )  ||   \
       pframe.data[0] != it->second.data0        ||   \
       pframe.data[1] != it->second.data1 ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x %02x, where %08x %d %02x %02x... was expected", 
                   driverName, _deviceName, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2], pframe.data[3],
                   _can_id, ( it->second.dlc + 2 ), it->second.data0, it->second.data1 );
    return asynError;
  }
  can_data_t helper;
  helper.can[3] = pframe.data[2];
  helper.can[2] = pframe.data[3];
  helper.can[1] = pframe.data[4];
  helper.can[0] = pframe.data[5];
  status = setDoubleParam( function, helper.fval );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%lf\033[0m", 
                   driverName, functionName, status, function, *value );
    return status;
  }

  status = (asynStatus) getDoubleParam( function, value );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%lf", 
                   driverName, _deviceName, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%lf\n", 
               driverName, _deviceName, functionName, function, *value );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynIsegEch44a class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//------------------------------------------------------------------------------
drvAsynIsegEch44a::drvAsynIsegEch44a( const char *portName, const char *CanPort ) 
  : asynPortDriver( portName, 
                    1, /* maxAddr */ 
                    NUM_ISEGECH44A_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynIsegEch44a";
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  
  _deviceName  = epicsStrDup( portName );

  createParam( P_ISEGECH44A_CRATE_STATUS_STRING,       asynParamUInt32Digital, &P_CrateStatus );
  createParam( P_ISEGECH44A_CRATE_CONTROL_STRING,      asynParamUInt32Digital, &P_CrateControl );
  createParam( P_ISEGECH44A_CRATE_EVENT_STATUS_STRING, asynParamUInt32Digital, &P_CrateEvtStatus );
  createParam( P_ISEGECH44A_CRATE_EVENT_MASK_STRING,   asynParamUInt32Digital, &P_CrateEvtMask );
  createParam( P_ISEGECH44A_CRATE_FAN_SPEED_STRING,    asynParamFloat64,       &P_CrateFanSpeed );
  createParam( P_ISEGECH44A_CRATE_ON_OFF_STRING,       asynParamInt32,         &P_CrateOnOff );


  // Connect to asyn generic pointer port with asynGenericPointer interface
  _pasynUser = pasynManager->createAsynUser( NULL, NULL );
  _pasynUser->userPvt = this;

  status = pasynManager->connectDevice( _pasynUser, CanPort, 0 );
  if ( asynSuccess != status ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": Unable to connect Device"
              << std::endl;
    return;
  }
  
  asynInterface* pasynInterface;
    
  // find the asynCommon interface
  pasynInterface = pasynManager->findInterface( _pasynUser,
                                                asynCommonType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": bus " << CanPort << " does not support asynCommon interface"
              << std::endl;
    return;
  }
  _pasynCommon = static_cast<asynCommon*>( pasynInterface->pinterface );
  _pvtCommon   = pasynInterface->drvPvt;
  
  // find the asynGenericPointer interface
  pasynInterface = pasynManager->findInterface( _pasynUser,
                                                asynGenericPointerType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": bus " << CanPort << " does not support asynGenericPointer interface"
              << std::endl;
    return;
  }
  _pasynGenericPointer = static_cast<asynGenericPointer*>( pasynInterface->pinterface );
  _pvtGenericPointer   = pasynInterface->drvPvt;
  _pasynUser->reason = _can_id;
  status = _pasynGenericPointer->registerInterruptUser( _pvtGenericPointer,
                                                        _pasynUser,
                                                        myInterruptCallbackGenericPointer,
                                                        this,
                                                        &_intrPvtGenericPointer
                                                        );
  if( asynSuccess != status  ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": failed to register interrupt"
              << std::endl;
    return;
  }
  
  // BEGIN "log-off"-message-handling
  //  memcopy( pasynUserLogOn_, pasynUser_, size_of( *pasynUser_ ) );
  _pasynUser->reason = _can_id | 1;
  status = _pasynGenericPointer->registerInterruptUser( _pvtGenericPointer,
                                                        _pasynUser,
                                                        myInterruptCallbackGenericPointer,
                                                        this,
                                                        &_intrPvtGenericPointer
                                                        );
  if( asynSuccess != status  ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": failed to register interrupt"
              << std::endl;
    return;
  }
  // END "log-off"-message-handling

  // Send "log-on" message
  can_frame_t frame;
  frame.can_id  = _can_id;
  frame.can_dlc = 2;
  frame.data[0] = 0xd8;
  frame.data[1] = 0x01;
  _pasynUser->timeout = 0.5;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynManager->queueLockPort: status=" << status
              << std::endl;
    return;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &frame );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynManager->queueUnlockPort: status=" << status
              << std::endl;
    return;
  }
  if ( asynSuccess != status ){
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynGenericPointer->write: status=" << status
              << std::endl;
    return ;
  }

  // Init value for crate switch
  frame.can_id = _can_id | 1;
  frame.can_dlc = 2;
  frame.data[0] = 0x1a;
  frame.data[1] = 0x05;
  _pasynUser->timeout = 0.5;
  status = pasynManager->queueLockPort( _pasynUser );
  if( asynSuccess != status) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynManager->queueLockPort: status=" << status
              << std::endl;
    return;
  }
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &frame );
  if ( asynSuccess != status ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynGenericPointer->write: status=" << status
              << std::endl;
    return;
  } 
  status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &frame );
  unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
  if( asynSuccess != unlockStatus ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": pasynGenericPointer->read: status=" << status
              << std::endl;
    return;
  }
  status = (asynStatus) setIntegerParam( P_CrateOnOff, frame.data[2] );

  // Start polling
  ReadPoller::create( CanPort );
  
  isegFrame cmdCrateStatus    = { 2, 0x1a, 0x00 };
  isegFrame cmdCrateCtrl      = { 2, 0x1a, 0x01 };
  isegFrame cmdCrateEvtStatus = { 2, 0x1a, 0x02 };
  isegFrame cmdCrateEvtMask   = { 2, 0x1a, 0x03 };
  isegFrame cmdCrateFanSpeed  = { 2, 0x1a, 0x04 };
  isegFrame cmdCrateOnOff     = { 2, 0x1a, 0x05 };

  _cmds.insert( std::make_pair( P_CrateStatus,    cmdCrateStatus ) );
  _cmds.insert( std::make_pair( P_CrateControl,   cmdCrateCtrl ) );
  _cmds.insert( std::make_pair( P_CrateEvtStatus, cmdCrateEvtStatus ) );
  _cmds.insert( std::make_pair( P_CrateEvtMask,   cmdCrateEvtMask ) );
  _cmds.insert( std::make_pair( P_CrateFanSpeed,  cmdCrateFanSpeed ) );
  _cmds.insert( std::make_pair( P_CrateOnOff,     cmdCrateOnOff ) );


}

//******************************************************************************
//! EOF
//******************************************************************************
