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
// brief   Asyn driver for ISEG EHS/EDS high voltage modules
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C++ includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

// EPICS includes
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

// ASYN includes
#include "asynDriver.h"
#include "asynGenericPointer.h"
#include "asynStandardInterfaces.h"

// local includes
#include "drvAsynIsegEhsEds.h"
#include "ReadPoller.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//! used to convert 32-bit float (IEEE 754) to 4*8 bit unsigned int
typedef union{
  epicsFloat32 fval;
  epicsUInt8   val[4];
} can_float_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynIsegHVDriver";

//_____ F U N C T I O N S ______________________________________________________
static void myInterruptCallbackGenericPointer( void *userPvt,
                                               asynUser *pasynUser,
                                               void *pointer ) {
  drvAsynIsegEhsEds* interface = static_cast<drvAsynIsegEhsEds*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

//------------------------------------------------------------------------------
//! @brief   Callback for generic pointer interface
//!
//! This function works as callback for the asynGenericPointer Interface
//! and is called each time a can frame with id == _can_id was received
//! by the lower level driver.
//! Depending on the the 1st two bytes the corresponding parameter will be
//! updated.
//!
//! @param   [in]  pointer    Address of the received can frame struct
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
void drvAsynIsegEhsEds::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  can_frame_t* pframe = (can_frame_t*)pointer;

  if( pframe->can_id == ( _can_id | 1 ) ) { 
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
  int opcode = ( pframe->data[0] << 8 ) | pframe->data[1];
  int addr = pframe->data[2];
  can_float_t myValue;
  
  switch( opcode ){
    
    // Channel Status
  case 0x6000:
    status = setUIntDigitalParam( addr, P_ChanStatus, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Control
  case 0x6001:
    status = setUIntDigitalParam( addr, P_ChanCtrl, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Event Status
  case 0x6002: 
    status = setUIntDigitalParam( addr, P_ChanEventStatus, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Event Mask
  case 0x6003: 
    status = setUIntDigitalParam( addr, P_ChanEventMask, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Set voltage
  case 0x6100: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_ChanVset, myValue.fval );
    break;

    // Set current/trip
  case 0x6101: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_ChanIset, myValue.fval );
    break;

    // Voltage measurement
  case 0x6102: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_ChanVmom, myValue.fval );
    break;

    // Current measurement
  case 0x6103:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_ChanImom, myValue.fval );
    break;

    // Current measurement range
  case 0x6109:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_ChanImom, myValue.fval );
    status = setIntegerParam( addr, P_ChanImomRange, pframe->data[7] );
    break;

    // Voltage bounds
  case 0x6104: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_ChanVbounds, myValue.fval );
    break;

    // Current bounds
  case 0x6105:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_ChanIbounds, myValue.fval );
    break;

    // Nominal Voltage
  case 0x6106: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_ChanVnom, myValue.fval );
    break;

    // Nominal Current
  case 0x6107:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_ChanInom, myValue.fval );
    break;

  default:
    return;
  }
  
  if ( asynSuccess != status ) {
    std::cerr << printTimestamp() << " " << driverName << ":" <<  _deviceName << ":asynReadHandler: Error"
              << std::endl;
    return;
  }
  callParamCallbacks( addr, addr );
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
asynStatus drvAsynIsegEhsEds::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "writeUInt32Digital";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  epicsUInt8 offset = ( 16 * addr );
  epicsUInt16 chanMsk = 0;
  int channels = maxAddr - ( 16 * addr );
  for ( int i = 0; i < channels && i < 16; i++ ) chanMsk |= ( 1 << i );

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id  = _can_id | 1;
  pframe.can_dlc = it->second.dlc + 2;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( ( chanMsk & 0xff00 ) >> 8 );
  pframe.data[3] = (epicsUInt8)( chanMsk & 0x00ff );
  pframe.data[4] = offset;

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
asynStatus drvAsynIsegEhsEds::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "writeUInt32Digital";

  if ( function == P_ModStatus ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_dlc = it->second.dlc + 2;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  epicsUInt8 offset = ( 16 * addr );
  if ( function == P_ChanCtrl ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)addr;
    pframe.data[3] = 0;
    pframe.data[4] = (epicsUInt8)( value & 0x00000028 );
  }
  if ( function == P_ChanEventStatus ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)addr;
    pframe.data[3] = (epicsUInt8)( ( value & 0x0000fc00 ) >> 8 );
    pframe.data[4] = (epicsUInt8)(   value & 0x000000fc );
  }
  if ( function == P_ChanEventMask ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)addr;
    pframe.data[3] = (epicsUInt8)( ( value & 0x0000fc00 ) >> 8 );
    pframe.data[4] = (epicsUInt8)(   value & 0x000000dc );
  }
  if ( function == P_ModCtrl ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)( ( value & 0x00005800 ) >> 8 );
    pframe.data[3] = (epicsUInt8)(   value & 0x00000060 );    
  }
  if ( function == P_ModEventStatus ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)( ( value & 0x00006400 ) >> 8 );
    pframe.data[3] = (epicsUInt8)(   value & 0x00000074 );    
  }
  if ( function == P_ModEventMask ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)( ( value & 0x00006400 ) >> 8 );
    pframe.data[3] = (epicsUInt8)(   value & 0x00000060 );    
  }
  if ( function == P_ModEventChanStatus ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = offset;
    pframe.data[3] = (epicsUInt8)( ( value & 0x0000ff00 ) >> 8 );
    pframe.data[4] = (epicsUInt8)(   value & 0x000000ff );    
  }
  if ( function == P_ModEventChanMask ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = offset;
    pframe.data[3] = (epicsUInt8)( ( value & 0x0000ff00 ) >> 8 );
    pframe.data[4] = (epicsUInt8)(   value & 0x000000ff );    
  }
  if ( function == P_ModEventGrpStatus ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)( ( value & 0x0000ff00 ) >> 8 );
    pframe.data[3] = (epicsUInt8)(   value & 0x000000ff );    
  }
  if ( function == P_ModEventGrpMask ) {
    pframe.can_id  = _can_id;
    pframe.data[2] = (epicsUInt8)( ( value & 0x00006400 ) >> 8 );
    pframe.data[3] = (epicsUInt8)(   value & 0x00000060 );    
  }

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

  status = setUIntDigitalParam( addr, function, value, mask );
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
asynStatus drvAsynIsegEhsEds::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readUInt32Digital";
    
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( function == P_ModStatus          ||
       function == P_ModCtrl            ||
       function == P_ModEventStatus     ||
       function == P_ModEventMask       ||
       function == P_ModEventChanStatus ||
       function == P_ModEventChanMask   ||
       function == P_ModEventGrpStatus  ||
       function == P_ModEventGrpMask    ) {
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
    if ( pframe.can_id  != _can_id                 || \
         pframe.can_dlc != ( it->second.dlc + 2 )  || \
         pframe.data[0] != it->second.data0        || \
         pframe.data[1] != it->second.data1 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x %02x, where %08x %d %02x %02x... was expected", 
                     driverName, _deviceName, functionName, function,
                     pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2], pframe.data[3],
                     _can_id, ( it->second.dlc + 2 ), it->second.data0, it->second.data1 );
      return asynError;
    }
    status = setUIntDigitalParam( addr, function, ( pframe.data[2] << 8 ) | pframe.data[3], mask );
  }
  
  status = (asynStatus) getUIntDigitalParam(addr, function, value, mask);
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
//! @brief   Called when asyn clients call pasynFloat64->write().
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegEhsEds::writeFloat64( asynUser *pasynUser, epicsFloat64 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  int addr = 0;
  const char *functionName = "writeFloat64";

  if ( function == P_ChanVmom  || \
       function == P_ChanImom  || \
       function == P_ChanVnom  || \
       function == P_ChanInom  || \
       function == P_Vmax      || \
       function == P_Imax      || \
       function == P_Supply24  || \
       function == P_Supply5   ||              \
       function == P_Temperature )
    return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
  if( it == _cmds.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = _can_id;
  pframe.can_dlc = it->second.dlc + 4;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( addr & 0xff );
  
  can_float_t myValue;
  myValue.fval = value;

  // convert current from uA to A
  if ( function == P_ChanIset || function == P_ChanIbounds ) myValue.fval *= 1.e-6;
  
  pframe.data[ it->second.dlc ]     = myValue.val[3];
  pframe.data[ it->second.dlc + 1 ] = myValue.val[2];
  pframe.data[ it->second.dlc + 2 ] = myValue.val[1];
  pframe.data[ it->second.dlc + 3 ] = myValue.val[0];

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

  if ( asynSuccess != status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, _deviceName, functionName, status, function,
                   _pasynUser->errorMessage );
    return asynError;
  }
  
  // Set the parameter and readback in the parameter library.
  status = setDoubleParam( addr, function, value );
  callParamCallbacks( addr, addr );
  if ( status ) 
    asynPrint( pasynUser, ASYN_TRACE_ERROR, 
               "%s:%s:%s: error, status=%d function=%d, value=%f\n", 
               driverName, _deviceName, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%f\n", 
               driverName, _deviceName, functionName, function, value );
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
asynStatus drvAsynIsegEhsEds::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readFloat64";
  
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( function == P_VRampSpeed || \
       function == P_IRampSpeed || \
       function == P_Vmax       || \
       function == P_Imax       || \
       function == P_Supply24   || \
       function == P_Supply5    || \
       function == P_Temperature ) {
    std::map<int, isegFrame>::const_iterator it = _cmds.find( function );
    if( it == _cmds.end() ) return asynError;
    can_frame_t pframe;
    pframe.can_id  = _can_id | 1;
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
    if ( pframe.can_id  != _can_id                 || \
         pframe.can_dlc != ( it->second.dlc + 4 )  || \
         pframe.data[0] != it->second.data0        || \
         pframe.data[1] != it->second.data1 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x %02x %02x %02x, where %08x %d %02x %02x... was expected", 
                     driverName, _deviceName, functionName, function,
                     pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1],
                     pframe.data[2], pframe.data[3], pframe.data[4], pframe.data[5],
                     _can_id, ( it->second.dlc + 4 ), it->second.data0, it->second.data1 );
      return asynError;
    }
    can_float_t myValue;
    myValue.val[3] = pframe.data[2];
    myValue.val[2] = pframe.data[3];
    myValue.val[1] = pframe.data[4];
    myValue.val[0] = pframe.data[5];
    status = setDoubleParam( addr, function, myValue.fval );
  }

  status = (asynStatus) getDoubleParam( addr, function, value );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%f", 
                   driverName, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s: function=%d, value=%f\n", 
               driverName, functionName, function, *value );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynIsegEhsEds class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id      The can id of this module
//! @param   [in]  channels    Number of channels this module has
//------------------------------------------------------------------------------
drvAsynIsegEhsEds::drvAsynIsegEhsEds( const char *portName,
                                      const char *CanPort,
                                      const int can_id,
                                      const int channels ) 
  : asynPortDriver( portName, 
                    channels, // maxAddr
                    NUM_ISEGEHSEDS_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  // Interrupt mask
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
                    1, // Autoconnect
                    0, // Default priority
                    0 ) // Default stack size
{
  const char *functionName = "drvAsynIsegEhsEds";
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  
  _deviceName  = epicsStrDup( portName );
  _can_id      = can_id; //( 1 << 9 ) | ( module_id << 3 );
  // epicsUInt16 chanMsk = 0;
  // for ( int i = 0; i < channels || i < 16; i++ ) chanMsk |= ( 1 << i );

  // Create parameters
  // Channel related parameters which hold the values
  createParam( P_ISEGEHSEDS_CHANSTATUS_STRING,       asynParamUInt32Digital, &P_ChanStatus );
  createParam( P_ISEGEHSEDS_CHAN_CTRL_STRING,        asynParamUInt32Digital, &P_ChanCtrl );
  createParam( P_ISEGEHSEDS_CHANEVTSTATUS_STRING,    asynParamUInt32Digital, &P_ChanEventStatus );
  createParam( P_ISEGEHSEDS_CHANEVTMASK_STRING,      asynParamUInt32Digital, &P_ChanEventMask );
  createParam( P_ISEGEHSEDS_VSET_STRING,             asynParamFloat64,       &P_ChanVset );
  createParam( P_ISEGEHSEDS_ISET_STRING,             asynParamFloat64,       &P_ChanIset );
  createParam( P_ISEGEHSEDS_VMOM_STRING,             asynParamFloat64,       &P_ChanVmom );
  createParam( P_ISEGEHSEDS_IMOM_STRING,             asynParamFloat64,       &P_ChanImom );
  createParam( P_ISEGEHSEDS_VBOUNDS_STRING,          asynParamFloat64,       &P_ChanVbounds );
  createParam( P_ISEGEHSEDS_IBOUNDS_STRING,          asynParamFloat64,       &P_ChanIbounds );
  createParam( P_ISEGEHSEDS_VNOM_STRING,             asynParamFloat64,       &P_ChanInom );
  createParam( P_ISEGEHSEDS_INOM_STRING,             asynParamFloat64,       &P_ChanInom );
  createParam( P_ISEGEHSEDS_IMOM_RANGE_STRING,       asynParamInt32,         &P_ChanImomRange );
  // Channel related parameters used as commands (no values)
  createParam( P_ISEGEHSEDS_R_CHANSTATUS_STRING,     asynParamInt32,         &P_R_ChanStatus );
  createParam( P_ISEGEHSEDS_R_CHAN_CTRL_STRING,      asynParamInt32,         &P_R_ChanCtrl );
  createParam( P_ISEGEHSEDS_R_CHANEVTSTATUS_STRING,  asynParamInt32,         &P_R_ChanEventStatus );
  createParam( P_ISEGEHSEDS_R_CHANEVTMASK_STRING,    asynParamInt32,         &P_R_ChanEventMask );
  createParam( P_ISEGEHSEDS_R_VSET_STRING,           asynParamInt32,         &P_R_ChanVset );
  createParam( P_ISEGEHSEDS_R_ISET_STRING,           asynParamInt32,         &P_R_ChanIset );
  createParam( P_ISEGEHSEDS_R_VMOM_STRING,           asynParamInt32,         &P_R_ChanVmom );
  createParam( P_ISEGEHSEDS_R_IMOM_STRING,           asynParamInt32,         &P_R_ChanImom );
  createParam( P_ISEGEHSEDS_R_VBOUNDS_STRING,        asynParamInt32,         &P_R_ChanVbounds );
  createParam( P_ISEGEHSEDS_R_IBOUNDS_STRING,        asynParamInt32,         &P_R_ChanIbounds );
  createParam( P_ISEGEHSEDS_R_VNOM_STRING,           asynParamInt32,         &P_R_ChanVnom );
  createParam( P_ISEGEHSEDS_R_INOM_STRING,           asynParamInt32,         &P_R_ChanInom );
  createParam( P_ISEGEHSEDS_R_IMOM_RANGE_STRING,     asynParamInt32,         &P_R_ChanImomRange );
  // Module related parameters
  createParam( P_ISEGEHSEDS_MODSTATUS_STRING,        asynParamUInt32Digital, &P_ModStatus );
  createParam( P_ISEGEHSEDS_MODCTRL_STRING,          asynParamUInt32Digital, &P_ModCtrl );
  createParam( P_ISEGEHSEDS_MODEVTSTATUS_STRING,     asynParamUInt32Digital, &P_ModEventStatus );
  createParam( P_ISEGEHSEDS_MODEVTMASK_STRING,       asynParamUInt32Digital, &P_ModEventMask );
  createParam( P_ISEGEHSEDS_MODEVTCHANSTATUS_STRING, asynParamUInt32Digital, &P_ModEventChanStatus );
  createParam( P_ISEGEHSEDS_MODEVTCHANMASK_STRING,   asynParamUInt32Digital, &P_ModEventChanMask );
  createParam( P_ISEGEHSEDS_MODEVTGRPSTATUS_STRING,  asynParamUInt32Digital, &P_ModEventGrpStatus );
  createParam( P_ISEGEHSEDS_MODEVTGRPMASK_STRING,    asynParamUInt32Digital, &P_ModEventGrpMask );
  createParam( P_ISEGEHSEDS_VRAMPSPEED_STRING,       asynParamFloat64,       &P_VRampSpeed );
  createParam( P_ISEGEHSEDS_IRAMPSPEED_STRING,       asynParamFloat64,       &P_IRampSpeed );
  createParam( P_ISEGEHSEDS_VMAX_STRING,             asynParamFloat64,       &P_Vmax );
  createParam( P_ISEGEHSEDS_IMAX_STRING,             asynParamFloat64,       &P_Imax );
  createParam( P_ISEGEHSEDS_SUPPLY24_STRING,         asynParamFloat64,       &P_Supply24 );
  createParam( P_ISEGEHSEDS_SUPPLY5_STRING,          asynParamFloat64,       &P_Supply5 );
  createParam( P_ISEGEHSEDS_TEMPERATURE_STRING,      asynParamFloat64,       &P_Temperature );

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

  // Start polling
  ReadPoller::create( CanPort );
  
  // Get some initial values
  epicsUInt8 j = 0; 
  int num = channels - ( 16 * j );
  while ( num > 0 ) {
    epicsUInt16 chanMsk = 0;
    for ( int i = 0; i < num && i < 16; i++ ) chanMsk |= ( 1 << i );

    // chan ctrl
    frame.can_id  = _can_id | 1;
    frame.can_dlc = 5;
    frame.data[0] = 0x60;
    frame.data[1] = 0x01;
    frame.data[2] = (epicsUInt8)( ( chanMsk & 0xff00 ) >> 8 );
    frame.data[3] = (epicsUInt8)( chanMsk & 0x00ff );
    frame.data[4] = ( 16 * j );
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

    // chan vset
    frame.can_id  = _can_id | 1;
    frame.can_dlc = 5;
    frame.data[0] = 0x61;
    frame.data[1] = 0x00;
    frame.data[2] = (epicsUInt8)( ( chanMsk & 0xff00 ) >> 8 );
    frame.data[3] = (epicsUInt8)( chanMsk & 0x00ff );
    frame.data[4] = ( 16 * j );
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
    
    // chan iset
    frame.can_id  = _can_id | 1;
    frame.can_dlc = 5;
    frame.data[0] = 0x61;
    frame.data[1] = 0x01;
    frame.data[2] = (epicsUInt8)( ( chanMsk & 0xff00 ) >> 8 );
    frame.data[3] = (epicsUInt8)( chanMsk & 0x00ff );
    frame.data[4] = ( 16 * j );
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
  
    j++;
    num = channels - ( 16 * j );
  }

  isegFrame chstat_cmd     = { 3, 0x60, 0x00 };
  isegFrame chanctrl_r_cmd = { 3, 0x60, 0x01 };
  isegFrame chesta_cmd     = { 3, 0x60, 0x02 };
  isegFrame chemsk_cmd     = { 3, 0x60, 0x03 };
  isegFrame vset_r_cmd     = { 3, 0x61, 0x00 };
  isegFrame iset_r_cmd     = { 3, 0x61, 0x01 };
  isegFrame vmom_cmd       = { 3, 0x61, 0x02 };
  isegFrame imom_cmd       = { 3, 0x61, 0x03 };
  isegFrame imom_range_cmd = { 3, 0x61, 0x09 }; // for device class 26 and 27, E08F2 and E08C2 only !!
  isegFrame vbounds_r_cmd  = { 3, 0x61, 0x04 };
  isegFrame ibounds_r_cmd  = { 3, 0x61, 0x05 };
  isegFrame vnom_cmd       = { 3, 0x61, 0x06 };
  isegFrame inom_cmd       = { 3, 0x61, 0x07 };
  isegFrame modstat_cmd    = { 2, 0x10, 0x00 };
  isegFrame modctrl_cmd    = { 2, 0x10, 0x01 };
  isegFrame modesta_cmd    = { 2, 0x10, 0x02 };
  isegFrame modemsk_cmd    = { 2, 0x10, 0x03 };
  isegFrame modechsta_cmd  = { 2, 0x10, 0x04 };
  isegFrame modechmsk_cmd  = { 2, 0x10, 0x05 };
  isegFrame modegrpsta_cmd = { 2, 0x10, 0x05 }; // TODO ? ask iseg
  isegFrame modegrpmsk_cmd = { 2, 0x10, 0x06 }; // TODO ? ask iseg
  isegFrame vramp_cmd      = { 2, 0x11, 0x00 };
  isegFrame iramp_cmd      = { 2, 0x11, 0x01 };
  isegFrame vmax_cmd       = { 2, 0x11, 0x02 };
  isegFrame imax_cmd       = { 2, 0x11, 0x03 };
  isegFrame supply24_cmd   = { 2, 0x11, 0x04 };
  isegFrame supply5_cmd    = { 2, 0x11, 0x05 };
  isegFrame temp_cmd       = { 2, 0x11, 0x06 };
  isegFrame chanctrl_w_cmd = { 3, 0x40, 0x01 };
  isegFrame vset_w_cmd     = { 3, 0x41, 0x00 };
  isegFrame iset_w_cmd     = { 3, 0x41, 0x01 };
  isegFrame vbounds_w_cmd  = { 3, 0x41, 0x04 };
  isegFrame ibounds_w_cmd  = { 3, 0x41, 0x05 };
  _cmds.insert( std::make_pair( P_R_ChanStatus,       chstat_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanCtrl,         chanctrl_r_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanEventStatus,  chesta_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanEventMask,    chemsk_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanVset,         vset_r_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanIset,         iset_r_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanVmom,         vmom_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanImom,         imom_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanImomRange,    imom_range_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanVbounds,      vbounds_r_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanIbounds,      ibounds_r_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanVnom,         vnom_cmd ) );
  _cmds.insert( std::make_pair( P_R_ChanInom,         inom_cmd ) );
  _cmds.insert( std::make_pair( P_ModStatus,          modstat_cmd ) );
  _cmds.insert( std::make_pair( P_ModCtrl,            modctrl_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventStatus,     modesta_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventMask,       modemsk_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventChanStatus, modechsta_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventChanMask,   modechmsk_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventGrpStatus,  modegrpsta_cmd ) );
  _cmds.insert( std::make_pair( P_ModEventGrpMask,    modegrpmsk_cmd ) );
  _cmds.insert( std::make_pair( P_VRampSpeed,         vramp_cmd ) );
  _cmds.insert( std::make_pair( P_IRampSpeed,         iramp_cmd ) );
  _cmds.insert( std::make_pair( P_Vmax,               vmax_cmd ) );
  _cmds.insert( std::make_pair( P_Imax,               imax_cmd ) );
  _cmds.insert( std::make_pair( P_Supply24,           supply24_cmd ) );
  _cmds.insert( std::make_pair( P_Supply5,            supply5_cmd ) );
  _cmds.insert( std::make_pair( P_Temperature,        temp_cmd ) );
  _cmds.insert( std::make_pair( P_ChanCtrl,           chanctrl_w_cmd ) );
  _cmds.insert( std::make_pair( P_ChanVset,           vset_w_cmd ) );
  _cmds.insert( std::make_pair( P_ChanIset,           iset_w_cmd ) );
  _cmds.insert( std::make_pair( P_ChanVbounds,        vbounds_w_cmd ) );
  _cmds.insert( std::make_pair( P_ChanIbounds,        ibounds_w_cmd ) );

}

