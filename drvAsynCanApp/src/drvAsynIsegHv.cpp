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
// brief   Asyn driver for ISEG EHS/EDS high voltage modules using the RPi Can interface
//
// version 1.0.0; Nov. 27, 2012
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
#include "drvAsynIsegHv.h"
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
void myInterruptCallbackGenericPointer( void *userPvt,
                                        asynUser *pasynUser,
                                        void *pointer ) {
  drvAsynIsegHv* interface = static_cast<drvAsynIsegHv*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

void drvAsynIsegHv::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  can_frame_t* pframe = (can_frame_t *)pointer;
  if ( pframe->can_id  != can_id_ ) return;
  
  int opcode = ( pframe->data[0] << 8 ) | pframe->data[1];
  int addr = 0;
  can_float_t myValue;

  switch( opcode ){

    // Channel Status
  case 0x4000:
    addr = pframe->data[2];
    status = setUIntDigitalParam( addr, P_Chan_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Control
  case 0x4001:
    addr = pframe->data[2];
    status = setUIntDigitalParam( addr, P_Chan_Event_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Event Status
  case 0x4002: 
    addr = pframe->data[2];
    status = setUIntDigitalParam( addr, P_Chan_Event_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Set voltage
  case 0x4100: 
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vset, myValue.fval );
    break;

    // Set current/trip
  case 0x4101: 
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Iset, myValue.fval );
    break;

    // Voltage measurement
  case 0x4102: 
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vmom, myValue.fval );
    break;

    // Current measurement
  case 0x4103:
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Imom, myValue.fval );
    break;

    // Voltage bounds
  case 0x4104: 
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vbounds, myValue.fval );
    break;

    // Current bounds
  case 0x4105:
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Ibounds, myValue.fval );
    break;

    // Nominal Voltage
  case 0x4106: 
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vnom, myValue.fval );
    break;

    // Nominal Current
  case 0x4107:
    addr = pframe->data[2];
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Inom, myValue.fval );
    break;

  default:
    status = asynError;
    // do nothing
  }
  
  if ( asynSuccess != status ) {
    std::cerr << driverName << ":" <<  deviceName_ << ":asynReadHandler: Error"
              << std::endl;
    return;
  }
  callParamCallbacks( addr, addr );
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          If pasynUser->reason is equal
//!          - P_ClearEvtStatus  the event status register of the module and all channels is cleared
//!          - P_EmergencyOff    all channels of the module are switched off w/o ramp.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  can_frame_t pframe;
  pframe.can_id  = can_id_;
  pframe.can_dlc = 4;
  pframe.data[0] = 0x10;
  pframe.data[1] = 0x01;
  pframe.data[2] = 0x18;
  pframe.data[3] = 0x40;

  pasynManager->lockPort( pasynUser_ );
  status = pasynGenericPointer_->write( pvtGenericPointer_, pasynUser_, &pframe );
  pasynManager->unlockPort( pasynUser_ );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pasynUser_->errorMessage );
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
asynStatus drvAsynIsegHv::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  if ( function == P_Mod_status || function == P_Mod_Event_status ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = cmdsUInt32D_.find( function );
  if( it == cmdsUInt32D_.end() ) return asynError;

  can_frame_t pframe;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  if ( function == P_Chan_Ctrl ) {
    pframe.can_id  = can_id_;
    pframe.data[2] = (epicsUInt8)addr;
    pframe.data[3] = 0;
    pframe.data[4] = (epicsUInt8)( value & 0x00000024 );
  } else if ( function == P_Mod_Ctrl ) {
    pframe.can_id  = can_id_;
    pframe.data[2] = (epicsUInt8)( value & 0x00005800 );
    pframe.data[3] = (epicsUInt8)( value & 0x00000060 );    
  } else {
    pframe.can_id  = can_id_ | 1;
    pframe.data[2] = (epicsUInt8)( mask & 0x0000ff00 );
    pframe.data[3] = (epicsUInt8)( mask & 0x000000ff );
    pframe.data[4] = 0;
  }

  pasynUser_->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( pasynUser_ );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, deviceName_, functionName, status, function );
    return status;
  }
  status = pasynGenericPointer_->write( pvtGenericPointer_, pasynUser_, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( pasynUser_ );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, deviceName_, functionName, status, function );
    return unlockStatus;
  }
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pasynUser_->errorMessage );
    return asynError;
  }
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
asynStatus drvAsynIsegHv::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readUInt32Digital";
    
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( function == P_Mod_status || function == P_Mod_Ctrl || function == P_Mod_Event_status ) {
    std::map<int, isegFrame>::const_iterator it = cmdsUInt32D_.find( function );
    if( it == cmdsUInt32D_.end() ) return asynError;
    
    can_frame_t pframe;
    pframe.can_id = can_id_ | 1;
    pframe.can_dlc = it->second.dlc;
    pframe.data[0] = it->second.data0;
    pframe.data[1] = it->second.data1;

    pasynUser_->timeout = pasynUser->timeout;
    status = pasynManager->queueLockPort( pasynUser_ );
    if( asynSuccess != status) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return status;
    }
    status = pasynGenericPointer_->write( pvtGenericPointer_, pasynUser_, &pframe );
    if ( asynSuccess != status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return status;
    } 
    status = pasynGenericPointer_->read( pvtGenericPointer_, pasynUser_, &pframe );
    unlockStatus = pasynManager->queueUnlockPort( pasynUser_ );
    if( asynSuccess != unlockStatus ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return unlockStatus;
    }
    
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, deviceName_, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }
    status = setUIntDigitalParam( addr, function, ( pframe.data[2] << 8 ) | pframe.data[3], mask );
  }
  
  status = (asynStatus) getUIntDigitalParam(addr, function, value, mask);
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%u mask=%u", 
                   driverName, deviceName_, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%u, mask=%u\n", 
               driverName, deviceName_, functionName, function, *value, mask );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynFloat64->write().
//!
//!          If pasynUser->reason is equal
//!          - P_Chan_Vset   the set voltage of a channel is written.
//!                          The addr parameter is used as channel number.
//!          - P_Chan_Iset   the current limit of a channel is written.
//!                          The addr parameter is used as channel number.
//!          - P_RampSpeed   the current ramp speed is written.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeFloat64( asynUser *pasynUser, epicsFloat64 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  int addr = 0;
  const char *functionName = "writeFloat64";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = cmdsFloat64_.find( function );
  if( it == cmdsFloat64_.end() ) return asynError;
  
  can_frame_t pframe;
  pframe.can_id = can_id_;
  pframe.can_dlc = it->second.dlc + 4;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( addr & 0xff );
  
  can_float_t myValue;
  myValue.fval = value;

  // convert current from uA to A
  if ( function == P_Chan_Iset ) myValue.fval *= 1.e-6;

  pframe.data[ it->second.dlc ]     = myValue.val[3];
  pframe.data[ it->second.dlc + 1 ] = myValue.val[2];
  pframe.data[ it->second.dlc + 2 ] = myValue.val[1];
  pframe.data[ it->second.dlc + 3 ] = myValue.val[0];

  pasynUser_->timeout = pasynUser->timeout;
  status = pasynManager->queueLockPort( pasynUser_ );
  if( asynSuccess != status) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                   driverName, deviceName_, functionName, status, function );
    return status;
  }
  status = pasynGenericPointer_->write( pvtGenericPointer_, pasynUser_, &pframe );
  unlockStatus = pasynManager->queueUnlockPort( pasynUser_ );
  if( asynSuccess != unlockStatus ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                   driverName, deviceName_, functionName, status, function );
    return unlockStatus;
  }

  if ( asynSuccess != status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pasynUser_->errorMessage );
    return asynError;
  }
  
  // Set the parameter and readback in the parameter library.
  status = setDoubleParam( addr, function, value );
  callParamCallbacks( addr, addr );
  if ( status ) 
    asynPrint( pasynUser, ASYN_TRACE_ERROR, 
               "%s:%s:%s: error, status=%d function=%d, value=%f\n", 
               driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%f\n", 
               driverName, deviceName_, functionName, function, value );
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
asynStatus drvAsynIsegHv::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readFloat64";
  
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( function == P_VRampSpeed || \
       function == P_IRampSpeed || \
       function == P_Supply24   || \
       function == P_Supply5    || \
       function == P_Temperature ) {
    std::map<int, isegFrame>::const_iterator it = cmdsFloat64_.find( function );
    if( it == cmdsFloat64_.end() ) return asynError;
    can_frame_t pframe;
    pframe.can_id = can_id_ | 1;
    pframe.can_dlc = it->second.dlc;
    pframe.data[0] = it->second.data0;
    pframe.data[1] = it->second.data1;

    pasynUser_->timeout = pasynUser->timeout;
    status = pasynManager->queueLockPort( pasynUser_ );
    if( asynSuccess != status) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return status;
    }
    status = pasynGenericPointer_->write( pvtGenericPointer_, pasynUser_, &pframe );
    if ( asynSuccess != status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return status;
    } 
    status = pasynGenericPointer_->read( pvtGenericPointer_, pasynUser_, &pframe );
    unlockStatus = pasynManager->queueUnlockPort( pasynUser_ );
    if( asynSuccess != unlockStatus ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                     driverName, deviceName_, functionName, status, function );
      return unlockStatus;
    }
    
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, deviceName_, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
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
//! @brief   Constructor for the drvAsynIsegHv class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  module_id   The id of the module
//------------------------------------------------------------------------------
drvAsynIsegHv::drvAsynIsegHv( const char *portName,
                              const char *CanPort,
                              const int module_id ) 
  : asynPortDriver( portName, 
                    16, /* maxAddr */ 
                    NUM_ISEGHV_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynIsegHv";
  asynStatus status = asynSuccess;
  
  deviceName_  = epicsStrDup( portName );
  can_id_      = ( 1 << 9 ) | ( module_id << 3 );
  
  // Create parameters
  // Channel related parameters which hold the values
  createParam( P_ISEGHV_CHANSTATUS_STRING,      asynParamUInt32Digital, &P_Chan_status );
  createParam( P_ISEGHV_CHAN_CTRL_STRING,       asynParamUInt32Digital, &P_Chan_Ctrl );
  createParam( P_ISEGHV_CHANEVTSTATUS_STRING,   asynParamUInt32Digital, &P_Chan_Event_status );
  createParam( P_ISEGHV_VSET_STRING,            asynParamFloat64,       &P_Chan_Vset );
  createParam( P_ISEGHV_ISET_STRING,            asynParamFloat64,       &P_Chan_Iset );
  createParam( P_ISEGHV_VMOM_STRING,            asynParamFloat64,       &P_Chan_Vmom );
  createParam( P_ISEGHV_IMOM_STRING,            asynParamFloat64,       &P_Chan_Imom );
  createParam( P_ISEGHV_VBOUNDS_STRING,         asynParamFloat64,       &P_Chan_Vbounds );
  createParam( P_ISEGHV_IBOUNDS_STRING,         asynParamFloat64,       &P_Chan_Ibounds );
  createParam( P_ISEGHV_VNOM_STRING,            asynParamFloat64,       &P_Chan_Inom );
  createParam( P_ISEGHV_INOM_STRING,            asynParamFloat64,       &P_Chan_Inom );
  // Channel related parameters used as commands (no values)
  createParam( P_ISEGHV_R_CHANSTATUS_STRING,    asynParamUInt32Digital, &P_R_Chan_status );
  createParam( P_ISEGHV_R_CHAN_CTRL_STRING,     asynParamUInt32Digital, &P_R_Chan_Ctrl );
  createParam( P_ISEGHV_R_CHANEVTSTATUS_STRING, asynParamUInt32Digital, &P_R_Chan_Event_status );
  createParam( P_ISEGHV_R_VSET_STRING,          asynParamUInt32Digital, &P_R_Chan_Vset );
  createParam( P_ISEGHV_R_ISET_STRING,          asynParamUInt32Digital, &P_R_Chan_Iset );
  createParam( P_ISEGHV_R_VMOM_STRING,          asynParamUInt32Digital, &P_R_Chan_Vmom );
  createParam( P_ISEGHV_R_IMOM_STRING,          asynParamUInt32Digital, &P_R_Chan_Imom );
  createParam( P_ISEGHV_R_VBOUNDS_STRING,       asynParamUInt32Digital, &P_R_Chan_Vbounds );
  createParam( P_ISEGHV_R_IBOUNDS_STRING,       asynParamUInt32Digital, &P_R_Chan_Ibounds );
  createParam( P_ISEGHV_R_VNOM_STRING,          asynParamUInt32Digital, &P_R_Chan_Vnom );
  createParam( P_ISEGHV_R_INOM_STRING,          asynParamUInt32Digital, &P_R_Chan_Inom );
  // Module related parameters
  createParam( P_ISEGHV_MODSTATUS_STRING,       asynParamUInt32Digital, &P_Mod_status );
  createParam( P_ISEGHV_MODCTRL_STRING,         asynParamUInt32Digital, &P_Mod_Ctrl );
  createParam( P_ISEGHV_MODEVTSTATUS_STRING,    asynParamUInt32Digital, &P_Mod_Event_status );
  createParam( P_ISEGHV_VRAMPSPEED_STRING,      asynParamFloat64,       &P_VRampSpeed );
  createParam( P_ISEGHV_IRAMPSPEED_STRING,      asynParamFloat64,       &P_IRampSpeed );
  createParam( P_ISEGHV_SUPPLY24_STRING,        asynParamFloat64,       &P_Supply24 );
  createParam( P_ISEGHV_SUPPLY5_STRING,         asynParamFloat64,       &P_Supply5 );
  createParam( P_ISEGHV_TEMPERATURE_STRING,     asynParamFloat64,       &P_Temperature );
  createParam( P_ISEGHV_CLEAREVTSTATUS_STRING,  asynParamInt32,         &P_ClearEvtStatus );

  // Connect to asyn generic pointer port with asynGenericPointer interface
  pasynUser_ = pasynManager->createAsynUser( NULL, NULL );
  status = pasynManager->connectDevice( pasynUser_, CanPort, can_id_ );
  if ( asynSuccess != status ) {
    std::cerr << driverName << ":" <<  deviceName_ << ":" << functionName
              << ": Unable to connect Device"
              << std::endl;
    return;
  }
    
  asynInterface* pasynInterface;
    
  // find the asynCommon interface
  pasynInterface = pasynManager->findInterface( pasynUser_,
                                                asynCommonType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  deviceName_ << ":" << functionName
              << ": bus " << CanPort << " does not support asynCommon interface"
              << std::endl;
    return;
  }
  pasynCommon_ = static_cast<asynCommon*>( pasynInterface->pinterface );
  pvtCommon_   = pasynInterface->drvPvt;
  
  // find the asynGenericPointer interface
  pasynInterface = pasynManager->findInterface( pasynUser_,
                                                asynGenericPointerType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  deviceName_ << ":" << functionName
              << ": bus " << CanPort << " does not support asynGenericPointer interface"
              << std::endl;
    return;
  }
  pasynGenericPointer_ = static_cast<asynGenericPointer*>( pasynInterface->pinterface );
  pvtGenericPointer_   = pasynInterface->drvPvt;
  pasynUser_->reason = 1;
  status = pasynGenericPointer_->registerInterruptUser( pvtGenericPointer_,
                                                        pasynUser_,
                                                        myInterruptCallbackGenericPointer,
                                                        this,
                                                        &intrPvtGenericPointer_
                                                        );
  if( asynSuccess != status  ) {
    std::cerr << driverName << ":" <<  deviceName_ << ":" << functionName
              << ": failed to register interrupt"
              << std::endl;
    return;
  }
  // Start polling
  ReadPoller::create( CanPort );
  
  isegFrame vset_w_cmd   = { 3, 0x41, 0x00 };
  isegFrame iset_w_cmd   = { 3, 0x41, 0x01 };
  isegFrame vramp_cmd    = { 2, 0x11, 0x00 };
  isegFrame iramp_cmd    = { 2, 0x11, 0x01 };
  isegFrame supply24_cmd = { 2, 0x11, 0x04 };
  isegFrame supply5_cmd  = { 2, 0x11, 0x05 };
  isegFrame temp_cmd     = { 2, 0x11, 0x06 };
  cmdsFloat64_.insert( std::make_pair( P_Chan_Vset,   vset_w_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Chan_Iset,   iset_w_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_VRampSpeed,  vramp_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_IRampSpeed,  iramp_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Supply24,    supply24_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Supply5,     supply5_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Temperature, temp_cmd ) );

  isegFrame chstat_cmd      = { 5, 0x60, 0x00 };
  isegFrame chan_ctrl_s_cmd = { 5, 0x40, 0x01 };
  isegFrame chan_ctrl_r_cmd = { 5, 0x60, 0x01 };
  isegFrame chesta_cmd      = { 5, 0x60, 0x02 };
  isegFrame vset_r_cmd      = { 5, 0x61, 0x00 };
  isegFrame iset_r_cmd      = { 5, 0x61, 0x01 };
  isegFrame vmom_cmd        = { 5, 0x61, 0x02 };
  isegFrame imom_cmd        = { 5, 0x61, 0x03 };
  isegFrame vbounds_cmd     = { 5, 0x61, 0x04 };
  isegFrame ibounds_cmd     = { 5, 0x61, 0x05 };
  isegFrame vnom_cmd        = { 5, 0x61, 0x06 };
  isegFrame inom_cmd        = { 5, 0x61, 0x07 };
  isegFrame modstat_cmd     = { 2, 0x10, 0x00 };
  isegFrame modctrl_cmd     = { 4, 0x10, 0x01 };
  isegFrame modesta_cmd     = { 2, 0x10, 0x02 };
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_status,       chstat_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_Chan_Ctrl,           chan_ctrl_s_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Ctrl,         chan_ctrl_r_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Event_status, chesta_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Vset,         vset_r_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Iset,         iset_r_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Vmom,         vmom_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Imom,         imom_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Vbounds,      vbounds_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Ibounds,      ibounds_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Vnom,         vnom_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_R_Chan_Inom,         inom_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_Mod_status,          modstat_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_Mod_Ctrl,            modctrl_cmd ) );
  cmdsUInt32D_.insert( std::make_pair( P_Mod_Event_status,    modesta_cmd ) );
}

//******************************************************************************
//! EOF
//******************************************************************************
