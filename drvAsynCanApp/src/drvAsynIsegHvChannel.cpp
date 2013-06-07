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
#include "drvAsynIsegHvChannel.h"
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
  drvAsynIsegHvChannel* interface = static_cast<drvAsynIsegHvChannel*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

void drvAsynIsegHvChannel::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  can_frame_t* pframe = (can_frame_t *)pointer;
  if ( pframe->can_id != can_id_ ) return;
  
  int opcode = ( pframe->data[0] << 8 ) | pframe->data[1];
  int addr = pframe->data[2];
  can_float_t myValue;

  switch( opcode ){

    // Channel Status
  case 0x4000:
    status = setUIntDigitalParam( addr, P_Chan_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Control
  case 0x4001:
    status = setUIntDigitalParam( addr, P_Chan_Event_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Channel Event Status
  case 0x4002: 
    status = setUIntDigitalParam( addr, P_Chan_Event_status, ( pframe->data[3] << 8 ) | pframe->data[4], 0xffff );
    break;

    // Set voltage
  case 0x4100: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vset, myValue.fval );
    break;

    // Set current/trip
  case 0x4101: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Iset, myValue.fval );
    break;

    // Voltage measurement
  case 0x4102: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vmom, myValue.fval );
    break;

    // Current measurement
  case 0x4103:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Imom, myValue.fval );
    break;

    // Voltage bounds
  case 0x4104: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vbounds, myValue.fval );
    break;

    // Current bounds
  case 0x4105:
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    myValue.fval *= 1.e6;
    status = setDoubleParam( addr, P_Chan_Ibounds, myValue.fval );
    break;

    // Nominal Voltage
  case 0x4106: 
    myValue.val[3] = pframe->data[3];
    myValue.val[2] = pframe->data[4];
    myValue.val[1] = pframe->data[5];
    myValue.val[0] = pframe->data[6];
    status = setDoubleParam( addr, P_Chan_Vnom, myValue.fval );
    break;

    // Nominal Current
  case 0x4107:
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
asynStatus drvAsynIsegHvChannel::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
  if( it == cmds_.end() ) return asynError;

  can_frame_t pframe;
  pframe.can_id  = can_id_;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)addr;
  pframe.data[3] = 0;
  pframe.data[4] = (epicsUInt8)( value & 0x00000024 );

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
asynStatus drvAsynIsegHvChannel::writeFloat64( asynUser *pasynUser, epicsFloat64 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  int addr = 0;
  const char *functionName = "writeFloat64";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
  if( it == cmds_.end() ) return asynError;
  
  can_float_t myValue;
  myValue.fval = value;

    // convert current from uA to A
  if ( function == P_Chan_Iset || function == P_Chan_Ibounds ) myValue.fval /= 1.e-6;

  can_frame_t pframe;
  pframe.can_id  = can_id_;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( addr & 0xff );
  pframe.data[3] = myValue.val[3];
  pframe.data[4] = myValue.val[2];
  pframe.data[5] = myValue.val[1];
  pframe.data[6] = myValue.val[0];

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
//! @brief   Constructor for the drvAsynIsegHvChannel class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  module_id   The id of the module
//------------------------------------------------------------------------------
drvAsynIsegHvChannel::drvAsynIsegHvChannel( const char *portName,
                                            const char *CanPort,
                                            const int module_id,
                                            const int channels ) 
  : asynPortDriver( portName, 
                    channels, // maxAddr
                    NUM_ISEGHV_CHAN_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask, // Interrupt mask
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
                    1, // Autoconnect
                    0, // Default priority
                    0 ), // Default stack size
  channels_(channels)
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

  // Connect to asyn generic pointer port with asynGenericPointer interface
  pasynUser_ = pasynManager->createAsynUser( NULL, NULL );
  status = pasynManager->connectDevice( pasynUser_, CanPort, 0 );
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
  
  isegFrame chanctrl_cmd = { 5, 0x40, 0x01 };
  isegFrame vset_cmd     = { 7, 0x41, 0x00 };
  isegFrame iset_cmd     = { 7, 0x41, 0x01 };
  isegFrame vbounds_cmd  = { 7, 0x41, 0x04 };
  isegFrame ibounds_cmd  = { 7, 0x41, 0x05 };
  cmds_.insert( std::make_pair( P_Chan_Ctrl,    chanctrl_cmd ) );
  cmds_.insert( std::make_pair( P_Chan_Vset,    vset_cmd ) );
  cmds_.insert( std::make_pair( P_Chan_Iset,    iset_cmd ) );
  cmds_.insert( std::make_pair( P_Chan_Vbounds, vbounds_cmd ) );
  cmds_.insert( std::make_pair( P_Chan_Ibounds, ibounds_cmd ) );

}

//******************************************************************************
//! EOF
//******************************************************************************
