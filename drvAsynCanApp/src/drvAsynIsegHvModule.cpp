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
#include "asynGenericPointerSyncIO.h"
#include "asynStandardInterfaces.h"

// local includes
#include "drvAsynIsegHvModule.h"
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
asynStatus drvAsynIsegHvModule::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  if ( function == P_Mod_status || function == P_Mod_Event_status ) return asynSuccess;

  std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
  if( it == cmds_.end() ) return asynError;

  status = setUIntDigitalParam( function, value, mask );

  can_frame_t pframe;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  if ( function == P_Mod_Ctrl ) {
    epicsUInt32 myVal;
    status = getUIntDigitalParam( function, &myVal, mask );
    pframe.can_id  = can_id_;
    pframe.data[2] = (epicsUInt8)( myVal & 0x00005800 );
    pframe.data[3] = (epicsUInt8)( myVal & 0x00000060 );    
  } else {
    pframe.can_id  = can_id_ | 1;
    pframe.data[2] = (epicsUInt8)( chanMsk_ & 0x0000ff00 );
    pframe.data[3] = (epicsUInt8)( chanMsk_ & 0x000000ff );
    pframe.data[4] = 0;
  }

  status = pasynGenericPointerSyncIO->write( pasynUser_, &pframe, pasynUser->timeout );

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
asynStatus drvAsynIsegHvModule::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "readUInt32Digital";
    
  if ( function == P_Mod_status || function == P_Mod_Ctrl || function == P_Mod_Event_status ) {
    std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
    if( it == cmds_.end() ) return asynError;
    
    can_frame_t pframe;
    pframe.can_id = can_id_ | 1;
    pframe.can_dlc = it->second.dlc;
    pframe.data[0] = it->second.data0;
    pframe.data[1] = it->second.data1;

    status = pasynGenericPointerSyncIO->writeRead( pasynUser_, &pframe, &pframe, pasynUser->timeout );
    
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, deviceName_, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }
    status = setUIntDigitalParam( function, ( pframe.data[2] << 8 ) | pframe.data[3], mask );
  }
  
  status = (asynStatus) getUIntDigitalParam( function, value, mask);
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
asynStatus drvAsynIsegHvModule::writeFloat64( asynUser *pasynUser, epicsFloat64 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "writeFloat64";

  if ( function != P_VRampSpeed && function != P_IRampSpeed ) return asynSuccess;

  std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
  if( it == cmds_.end() ) return asynError;

  can_float_t myValue;
  myValue.fval = value;
  
  can_frame_t pframe;
  pframe.can_id = can_id_;
  pframe.can_dlc = it->second.dlc + 4;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = myValue.val[3];
  pframe.data[3] = myValue.val[2];
  pframe.data[4] = myValue.val[1];
  pframe.data[5] = myValue.val[0];

  status = pasynGenericPointerSyncIO->write( pasynUser_, &pframe, pasynUser->timeout );
  
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pasynUser_->errorMessage );
    return asynError;
  }
  
  // Set the parameter and readback in the parameter library.
  status = setDoubleParam( function, value );
  callParamCallbacks();
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
asynStatus drvAsynIsegHvModule::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "readFloat64";
  
  if ( function == P_VRampSpeed || \
       function == P_IRampSpeed || \
       function == P_Supply24   || \
       function == P_Supply5    || \
       function == P_Temperature ) {
    std::map<int, isegFrame>::const_iterator it = cmds_.find( function );
    if( it == cmds_.end() ) return asynError;
    can_frame_t pframe;
    pframe.can_id = can_id_ | 1;
    pframe.can_dlc = it->second.dlc;
    pframe.data[0] = it->second.data0;
    pframe.data[1] = it->second.data1;
    fprintf(stderr,"%s:%s:%s function=%d step3\n",driverName, deviceName_, functionName, function);

    status = pasynGenericPointerSyncIO->writeRead( pasynUser_, &pframe, &pframe, pasynUser->timeout );
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, deviceName_, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }
    fprintf(stderr,"%s:%s:%s function=%d step4\n",driverName, deviceName_, functionName, function);
    can_float_t myValue;
    myValue.val[3] = pframe.data[2];
    myValue.val[2] = pframe.data[3];
    myValue.val[1] = pframe.data[4];
    myValue.val[0] = pframe.data[5];

    status = setDoubleParam( function, myValue.fval );
  }

  status = (asynStatus) getDoubleParam( function, value );
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
//! @brief   Constructor for the drvAsynIsegHvModule class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  module_id   The id of the module
//------------------------------------------------------------------------------
drvAsynIsegHvModule::drvAsynIsegHvModule( const char *portName,
                                          const char *CanPort,
                                          const int module_id,
                                          const int channels ) 
  : asynPortDriver( portName, 
                    1, // maxAddr  
                    NUM_ISEGHV_MOD_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask, // Interrupt mask
                    ASYN_CANBLOCK, // asynFlags.
                    1,   // Autoconnect
                    0,   // Default priority
                    0 )  // Default stack size
{
  const char *functionName = "drvAsynIsegHv";
  asynStatus status = asynSuccess;
  
  deviceName_  = epicsStrDup( portName );
  can_id_      = ( 1 << 9 ) | ( module_id << 3 );
  for ( int i = 0; i < channels || i < 16; i++ ) chanMsk_ |= ( 1 << i );
  
  // Create parameters
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

  // Connect to asyn generic pointer port with asynGenericPointer interface
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pasynUser_, 0 );
  if ( status != asynSuccess ) {
    printf( "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
            driverName, deviceName_, functionName, CanPort );
  }

  isegFrame chstat_cmd   = { 5, 0x60, 0x00 };
  isegFrame chanctrl_cmd = { 5, 0x60, 0x01 };
  isegFrame chesta_cmd   = { 5, 0x60, 0x02 };
  isegFrame vset_cmd     = { 5, 0x61, 0x00 };
  isegFrame iset_cmd     = { 5, 0x61, 0x01 };
  isegFrame vmom_cmd     = { 5, 0x61, 0x02 };
  isegFrame imom_cmd     = { 5, 0x61, 0x03 };
  isegFrame vbounds_cmd  = { 5, 0x61, 0x04 };
  isegFrame ibounds_cmd  = { 5, 0x61, 0x05 };
  isegFrame vnom_cmd     = { 5, 0x61, 0x06 };
  isegFrame inom_cmd     = { 5, 0x61, 0x07 };
  isegFrame modstat_cmd  = { 2, 0x10, 0x00 };
  isegFrame modctrl_cmd  = { 4, 0x10, 0x01 };
  isegFrame modesta_cmd  = { 2, 0x10, 0x02 };
  isegFrame vramp_cmd    = { 2, 0x11, 0x00 };
  isegFrame iramp_cmd    = { 2, 0x11, 0x01 };
  isegFrame supply24_cmd = { 2, 0x11, 0x04 };
  isegFrame supply5_cmd  = { 2, 0x11, 0x05 };
  isegFrame temp_cmd     = { 2, 0x11, 0x06 };
  cmds_.insert( std::make_pair( P_R_Chan_status,       chstat_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Ctrl,         chanctrl_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Event_status, chesta_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Vset,         vset_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Iset,         iset_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Vmom,         vmom_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Imom,         imom_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Vbounds,      vbounds_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Ibounds,      ibounds_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Vnom,         vnom_cmd ) );
  cmds_.insert( std::make_pair( P_R_Chan_Inom,         inom_cmd ) );
  cmds_.insert( std::make_pair( P_Mod_status,          modstat_cmd ) );
  cmds_.insert( std::make_pair( P_Mod_Ctrl,            modctrl_cmd ) );
  cmds_.insert( std::make_pair( P_Mod_Event_status,    modesta_cmd ) );
  cmds_.insert( std::make_pair( P_VRampSpeed,          vramp_cmd ) );
  cmds_.insert( std::make_pair( P_IRampSpeed,          iramp_cmd ) );
  cmds_.insert( std::make_pair( P_Supply24,            supply24_cmd ) );
  cmds_.insert( std::make_pair( P_Supply5,             supply5_cmd ) );
  cmds_.insert( std::make_pair( P_Temperature,         temp_cmd ) );

}

//******************************************************************************
//! EOF
//******************************************************************************
