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

#include "drvAsynIsegHv.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//! used to convert 32-bit float (IEEE 754) to 4*8 bit unsigned int
typedef union{
  epicsFloat32 fval;
  //  epicsUInt32  ival;
  epicsUInt8   val[4];
} can_float_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynIsegHVDriver";

//_____ F U N C T I O N S ______________________________________________________

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
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  can_frame_t pframe;
  if ( function == P_ClearEvtStatus ) {
    pframe.can_id  = can_id_;
    pframe.can_dlc = 4;
    pframe.data[0] = 0x10;
    pframe.data[1] = 0x01;
    pframe.data[2] = 0x18;
    pframe.data[3] = 0x40;
  } else if ( function == P_EmergencyOff ) {
    pframe.can_id  = can_id_;
    pframe.can_dlc = 6;
    pframe.data[0] = 0x22;
    pframe.data[1] = 0x01;
    pframe.data[2] = 0x00;
    pframe.data[3] = 0x00;
    pframe.data[4] = 0xff;
    pframe.data[5] = 0xff;
  }
  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pAsynUserGenericPointer_->errorMessage );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!
//!          If pasynUser->reason is equal
//!          - P_Chan_status        the status register of a channel is read out.
//!                                 The addr parameter is used as channel number.
//!          - P_Chan_Event_status  the event status register of a channel is read out.
//!                                 The addr parameter is used as channel number.
//!          - P_Mod_status         the status register of the module is read out.
//!          - P_Mod_Event_status   the event status register of the module is read out.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------_-----------------------------------------------
asynStatus drvAsynIsegHv::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char *functionName = "readUInt32Digital";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  if ( function == P_SwitchOnOff ) {
    status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
    return status;
  }
  if ( ( function == P_Mod_status || function == P_Mod_Event_status ) && ( addr != 0 ) ) {
    status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
    return status;
  }

  std::map<int, isegFrame>::const_iterator it = cmdsUIn32D_.find( function );
  if( it == cmdsUIn32D_.end() ) return asynError;

  can_frame_t pframe;
  pframe.can_id = can_id_ | 1;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( addr & 0xff );

  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, pasynUser->timeout );
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, deviceName_, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pAsynUserGenericPointer_->errorMessage );
    return asynError;
  }
  if ( ( pframe.can_id  != can_id_ ) ||
       ( pframe.can_dlc != ( it->second.dlc + 2 ) ) ||
       ( pframe.data[0] != it->second.data0 ) ||
       ( pframe.data[1] != it->second.data1 ) 
       ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x where %08x %d %02x %02x was expected", 
                   driverName, deviceName_, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1],
                   can_id_, it->second.dlc + 4, it->second.data0, it->second.data1 );
    return asynError;
  }

  // if ( it->second.dlc == 3 &&
  //      pframe.data[2] != (epicsUInt8)( addr & 0xff )
  //      ) {
  //   epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
  //                  "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x where %08x %d %02x %02x %02x was expected\033[0m", 
  //                  driverName, deviceName_, functionName, function,
  //                  pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2],
  //                  can_id_, it->second.dlc + 4, it->second.data0, it->second.data1, ( addr & 0xff ) );
  //   return asynError;
  // }  

  // update the parameter with the received value
  epicsUInt32 myValue  = ( pframe.data[it->second.dlc] << 8 ) | ( pframe.data[it->second.dlc + 1]);
  status = setUIntDigitalParam( addr, function, myValue, mask );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, myValue, mask );
  
  // read back parameter
  status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%u, mask=%u\n", 
               driverName, deviceName_, functionName, function, *value, mask );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->write().
//!
//!          If pasynUser->reason is equal to P_SwitchOnOff this function
//!          switches all channels on / off
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
  const char* functionName = "writeUInt32Digital";

  if ( function != P_SwitchOnOff ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( 0 != addr ) return asynSuccess;
  
  status = (asynStatus) setUIntDigitalParam( addr, function, value, mask );
  status = (asynStatus) callParamCallbacks( addr, addr );  
  if ( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, functionName, function, value, mask );

  can_frame_t pframe;
  epicsUInt8 dummy = ( value ? 0xff : 0x00 );
  /*
    // use mask to select channels
    epicsUInt8 buffer[2] = {0,0};
    if ( value ) {
    epicsUInt8 dummy = ( ( mask & 0xff00 ) >> 8 );
    buffer[0] = 0xff & dummy;
    dummy = mask & 0xff;
    buffer[1] = 0xff & dummy;
  */
  pframe.can_id  = can_id_;
  pframe.can_dlc = 6;
  pframe.data[0] = 0x22;
  pframe.data[1] = 0x00;
  pframe.data[2] = 0x00;
  pframe.data[3] = 0x00;
  pframe.data[4] = dummy; // buffer[0];
  pframe.data[5] = dummy; // buffer[1];

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pAsynUserGenericPointer_->errorMessage );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynFloat64->read().
//!
//!          If pasynUser->reason is equal
//!          - P_Chan_Vmom   the measured voltage of a channel is read out.
//!                          The addr parameter is used as channel number.
//!          - P_Chan_Imom   the measured current of a channel is read out.
//!                          The addr parameter is used as channel number.
//!          - P_Chan_Vset   the set voltage of a channel is read out.
//!                          The addr parameter is used as channel number.
//!          - P_Chan_Iset   the current limit of a channel is read out.
//!                          The addr parameter is used as channel number.
//!          - P_Temperature the board temperature is read out.
//!          - P_RampSpeed   the current ramp speed is read out.
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
  const char *functionName = "readFloat64";
  
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( ( function == P_Temperature || function == P_RampSpeed ) && ( addr != 0 ) ) {
    status = (asynStatus) getDoubleParam( addr, function, value );
    return status;
  }

  std::map<int, isegFrame>::const_iterator it = cmdsFloat64_.find( function );
  if( it == cmdsFloat64_.end() ) return asynError;

  can_frame_t pframe;
  pframe.can_id = can_id_ | 1;
  pframe.can_dlc = it->second.dlc;
  pframe.data[0] = it->second.data0;
  pframe.data[1] = it->second.data1;
  pframe.data[2] = (epicsUInt8)( addr & 0xff );

  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, pasynUser->timeout );
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, deviceName_, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pAsynUserGenericPointer_->errorMessage );
    return asynError;
  }
  if ( ( pframe.can_id  != can_id_ ) ||
       ( pframe.can_dlc != ( it->second.dlc + 4 ) ) ||
       ( pframe.data[0] != it->second.data0 ) ||
       ( pframe.data[1] != it->second.data1 ) 
       ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x where %08x %d %02x %02x was expected", 
                   driverName, deviceName_, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1],
                   can_id_, it->second.dlc + 4, it->second.data0, it->second.data1 );
    return asynError;
  }
  
  can_float_t myValue;
  //  myValue.ival = *( epicsUInt32*)&preadframe->data[ it->second.dlc ];
  myValue.val[3] = pframe.data[ it->second.dlc ];
  myValue.val[2] = pframe.data[ it->second.dlc + 1 ];
  myValue.val[1] = pframe.data[ it->second.dlc + 2 ];
  myValue.val[0] = pframe.data[ it->second.dlc + 3 ];

  // convert current from A to uA/nA
  if ( function == P_Chan_Imom || function == P_Chan_Iset ) myValue.fval *= conversion_;

  // update the parameter with the received value
  status = setDoubleParam( addr, function, myValue.fval );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%f\033[0m", 
                   driverName, deviceName_, functionName, status, function, myValue.fval );

  // read back parameter
  status = (asynStatus) getDoubleParam( addr, function, value );
  if ( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%f", 
                   driverName, deviceName_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%f\n", 
               driverName, deviceName_, functionName, function, *value );
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
  int addr = 0;
  const char *functionName = "writeFloat64";

  if ( function == P_Chan_Vmom || function == P_Chan_Imom || function == P_Temperature )
    return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( function == P_RampSpeed && addr != 0 ) return asynSuccess;

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

  //  *(epicsUInt32*)&pframe.data[it->second.dlc] = myValue.ival;
  pframe.data[ it->second.dlc ]     = myValue.val[3];
  pframe.data[ it->second.dlc + 1 ] = myValue.val[2];
  pframe.data[ it->second.dlc + 2 ] = myValue.val[1];
  pframe.data[ it->second.dlc + 3 ] = myValue.val[0];

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, deviceName_, functionName, status, function,
                   pAsynUserGenericPointer_->errorMessage );
    return asynError;
  }
  
  /* Set the parameter and readback in the parameter library. */
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
//! @brief   Called when asyn clients call pasynOption->read()
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//! @param   [in]  key             Name of option
//! @param   [out] value           String containing the value for the option
//! @param   [in]  maxChars        Size of value string
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError is returned. An error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::readOption( asynUser *pasynUser, const char *key,
                                      char *value, int maxChars ) {
  const char* functionName = "readOption";

  if( epicsStrCaseCmp( key, "use_na" ) == 0 ) {
    if ( 1.e6 == conversion_ )  strcpy(value, "0" );
    else                        strcpy(value, "1" );
  } else {
    // unknown option
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: Invalid option key '%s'",
                   driverName, functionName, key );
    return asynError;
  }
    
  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynOption->write()
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//! @param   [in]  key             Name of option
//! @param   [in]  value           String containing the value for the option
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeOption( asynUser *pasynUser, const char *key, const char *value ) {
  const char* functionName = "writeOption";

  if( epicsStrCaseCmp( key, "use_na" ) == 0 ) {
    int dummy = 0;
    if( sscanf( value, "%u", &dummy ) != 1 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "Bad number");
      return asynError;
    }
    if ( dummy )  conversion_ = 1.e9;
    else          conversion_ = 1.e6;
  } else {
    // unknown option
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: Invalid option key '%s'",
                   driverName, functionName, key );
    return asynError;
  }
  
  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynIsegHv class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  crate_id    The id of the crate
//! @param   [in]  module_id   The id of the module inside the crate
//------------------------------------------------------------------------------
drvAsynIsegHv::drvAsynIsegHv( const char *portName, const char *CanPort,
                              const int crate_id, const int module_id ) 
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
  can_id_      = ( 1 << 9 ) | ( crate_id << 6 ) | ( module_id << 3 );

  // Create parameters
  createParam( P_ISEGHV_CHANSTATUS_STRING,    asynParamUInt32Digital, &P_Chan_status );
  createParam( P_ISEGHV_CHANEVTSTATUS_STRING, asynParamUInt32Digital, &P_Chan_Event_status );
  createParam( P_ISEGHV_VMOM_STRING,          asynParamFloat64,       &P_Chan_Vmom );
  createParam( P_ISEGHV_IMOM_STRING,          asynParamFloat64,       &P_Chan_Imom );
  createParam( P_ISEGHV_VSET_STRING,          asynParamFloat64,       &P_Chan_Vset );
  createParam( P_ISEGHV_ISET_STRING,          asynParamFloat64,       &P_Chan_Iset );
  // These parameters are only used once per module:
  createParam( P_ISEGHV_MODSTATUS_STRING,      asynParamUInt32Digital, &P_Mod_status );
  createParam( P_ISEGHV_MODEVTSTATUS_STRING,   asynParamUInt32Digital, &P_Mod_Event_status );
  createParam( P_ISEGHV_TEMPERATURE_STRING,    asynParamFloat64,       &P_Temperature );
  createParam( P_ISEGHV_RAMPSPEED_STRING,      asynParamFloat64,       &P_RampSpeed );
  createParam( P_ISEGHV_CLEAREVTSTATUS_STRING, asynParamInt32,         &P_ClearEvtStatus );
  createParam( P_ISEGHV_EMO_STRING,            asynParamInt32,         &P_EmergencyOff );
  createParam( P_ISEGHV_SWITCH_STRING,         asynParamUInt32Digital, &P_SwitchOnOff );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
             driverName, deviceName_, functionName, CanPort );
    return;
  }

  // used to convert the measured current from A to u/nA
  conversion_ = 1.e6;
  
  can_frame_t pframe;
  // Send "status connected" message
  pframe.can_id  = can_id_;
  pframe.can_dlc = 2;
  pframe.data[0] = 0xd8;
  pframe.data[1] = 0x01;
  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, 1. );

  // Get inital value for Switch-Parameter
  // It seems not all modules understand this command...
  pframe.can_id  = can_id_ | 1;
  pframe.can_dlc = 2;
  pframe.data[0] = 0x22;
  pframe.data[1] = 0x00;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, .5 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s: No reply from device within 0.5 s! \033[0m \n",
             driverName, deviceName_, functionName, P_ISEGHV_SWITCH_STRING );
  } else {
    if ( pframe.data[4] & 0xff || pframe.data[5] & 0xff)
      setUIntDigitalParam( 0, P_SwitchOnOff, 1, 1 );
    else
      setUIntDigitalParam( 0, P_SwitchOnOff, 0, 1 );
  }
  
  // get initial values for vset/iset parameters
  for ( unsigned int i = 0; i < 16; i++ ) {
    pframe.can_id  = can_id_ | 1;
    pframe.can_dlc = 3;
    pframe.data[0] = 0x41;
    pframe.data[1] = 0x00;
    pframe.data[2] = i;
    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, 1. );
    if ( status != asynSuccess ) {
      fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s %d: No reply from device within 1 s! \033[0m \n",
               driverName, deviceName_, functionName, P_ISEGHV_VSET_STRING, i );
      return;
    }
    can_float_t myValue;
    //    myValue.ival = *( epicsUInt32*)&pframe.data[4];
    myValue.val[3] = pframe.data[4];
    myValue.val[2] = pframe.data[5];
    myValue.val[1] = pframe.data[6];
    myValue.val[0] = pframe.data[7];
    setDoubleParam( i, P_Chan_Vset, myValue.fval );

    pframe.can_id  = can_id_ | 1;
    pframe.can_dlc = 3;
    pframe.data[0] = 0x41;
    pframe.data[1] = 0x01;
    pframe.data[2] = i;
    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, 1. );
    if ( status != asynSuccess ) {
      fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s %d: No reply from device within 1 s! \033[0m \n",
               driverName, deviceName_, functionName, P_ISEGHV_ISET_STRING, i );
      return;
    }
    //    myValue.ival = *( epicsUInt32*)&pframe.data[4];
    myValue.val[3] = pframe.data[4];
    myValue.val[2] = pframe.data[5];
    myValue.val[1] = pframe.data[6];
    myValue.val[0] = pframe.data[7];
    myValue.fval *= 1.e6;
    setDoubleParam( i, P_Chan_Iset, myValue.fval );
  }

  isegFrame vset_cmd = { 3, 0x41, 0x00 };
  isegFrame iset_cmd = { 3, 0x41, 0x01 };
  isegFrame vmom_cmd = { 3, 0x41, 0x02 };
  isegFrame imom_cmd = { 3, 0x41, 0x03 };
  isegFrame temp_cmd = { 2, 0x11, 0x06 };
  isegFrame ramp_cmd = { 2, 0x11, 0x00 };
  cmdsFloat64_.insert( std::make_pair( P_Chan_Vmom, vmom_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Chan_Imom, imom_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Chan_Vset, vset_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Chan_Iset, iset_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_Temperature, temp_cmd ) );
  cmdsFloat64_.insert( std::make_pair( P_RampSpeed, ramp_cmd ) );

  isegFrame chstat_cmd = { 3, 0x40, 0x00 };
  isegFrame chesta_cmd = { 3, 0x40, 0x02 };
  isegFrame modstat_cmd = { 2, 0x10, 0x00 };
  isegFrame modesta_cmd = { 2, 0x10, 0x02 };
  cmdsUIn32D_.insert( std::make_pair( P_Chan_status, chstat_cmd ) );
  cmdsUIn32D_.insert( std::make_pair( P_Chan_Event_status, chesta_cmd ) );
  cmdsUIn32D_.insert( std::make_pair( P_Mod_status, modstat_cmd ) );
  cmdsUIn32D_.insert( std::make_pair( P_Mod_Event_status, modesta_cmd ) );
}

//******************************************************************************
//! EOF
//******************************************************************************
