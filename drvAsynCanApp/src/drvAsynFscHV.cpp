//******************************************************************************
// Copyright (C) 2015 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Johannes-Gutenberg Universitaet Mainz
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
// brief   Asyn driver for Wiener VME crate remote control
//
// version 3.0.0; Jul. 29, 2014
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

#include "drvAsynFscHV.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynPandaFscHVDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->read().
//!
//! This function is used to read-out the temperature of the PANDA FSC HV supply.
//!          
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynFscHV::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "readInt32";

  if( function == _temp ) { // Parameter "TEMPERATURE" should be read
    // Create the CAN frame to send command to HV
    can_frame_t frame;
    frame.can_id  = 0x480 | _createId; // 0x480 = 1001 000 0000
    frame.can_dlc = 1;
    frame.data[0] = 0x05; 

    // Now send the command and try to read a response
    status = pasynGenericPointerSyncIO->writeRead( _pcanif, &frame, &frame, pasynUser->timeout );

    // Check if above function returned with a timeout-error
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, _deviceName, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }

    // Check if above function returned with any other unnormal status
    if ( status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d %s", 
                     driverName, _deviceName, functionName, status, function,
                     _pcanif->errorMessage );
      return asynError;
    }

    // Check if the reply matches our expectation
    if ( frame.can_id != ( 0x400 | _crateId ) || frame.can_dlc != 2 || frame.data[0] != 0x05 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d %02x...' where '%x08 2 0x05...' was expected.\033[0m", 
                     driverName, _deviceName, functionName, function, frame.can_id, frame.can_dlc, frame.data[0], (0x400 | _createId) );
      return asynError;
    }
   
    // Update the parameter in our local buffer
    status = setIntegerParam( _temp, pframe.data[1] );
    // Call parameter callbacks (if someone registered to monitor changes)
    status = (asynStatus) callParamCallbacks();
  }

  // read back parameter from local buffer
  status = (asynStatus) getIntegerParam( function, value );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u\033[0m", 
                   driverName, _deviceName, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%u\n", 
               driverName, _deviceName, functionName, function, *value );
  
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!
//! This function is used to read-out the status of the PANDA FSC HV supply
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynFscHV::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "readUInt32Digital";

  if ( function == _status ) { // Parameter "STATUS" should be read
    // Create the CAN frame to send command to HV
    can_frame_t frame;
    frame.can_id  = 0x480 | _createId; // 0x480 = 1001 000 0000
    frame.can_dlc = 1;
    frame.data[0] = 0x01;

    // Now send the command and try to read a response
    status = pasynGenericPointerSyncIO->writeRead( _pcanif, &frame, &frame, pasynUser->timeout );

    // Check if above function returned with a timeout-error
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, _deviceName, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }

    // Check if above function returned with any other unnormal status
    if ( status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d %s", 
                     driverName, _deviceName, functionName, status, function,
                     _pcanif->errorMessage );
      return asynError;
    }

    // Check if the reply matches our expectation
    if ( frame.can_id != ( 0x400 | _crateId ) || frame.can_dlc != 3 || frame.data[0] != 0x01 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d %02x...' where '%x08 3 0x01...' was expected.\033[0m", 
                     driverName, _deviceName, functionName, function, frame.can_id, frame.can_dlc, frame.data[0], (0x400 | _createId) );
      return asynError;
    }
   
    // Update the parameter in our local buffer
    // data byte 1 is value for switch (3rd param is a bitmask (switch is only 1 bit)
    status = setUIntDigitalParam( _switch, pframe.data[1], 0x01 );
    // byte 2 is the error byte. 
    status = setUIntDigitalParam( _status, pframe.data[2], mask );

    // Call parameter callbacks (if someone registered to monitor changes)
    status = (asynStatus) callParamCallbacks();
  }
  
  // read back parameter
  status = (asynStatus) getUIntDigitalParam( function, value, mask );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, _deviceName, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
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
asynStatus drvAsynFscHV::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeUInt32Digital";
  
  // Parameter "SWITCH" is the only writable UInt32Digital parameter...
  if ( function != _switch ) return asynSuccess;

  // Set the parameter in the parameter library.
  status = (asynStatus) setUIntDigitalParam( function, value, mask );
  
  // Do callbacks so higher layers see any changes
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d, mask=%u\033[0m", 
                   driverName, _deviceName, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, _deviceName, functionName, function, value, mask );

  // Create the CAN frame to send command to HV
  can_frame_t frame;
  frame.can_id  = 0x480 | _createId; // 0x480 = 1001 000 0000
  frame.can_dlc = 2;
  frame.data[0] = 0x02;
  frame.data[1] = ( value & 0x01 );

  // Now send the command and try to read a response
  status = pasynGenericPointerSyncIO->writeRead( _pcanif, &frame, &frame, pasynUser->timeout );

  // Check if above function returned with a timeout-error
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, _deviceName, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }

  // Check if above function returned with any other unnormal status
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, _deviceName, functionName, status, function,
                   _pcanif->errorMessage );
    return asynError;
  }

  // Check if the reply matches our expectation
  if ( frame.can_id != ( 0x400 | _crateId ) || frame.can_dlc != 3 || frame.data[0] != 0x02 || frame.data[1] != (value & 0x01) ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d %02x %02x ...' where '%x08 3 0x01 %02x...' was expected.\033[0m", 
                   driverName, _deviceName, functionName, function, frame.can_id, frame.can_dlc, frame.data[0], frame.data[1], (0x400 | _createId), (value & 0x01) );
    return asynError;
  }

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
asynStatus drvAsynFscHV::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  epicsUInt8 addr = 255;
  asynStatus status = asynSuccess;
  static const char *functionName = "readFloat64";
  
  if( function == _volt0 ) addr = 0;
  if( function == _volt1 ) addr = 1;
  if( function == _volt2 ) addr = 2;
  if( function == _curr0 ) addr = 3;
  if( function == _curr1 ) addr = 4;
  if( function == _curr2 ) addr = 5;

  if( 255 == addr ) return asynSuccess; 

  // Create the CAN frame to send command to HV
  can_frame_t frame;
  frame.can_id  = 0x480 | _createId; // 0x480 = 1001 000 0000
  frame.can_dlc = 2;
  frame.data[0] = 0x03;
  frame.data[1] = addr;

  // Now send the command and try to read a response
  status = pasynGenericPointerSyncIO->writeRead( _pcanif, &frame, &frame, pasynUser->timeout );

  // Check if above function returned with a timeout-error
  if ( asynTimeout == status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                   driverName, _deviceName, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }

  // Check if above function returned with any other unnormal status
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d %s", 
                   driverName, _deviceName, functionName, status, function,
                   _pcanif->errorMessage );
    return asynError;
  }

  // Check if the reply matches our expectation
  if ( frame.can_id != ( 0x400 | _crateId ) || frame.can_dlc != 6 || frame.data[0] != 0x03 || frame.data[1] != addr ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d %02x %02x...' where '%x08 6 0x03 %02x...' was expected.\033[0m", 
                   driverName, _deviceName, functionName, function, frame.can_id, frame.can_dlc, frame.data[0], frame.data[1], (0x400 | _createId), addr );
    return asynError;
  }
 
  // "convert" uint8_t[4] type data from can fame to 32bit-float
  can_data_t myValue;
  myValue.can[3] = frame.data[2];
  myValue.can[2] = frame.data[3];
  myValue.can[1] = frame.data[4];
  myValue.can[0] = frame.data[5];

  // Update the parameter in our local buffer
  status = setDoubleParam( function, myValue.fval );

  // Call parameter callbacks (if someone registered to monitor changes)
  status = (asynStatus) callParamCallbacks();
  
  // read back parameter
  status = (asynStatus) getDoubleParam( function, value );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, _deviceName, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%u, mask=%u\n", 
               driverName, _deviceName, functionName, function, *value, mask );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynFscHV class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  crate_id    The CAN id offset of the crate
//------------------------------------------------------------------------------
drvAsynFscHV::drvAsynFscHV( const char *portName, const char *CanPort, const int crate_id ) 
  : asynPortDriver( portName, 
                    1, /* maxAddr */ 
                    NUM_PANDAFSCHV_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  static const char *functionName = "drvAsynFscHV";

  asynStatus status = asynSuccess;

  _deviceName  = epicsStrDup( portName );
  _crateId    = crate_id;

  createParam( P_FSCHV_STATUS_STRING, asynParamUInt32Digital, &_status );
  createParam( P_FSCHV_SWITCH_STRING, asynParamUInt32Digital, &_switch );
  createParam( P_FSCHV_VOLT0_STRING,  asynParamFloat64,       &_volt0 );
  createParam( P_FSCHV_VOLT1_STRING,  asynParamFloat64,       &_volt1 );
  createParam( P_FSCHV_VOLT2_STRING,  asynParamFloat64,       &_volt2 );
  createParam( P_FSCHV_CURR0_STRING,  asynParamFloat64,       &_curr0 );
  createParam( P_FSCHV_CURR1_STRING,  asynParamFloat64,       &_curr1 );
  createParam( P_FSCHV_CURR2_STRING,  asynParamFloat64,       &_curr2 );
  createParam( P_FSCHV_TEMP_STRING,   asynParamInt32,         &_temp );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &_pcanif, 0 );
  if ( status != asynSuccess ) {
    printf( "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
            driverName, _deviceName, functionName, CanPort );
  }
  


}
