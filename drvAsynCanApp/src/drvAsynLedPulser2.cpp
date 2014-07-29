//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Ruhr-Universitaet Bochum, Lehrstuhl fuer Experimentalphysik I
// Extensions for sencond generation LED pulser:
// Copyright (C) 2014 Tobias Triffterer <tobias@ep1.ruhr-uni-bochum.de>
//               Ruhr-Universität Bochum, Insitut für Experimentalphysik I
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
// brief   Asyn driver for PANDA LED Pulser
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>

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

#include "drvAsynLedPulser2.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynLedPulser2Driver";

//_____ F U N C T I O N S ______________________________________________________

void drvAsynLedPulser2::setFrequency( epicsInt32 value, epicsUInt8 &high, epicsUInt8 &low ) {
  const epicsUInt32 ocr1a = ( epicsUInt32 ) ( epicsFloat64 ( _ucClockFrequency ) / ( epicsFloat64 ( _ucPrescaler ) * epicsFloat64 ( value ) ) - 0.5 );

  high = (epicsUInt8)( ( ocr1a & 0xff00 ) >> 8 );
  low  = (epicsUInt8)( ocr1a & 0x00ff );
}

//------------------------------------------------------------------------------

epicsInt32 drvAsynLedPulser2::getFrequency( epicsUInt8 high, epicsUInt8 low ) {
  const epicsUInt32 ocr1a = ( high << 8 ) + low;
  epicsFloat64 rtn = epicsFloat64 ( _ucClockFrequency ) / ( epicsFloat64 ( _ucPrescaler ) * epicsFloat64 ( ocr1a + 1 ) );
  return rtn;
}

//------------------------------------------------------------------------------

void drvAsynLedPulser2::setIntensity( epicsUInt16 value, epicsUInt8 &high, epicsUInt8 &low ){
  high = (epicsUInt8)( ( value & 0xff00 ) >> 8 );
  low  = (epicsUInt8)( value & 0x00ff );
}

//------------------------------------------------------------------------------

epicsUInt16 drvAsynLedPulser2::getIntensity( epicsUInt8 high, epicsUInt8 low ){
  return (epicsUInt16) ( high << 8 ) + low;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          If pasynUser->reason is equal to
//!          - P_write the values of the other parameters are send to the light pulser
//!          - P_read  the values of the other parameters are read from the light pulser
//
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynLedPulser2::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeInt32";
  
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

  if ( function == P_write ) {
    can_frame_t pframe;
    pframe.can_id  = _can_id;
    pframe.can_dlc = 8;

    pframe.data[0] = 0x02; // Command charactor for writing configuration
    
    epicsInt32 dummyInt;
    epicsUInt32 dummyUInt;
    //epicsFloat64 dummyFloat;

    getUIntDigitalParam( P_Color_out, &dummyUInt, 7 );
    pframe.data[1] = (epicsUInt8)dummyUInt; // Color and trigger mode connected in data byte 1: bits 0, 1, 2: color

    getIntegerParam( P_Intensity_out, &dummyInt );
    setIntensity( dummyInt, pframe.data[2], pframe.data[3] );

    getIntegerParam( P_Cycles_out, &dummyInt );
    pframe.data[4] = (epicsUInt8)( ( dummyInt & 0xff00 ) >> 8 );
    pframe.data[5] = (epicsUInt8)( dummyInt & 0x00ff );

    getUIntDigitalParam( P_Trg_Mode_out, &dummyUInt, 3 );
    pframe.data[1] |= (epicsUInt8) ( dummyUInt << 4 ); // Color and trigger mode connected in data byte 1: bits 4 and 5: trigger mode

    getIntegerParam( P_Frequency_out, &dummyInt );
    setFrequency( dummyInt, pframe.data[6], pframe.data[7] );

    //status = pasynGenericPointerSyncIO->writeRead( _pasynGenericPointer, &pframe, &pframe, pasynUser->timeout );
    status = pasynGenericPointerSyncIO->write( _pasynGenericPointer, &pframe, pasynUser->timeout );
    if ( status ) {
      //epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      //               "\033[31;1m%s:%s:%s: function=%d, No reply from device within %f s.\033[0m", 
      //               driverName, _deviceName, functionName, function, pasynUser->timeout );
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, WRITE STATUS=%d No reply from device within %f s.\033[0m", 
                     driverName, _deviceName, functionName, function, status, pasynUser->timeout );
      return asynError;
    }
    epicsThreadSleep(0.05);
    status = pasynGenericPointerSyncIO->read( _pasynGenericPointer, &pframe, pasynUser->timeout );
    if ( status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, READ STATUS=%d No reply from device within %f s.\033[0m", 
                     driverName, _deviceName, functionName, function, status, pasynUser->timeout );
      return asynError;
    }
    // The LED pulser will answer with its current configuration to make sure everything has been set properly.
    // Assign these received values to the input parameters:
    parseReadCanFrame( pframe );
    callParamCallbacks();
    
  }
  if ( function == P_read ) {
    can_frame_t pframe;
    pframe.can_id  = _can_id;
    pframe.can_dlc = 1;
    pframe.data[0] = 0x01; // Command character to read configuration

    status = pasynGenericPointerSyncIO->writeRead( _pasynGenericPointer, &pframe, &pframe, pasynUser->timeout );
    if ( status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, No reply from device within %f s.\033[0m", 
                     driverName, _deviceName, functionName, function, pasynUser->timeout );
      return asynError;
    }
    if (pframe.can_id != _can_id || pframe.can_dlc != 8)
    {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, Received data not belonging to the LedPulser2 with CAN ID 0x%x. Received packet has ID 0x%x.\033[0m", 
                     driverName, _deviceName, functionName, function, _can_id, pframe.can_id );
      return asynError;
    }
    parseReadCanFrame( pframe );

    status = (asynStatus) callParamCallbacks();
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief Parse a CAN frame
//!
//! Extracts the light pulser data from a CAN frame and assigns it to the asyn parameters.
//!
//! @param [in] pframe                  A CAN frame received from the LED pulser
//! @param [in] alsoAssignOutputParams  If true, the data received from the LED pulser will
//!                                     also be written to the output parameters.
//!                                     This is used for initialization.
//------------------------------------------------------------------------------
void drvAsynLedPulser2::parseReadCanFrame( const can_frame_t& pframe, const bool alsoAssignOutputParams ) {
  epicsUInt16  readColor     = pframe.data[1] & 0x07; // Color and trigger mode connected in data byte 1: bits 0, 1, 2: color
  epicsInt32   readIntensity = getIntensity( pframe.data[2], pframe.data[3] );
  epicsUInt16  readCycles    = ( pframe.data[4] << 8 ) | ( pframe.data[5]);
  epicsUInt16  readTrgMode   = ( pframe.data[1] & 0x30 ) >> 4; // Color and trigger mode connected in data byte 1: bits 4 and 5: trigger mode
  epicsInt32   readFrequency = getFrequency( pframe.data[6], pframe.data[7] );
    
  setUIntDigitalParam( P_Color_in, readColor, 7 );
  setIntegerParam( P_Intensity_in, readIntensity );
  setIntegerParam( P_Cycles_in, readCycles );
  setUIntDigitalParam( P_Trg_Mode_in, readTrgMode, 3 );
  setIntegerParam( P_Frequency_in, readFrequency );
  if ( alsoAssignOutputParams ) {
    setUIntDigitalParam( P_Color_out, readColor, 7 );
    setIntegerParam( P_Intensity_out, readIntensity );
    setIntegerParam( P_Cycles_out, readCycles );
    setUIntDigitalParam( P_Trg_Mode_out, readTrgMode, 3 );
    setIntegerParam( P_Frequency_out, readFrequency );
  }
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynLedPulser2 class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id      The can id of the light pulser
//! @param   [in]  filename    Name of the file containing calibration data for LCD
//------------------------------------------------------------------------------
drvAsynLedPulser2::drvAsynLedPulser2( const char *portName, const char *CanPort,
                                      const int can_id ) 
  : asynPortDriver( portName, 
                    0, // maxAddr 
                    NUM_LEDPULSER2_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  // Interrupt mask
                    ASYN_CANBLOCK, // asynFlags
                    1, // Autoconnect
                    0, // Default priority
                    0 ) // Default stack size
{
  const char *functionName = "drvAsynLedPulser2";
  asynStatus status = asynSuccess;
  
  _deviceName  = epicsStrDup( portName );
  _can_id      = can_id;

  // Create parameters
  // Input variables, i.e. data read from the LED pulser
  createParam( P_LEDPULSER2_INTENSITY_IN_STRING,  asynParamInt32,         &P_Intensity_in );
  createParam( P_LEDPULSER2_FREQUENCY_IN_STRING,  asynParamInt32,         &P_Frequency_in );
  createParam( P_LEDPULSER2_CYCLES_IN_STRING,     asynParamInt32,         &P_Cycles_in );
  createParam( P_LEDPULSER2_TRG_MODE_IN_STRING,   asynParamUInt32Digital, &P_Trg_Mode_in );
  createParam( P_LEDPULSER2_COLOR_IN_STRING,      asynParamUInt32Digital, &P_Color_in );
  // Output variables, i.e. data to be written to the light pulser
  createParam( P_LEDPULSER2_INTENSITY_OUT_STRING, asynParamInt32,         &P_Intensity_out );
  createParam( P_LEDPULSER2_FREQUENCY_OUT_STRING, asynParamInt32,         &P_Frequency_out );
  createParam( P_LEDPULSER2_CYCLES_OUT_STRING,    asynParamInt32,         &P_Cycles_out );
  createParam( P_LEDPULSER2_TRG_MODE_OUT_STRING,  asynParamUInt32Digital, &P_Trg_Mode_out );
  createParam( P_LEDPULSER2_COLOR_OUT_STRING,     asynParamUInt32Digital, &P_Color_out );
  // Command variables to tell the driver when to initiate CAN communication
  createParam( P_LEDPULSER2_WRITE_STRING,         asynParamInt32,         &P_write );
  createParam( P_LEDPULSER2_READ_STRING,          asynParamInt32,         &P_read );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &_pasynGenericPointer, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: can't connect to asynGenericPointer on port '%s'\033[0m\n", 
             driverName, _deviceName, functionName, CanPort );
    return;
  }
  
  // Get inital value for Switch-Parameter
  can_frame_t pframe;
  pframe.can_id  = _can_id;
  pframe.can_dlc = 1;
  pframe.data[0] = 0x01; // Command character to read configuration
  status = pasynGenericPointerSyncIO->writeRead( _pasynGenericPointer, &pframe, &pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: No reply from device within 1 s!\033[0m\n",
             driverName, _deviceName, functionName );
    return;
  }

  // Set initial values on parameters for both input and output
  parseReadCanFrame( pframe, true );
  // invoke callParamCallbacks()?
}

//******************************************************************************
//! EOF
//******************************************************************************
