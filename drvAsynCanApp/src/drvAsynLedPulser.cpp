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
//! @brief   Asyn driver for PANDA LED Pulser
//!          using the RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
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

#include "drvAsynLedPulser.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynLedPulserDriver";

//_____ F U N C T I O N S ______________________________________________________

void drvAsynLedPulser::setFrequency( epicsFloat64 value, epicsUInt8 &high, epicsUInt8 &low ) {
  epicsFloat64 ocr1aD = 8000. / ( 16. * value ) - 1;
  epicsUInt32 ocr1a = (epicsUInt32)( ocr1aD + 0.5 );

  high = (epicsUInt8)( ( ocr1a & 0xff00 ) >> 8 );
  low  = (epicsUInt8)( ocr1a & 0x00ff );
}

//------------------------------------------------------------------------------

epicsFloat64 drvAsynLedPulser::getFrequency( epicsUInt8 high, epicsUInt8 low ) {
  epicsUInt32 ocr1a = ( high << 8 ) + low + 1;
  epicsFloat64 rtn = 8000. / ( 16. * ocr1a );
  return rtn;
}

//------------------------------------------------------------------------------

void drvAsynLedPulser::setIntensity( epicsFloat64 value, epicsUInt8 &high, epicsUInt8 &low ){
  epicsFloat64 diff = 1000.;
  epicsUInt32 sval = 2047;
  std::vector<LCD_values>::const_iterator it = lcd_values_.begin();

  if ( 0 == value ) {
    sval = 2047;
  } else {
    for ( ; it != lcd_values_.end(); ++it ) {
      if ( std::fabs( value - it->intensity ) < diff ) {
        diff = std::fabs( value - it->intensity );
        sval = it->dac;
      }
    }
  }
  
  high = (epicsUInt8)( ( sval & 0xff00 ) >> 8 );
  low  = (epicsUInt8)( sval & 0x00ff );
}

//------------------------------------------------------------------------------

epicsFloat64 drvAsynLedPulser::getIntensity( epicsUInt8 high, epicsUInt8 low ){
  epicsUInt32 val = ( high << 8 ) + low;
  epicsFloat64 rtn;
  std::vector<LCD_values>::const_iterator it = lcd_values_.begin();

  for ( ; it != lcd_values_.end(); ++it ) {
    if ( it->dac == val ) {
      rtn = it->intensity;
      break;
    } else {
      rtn = -1.;
    }
  }
  return rtn;
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
asynStatus drvAsynLedPulser::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
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

  if ( function == P_write ) {
    can_frame_t *pframe = new can_frame_t;
    pframe->can_id  = can_id_;
    pframe->can_dlc = 8;

    epicsInt32 dummyInt;
    epicsUInt32 dummyUInt;
    epicsFloat64 dummyFloat;

    getUIntDigitalParam( P_Color, &dummyUInt, 3 );
    pframe->data[0] = (epicsUInt8)dummyUInt;

    getDoubleParam( P_Intensity, &dummyFloat );
    setIntensity( dummyFloat, pframe->data[1], pframe->data[2] );

    getIntegerParam( P_Cycles, &dummyInt );
    pframe->data[3] = (epicsUInt8)( ( dummyInt & 0xff00 ) >> 8 );
    pframe->data[4] = (epicsUInt8)( dummyInt & 0x00ff );

    getUIntDigitalParam( P_Trg_Mode, &dummyUInt, 1 );
    pframe->data[0] = (epicsUInt8)dummyUInt;

    getDoubleParam( P_Frequency, &dummyFloat );
    setIntensity( dummyFloat, pframe->data[6], pframe->data[7] );

    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, pasynUser->timeout );
    if ( status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, No reply from device within %f s.\033[0m", 
                     driverName, deviceName_, functionName, function, pasynUser->timeout );
      return asynError;
    }
  }
  if ( function == P_read ) {
    can_frame_t *pframe = new can_frame_t;
    pframe->can_id  = can_id_;
    pframe->can_dlc = 1;
    pframe->data[0] = 0;

    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, pasynUser->timeout );
    if ( status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, No reply from device within %f s.\033[0m", 
                     driverName, deviceName_, functionName, function, pasynUser->timeout );
      return asynError;
    }
    epicsUInt16  readColor     = pframe->data[0];
    epicsFloat64 readIntensity = getIntensity( pframe->data[1], pframe->data[2] );
    epicsUInt16  readCycles    = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
    epicsUInt16  readTrgMode   = pframe->data[5];
    epicsFloat64 readFrequency = getFrequency( pframe->data[6], pframe->data[7] );
    
    setUIntDigitalParam( P_Color, readColor, 3 );
    setDoubleParam( P_Intensity, readIntensity );
    setIntegerParam( P_Cycles, readCycles );
    setUIntDigitalParam( P_Trg_Mode, readTrgMode, 1 );
    setDoubleParam( P_Frequency, readFrequency );

    status = (asynStatus) callParamCallbacks();
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynLedPulser class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id      The can id of the light pulser
//! @param   [in]  filename    Name of the file containing calibration data for LCD
//------------------------------------------------------------------------------
drvAsynLedPulser::drvAsynLedPulser( const char *portName, const char *CanPort,
                                    const int can_id, const char *filename ) 
  : asynPortDriver( portName, 
                    0, /* maxAddr */ 
                    NUM_LEDPULSER_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynLedPulser";
  asynStatus status = asynSuccess;
  
  deviceName_  = epicsStrDup( portName );
  can_id_      = can_id;

  // Read in calibration of LCD
  char *path = getenv("LIGHT_PULSER_CALIBRATION");
  strcat( path, filename );
  std::ifstream input( path, std::ios_base::in );
  std::string line;

  if ( !input.is_open() ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: Can not open file '%s': %s\033[0m\n", 
             driverName, deviceName_, functionName, path, strerror( errno ) );
    return;
  }
  while( std::getline( input, line ) ) {
    
    // handle comments in config file
    size_t comment;
    if ( (comment = line.find('#') ) != std::string::npos )
      line.erase( comment ); // erase everthing after first '#' from string
    
    if ( 0 == line.length() ) continue; // string is empty, no need to parse it
      
    std::istringstream parse( line );
    std::string dac_;
    std::string intensity_;
    
    getline( parse, dac_,  ' ' );
    getline( parse, intensity_, ' ' );
    
    epicsUInt16 dacDummy = atoi( dac_.c_str() );
    if ( 0 == dacDummy ) {
      fprintf( stderr, "\033[31;1m%s:%s:%s: Error reading config file: Syntax error in line '%s'\033[0m\n",
               driverName, deviceName_, functionName, line.c_str() );
      return;
    }
    epicsFloat64 intDummy = (epicsFloat64)atof( intensity_.c_str() );
    if ( 0.0 == intDummy ) {
      fprintf( stderr, "\033[31;1m%s:%s:%s: Error reading config file: Syntax error in line '%s'\033[0m\n",
               driverName, deviceName_, functionName, line.c_str() );
      return;
    }
    LCD_values dummyLCD = { dacDummy, intDummy };
    lcd_values_.push_back( dummyLCD );
  }
  input.close();

  // Create parameters
  createParam( P_LEDPULSER_INTENSITY_STRING, asynParamFloat64,       &P_Intensity );
  createParam( P_LEDPULSER_FREQUENCY_STRING, asynParamFloat64,       &P_Frequency );
  createParam( P_LEDPULSER_CYCLES_STRING,    asynParamInt32,         &P_Cycles );
  createParam( P_LEDPULSER_TRG_MODE_STRING,  asynParamUInt32Digital, &P_Trg_Mode );
  createParam( P_LEDPULSER_COLOR_STRING,     asynParamUInt32Digital, &P_Color );
  createParam( P_LEDPULSER_WRITE_STRING,     asynParamInt32,         &P_write );
  createParam( P_LEDPULSER_READ_STRING,      asynParamInt32,         &P_read );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: can't connect to asynGenericPointer on port '%s'\033[0m\n", 
             driverName, deviceName_, functionName, CanPort );
    return;
  }
  
  // Get inital value for Switch-Parameter
  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = can_id_;
  pframe->can_dlc = 1;
  pframe->data[0] = 0x00;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: No reply from device within 1 s!\033[0m\n",
             driverName, deviceName_, functionName );
    return;
  }

  epicsUInt16  initColor     = pframe->data[0];
  epicsFloat64 initIntensity = getIntensity( pframe->data[1], pframe->data[2] );
  epicsUInt16  initCycles    = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
  epicsUInt16  initTrgMode   = pframe->data[5];
  epicsFloat64 initFrequency = getFrequency( pframe->data[6], pframe->data[7] );
  
  setUIntDigitalParam( P_Color, initColor, 3 );
  setDoubleParam( P_Intensity, initIntensity );
  setIntegerParam( P_Cycles, initCycles );
  setUIntDigitalParam( P_Trg_Mode, initTrgMode, 1 );
  setDoubleParam( P_Frequency, initFrequency );

}

//******************************************************************************
//! EOF
//******************************************************************************
