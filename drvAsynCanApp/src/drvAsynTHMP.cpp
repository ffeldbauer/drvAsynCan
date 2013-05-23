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
// brief   Asyn driver for ISEG EHS 8 620p-F and ISEG EHS 8 210p-F
//          high voltage modules using the RPi Can interface
//
// version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

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

#include "drvAsynTHMP.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynTHMPDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Background process to periodically poll for CAN frames from CanPort
//!
//! @param   [in]  drvPvt Pointer to drvAsynTHMP class object
//!
//! @sa      drvAsynTHMP::readPoller
//! @sa      drvAsynTHMP::drvAsynTHMP
//------------------------------------------------------------------------------
void THMPreadPoller( void *drvPvt ) {
  drvAsynTHMP *pPvt = (drvAsynTHMP *)drvPvt;
  pPvt->readPoller();
}

//------------------------------------------------------------------------------
//! @brief   Background process to periodically poll for CAN frames from CanPort
//!
//!          Periodically poll CanPort for 1 second for new CAN frames.
//!          If a new frame is received and its id matches can_id_ the
//!          corresponding parameter will be updated and the callback will be called
//!
//! @sa      drvAsynTHMP::drvAsynTHMP
//------------------------------------------------------------------------------
void drvAsynTHMP::readPoller() {
  asynStatus status;
  const char* functionName = "readPoller";
  can_frame_t *pframe = new can_frame_t;
  
  /* Loop forever */    
  while (1) {
    status = pasynGenericPointerSyncIO->read( pAsynUserGenericPointer_, pframe, 1. );
    if ( status == asynTimeout )      continue;
    if ( pframe->can_id != can_id_ )  continue;
    
    switch ( pframe->data[0] ) {
      
    case 0x01: // ADC Conversion
      if ( pframe->can_dlc != 4 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid data length of frame for command 0x01: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->can_dlc );
        break;
      }
      if ( pframe->data[1] >= 64 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid channel number for command 0x01: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->data[1] );
        break;
      }
      // epicsInt32 myValue = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
      status = (asynStatus) setIntegerParam( pframe->data[1], P_RawValue,
                                             ( pframe->data[2] << 8 ) | ( pframe->data[3]) );
      if( status ) 
        fprintf( stderr, "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m\n", 
                 driverName, deviceName_, functionName, status, P_RawValue,
                 ( pframe->data[2] << 8 ) | ( pframe->data[3]) );
      callParamCallbacks( pframe->data[1], pframe->data[1] );
      break;
      
    case 0x03: // I/O Board
      if ( pframe->can_dlc != 4 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid data length of frame for command 0x03: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->can_dlc );
        break;
      }
      if ( pframe->data[1] >= 2 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid channel number for command 0x03: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->data[1] );
        break;
      }
      // epicsUInt32 myValue = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
      status = (asynStatus) setUIntDigitalParam( pframe->data[1], P_IoBoard,
                                                 ( pframe->data[2] << 8 ) | ( pframe->data[3]),
                                                 0xffff );
      if( status ) 
        fprintf( stderr, "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m\n", 
                 driverName, deviceName_, functionName, status, P_IoBoard,
                 ( pframe->data[2] << 8 ) | ( pframe->data[3]) );
      callParamCallbacks( pframe->data[1], pframe->data[1] );
      break;
      
    case 0x04: // Serials
      if ( pframe->can_dlc != 8 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid data length of frame for command 0x04: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->can_dlc );
        break;
      }
      if ( pframe->data[1] >= 9 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid channel number for command 0x04: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->data[2] );
        break;
      }
      // epicsInt32 myValue = ( pframe->data[2] << 16 ) | ( pframe->data[3] << 8 ) | ( pframe->data[4]);
      status = (asynStatus) setUIntDigitalParam( pframe->data[1], P_Serials,
                                                 ( pframe->data[2] << 16 ) | ( pframe->data[3] << 8 ) | ( pframe->data[4] ),
                                                 0xffffff );
      status = (asynStatus) setUIntDigitalParam( pframe->data[1] + 9, P_Serials,
                                                 ( pframe->data[5] << 16 ) | ( pframe->data[6] << 8 ) | ( pframe->data[7] ),
                                                 0xffffff );
      if( status ) 
        fprintf( stderr, "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m\n", 
                 driverName, deviceName_, functionName, status, P_Serials,
                 ( pframe->data[2] << 8 ) | ( pframe->data[4]));
      callParamCallbacks( pframe->data[1], pframe->data[1] );
      break;
      
    case 0xff: // Firmware
      if ( pframe->can_dlc != 3 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid data length of frame for command 0xff: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->can_dlc );
        break;
      }
      //epicsUInt32 myValue  = ( pframe->data[2] << 8 ) | ( pframe->data[3]);
      status = (asynStatus) setUIntDigitalParam( 0, P_Firmware,
                                                 ( pframe->data[1] << 8 ) | ( pframe->data[2]),
                                                 0xffff );
      if( status ) 
        fprintf( stderr, "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m\n", 
                 driverName, deviceName_, functionName, status, P_Serials,
                 ( pframe->data[1] << 8 ) | ( pframe->data[2]) );
      callParamCallbacks( 0, 0 );
      break;
      
    case 0xe0: // Error message
      if ( pframe->can_dlc != 3 ) {
        fprintf( stderr, "\033[31;1m%s:%s:%s: invalid data length of frame for command 0xe0: %d\033[0m\n", 
                 driverName, deviceName_, functionName, pframe->can_dlc );
        break;
      }
      //epicsUInt32 myValue  = ( pframe->data[2] << 2 ) | ( pframe->data[3]);
      status = (asynStatus) setUIntDigitalParam( 0, P_Error,
                                                 ( pframe->data[1] << 8 ) | ( pframe->data[2]),
                                                 0xffff );
      if( status ) 
        fprintf( stderr, "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m\n", 
                 driverName, deviceName_, functionName, status, P_Error, 
                ( pframe->data[1] << 8 ) | ( pframe->data[2]) );
      callParamCallbacks( 0, 0 );
      break;
    }
  }
  delete pframe;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          Trigger read out raw values, IO values, and serials, respectively
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
 asynStatus drvAsynTHMP::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";

  if ( function == P_RawValue ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  
  can_frame_t pframe;
  pframe.can_id = can_id_;
  if ( function == P_ConfigIO ) {
    if ( addr > 1 ) return asynError;
    pframe.can_dlc = 3;
    pframe.data[0] = 0x05;
    pframe.data[1] = (epicsUInt8)( addr & 0xff );
    pframe.data[2] = (epicsUInt8)( value & 0xff );
  } else if ( function == P_Trg_ADC ) {
    if ( addr != 0 ) return asynError;
    pframe.can_dlc = 1;
    pframe.data[0] = 0x01;
  } else if ( function == P_Trg_IO ) {
    if ( addr != 0 ) return asynError;
    pframe.can_dlc = 2;
    pframe.data[0] = 0x03;
    pframe.data[1] = (epicsUInt8) ( addr & 0xff );
  } else if ( function == P_Trg_Serials ) {
    if ( addr != 0 ) return asynError;
    pframe.can_dlc = 1;
    pframe.data[0] = 0x04;
  } else {
    return asynError;
  }

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Could not send can frame.\033[0m", 
                   driverName, deviceName_, functionName, function );
    return asynError;
  }
  
  return status;
 }

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->write().
//!
//!          If pasynUser->reason is equal to P_IoBoard the settings of
//!          the corresponding I/O piggyback board are updated
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynTHMP::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  if ( function != P_IoBoard ) return asynSuccess;
  
  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( addr > 1 ) return asynSuccess;
  
  /* Set the parameter in the parameter library. */
  status = (asynStatus) setUIntDigitalParam( addr, function, value, mask );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks( addr, addr );
  
  if (status) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, deviceName_, functionName, function, value, mask );
  
  can_frame_t pframe;
  pframe.can_id = can_id_;
  pframe.can_dlc = 4;
  pframe.data[0] = 0x02;
  pframe.data[1] = (epicsUInt8)( addr & 0xff );
  pframe.data[2] = (epicsUInt8)( value >> 8 );
  pframe.data[3] = (epicsUInt8)( value & 0xff );
  
  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, &pframe, pasynUser->timeout );
  
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynTHMP class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id      The CAN id of this THMP
//------------------------------------------------------------------------------
drvAsynTHMP::drvAsynTHMP( const char *portName, const char *CanPort,
                          const int can_id ) 
  : asynPortDriver( portName, 
                    64, /* maxAddr */ 
                    NUM_THMP_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  asynStatus status;
  const char *functionName = "drvAsynTHMP";

  // Create parameters
  createParam( P_THMP_ADCBUFFER_STRING,     asynParamInt32,         &P_RawValue );
  createParam( P_THMP_IOBUFFER_STRING,      asynParamUInt32Digital, &P_IoBoard );
  createParam( P_THMP_CONFIGIO_STRING,      asynParamInt32,         &P_ConfigIO );
  createParam( P_THMP_SERIALS_STRING,       asynParamUInt32Digital, &P_Serials );
  createParam( P_THMP_FIRMWARE_STRING,      asynParamUInt32Digital, &P_Firmware );
  createParam( P_THMP_ERROR_STRING,         asynParamUInt32Digital, &P_Error );
  createParam( P_THMP_TRG_ADCBUFFER_STRING, asynParamInt32,         &P_Trg_ADC );
  createParam( P_THMP_TRG_IOBUFFER_STRING,  asynParamInt32,         &P_Trg_IO );
  createParam( P_THMP_TRG_SERIALS_STRING,   asynParamInt32,         &P_Trg_Serials );
  
  deviceName_  = epicsStrDup( portName );
  can_id_      = can_id;
  
  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: can't connect to asynGenericPointer on port '%s'\033[0m\n", 
             driverName, deviceName_, functionName, CanPort );
    return;
  }

  this->eventId_ = epicsEventCreate( epicsEventEmpty );
  /* Create the thread that polls interface for received messages */
  status = (asynStatus)( epicsThreadCreate( "drvAsynTHMPTask",
                                            epicsThreadPriorityMedium,
                                            epicsThreadGetStackSize(epicsThreadStackMedium),
                                            (EPICSTHREADFUNC)::THMPreadPoller,
                                            this ) == NULL );
  if (status) {
    fprintf( stderr, "\033[31;1m%s:%s: epicsThreadCreate failure\033[0m\n", driverName, functionName);
    return;
  }
  
}

//******************************************************************************
//! EOF
//******************************************************************************
