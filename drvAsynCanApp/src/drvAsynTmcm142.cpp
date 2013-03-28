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
//! @brief   Asyn driver for TMCM142 1-axis stepper controller/driver
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

#include "drvAsynTmcm142.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//! helper to split the 32-bit integer value from the records into 4 unsigned chars
typedef union{
  epicsInt32 val32;
  epicsUInt8 val8[4];
} split_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynTMCM142Driver";

//_____ F U N C T I O N S ______________________________________________________

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
asynStatus drvAsynTmcm142::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char *functionName = "readInt32";
    
  status = getAddress( pasynUser, &addr ); if( status ) return status;

  std::map<int, epicsUInt8>::const_iterator it = cmds_r_.find( function );
  if( it == cmds_r_.end() ) {
    status = (asynStatus) getIntegerParam( addr, function, value );
    if( status ) 
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s: status=%d, function=%d, value=%d", 
                     driverName, functionName, status, function, *value );
    else        
      asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
                 "%s:%s: function=%d, value=%d\n", 
                 driverName, functionName, function, *value );
    return status;
  }
  
  can_frame_t pframe;
  pframe.can_id = can_id_w_;
  pframe.can_dlc = 7;
  pframe.data[0] = it->second;      // Command number
  pframe.data[1] = addr;            // Type
  pframe.data[2] = 0;               // Motor/Bank address
  pframe.data[3] = 0;
  pframe.data[4] = 0;
  pframe.data[5] = 0;
  pframe.data[6] = 0;

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
  if ( ( pframe.can_id  != can_id_r_ ) ||
       ( pframe.can_dlc != 7 ) ||
       ( pframe.data[0] != ( can_id_w_ & 0xff ) ) ||
       ( pframe.data[2] != it->second ) 
       ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x where %08x 7 %02x XX %02x was expected\033[0m", 
                   driverName, deviceName_, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2],
                   can_id_r_, ( can_id_w_ & 0xff ), it->second );
    return asynError;
  }
  // Update status
  status = (asynStatus) setIntegerParam( 0, P_STATUS, pframe.data[1] );
  status = (asynStatus) callParamCallbacks( 0, 0 );

  //  epicsInt32 myValue = *(epicsInt32*)&preadframe->data[3];
  split_t myValue;
  myValue.val8[3] = pframe.data[3];
  myValue.val8[2] = pframe.data[4];
  myValue.val8[1] = pframe.data[5];
  myValue.val8[0] = pframe.data[6];
  
  // update value of parameter
  status = (asynStatus) setIntegerParam( addr, function, myValue.val32 );
  status = (asynStatus) getIntegerParam( addr, function, value );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%d", 
                   driverName, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d\n", 
               driverName, functionName, function, *value );
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
asynStatus drvAsynTmcm142::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";

  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  std::map<int, epicsUInt8>::const_iterator it = cmds_w_.find( function );
  if( it == cmds_w_.end() ) return asynError;
  
  split_t myValue; myValue.val32 = value;
  can_frame_t pframe;
  pframe.can_id = can_id_w_;
  pframe.can_dlc = 7;
  pframe.data[0] = it->second;      // Command number
  pframe.data[1] = addr;            // Type
  pframe.data[2] = 0;               // Motor/Bank address
  // *(epicsUInt32*)&pframe.data[3] = value;
  pframe.data[3] = myValue.val8[3];
  pframe.data[4] = myValue.val8[2];
  pframe.data[5] = myValue.val8[1];
  pframe.data[6] = myValue.val8[0];

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
  if ( ( pframe.can_id  != can_id_r_ ) ||
       ( pframe.can_dlc != 7 ) ||
       ( pframe.data[0] != ( can_id_w_ & 0xff ) ) ||
       ( pframe.data[2] != it->second ) 
       ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x %02x where %08x 7 %02x XX %02x was expected\033[0m", 
                   driverName, deviceName_, functionName, function,
                   pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2],
                   can_id_r_, ( can_id_w_ & 0xff ), it->second );
    return asynError;
  }
  // update status
  status = (asynStatus) setIntegerParam( 0, P_STATUS, pframe.data[1] );
  status = (asynStatus) callParamCallbacks( 0, 0 );

  // update value of parameter
  status = (asynStatus)setIntegerParam( addr, function, value );
  status = (asynStatus)callParamCallbacks( addr, addr );
    
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s:%s: status=%d, function=%d, value=%d", 
                   driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d\n", 
               driverName, functionName, function, value );

  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynTmcm142 class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id_w    The can id of the TMCM142 driver
//! @param   [in]  can_id_r    The can reply id of the TMCM142 driver
//------------------------------------------------------------------------------
drvAsynTmcm142::drvAsynTmcm142( const char *portName, const char *CanPort,
                                const int can_id_w, const int can_id_r ) 
  : asynPortDriver( portName, 
                    256, /* maxAddr */ 
                    NUM_TMCM142_PARAMS,
                    asynCommonMask | asynInt32Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynTmcm142";
  asynStatus status = asynSuccess;
  
  deviceName_  = epicsStrDup( portName );
  can_id_w_    = can_id_w;
  can_id_r_    = can_id_r;
  
  createParam( P_TMCM142_ROR_STRING,    asynParamInt32,         &P_ROR );  // rotate right
  createParam( P_TMCM142_ROL_STRING,    asynParamInt32,         &P_ROL );  // rotate left
  createParam( P_TMCM142_MST_STRING,    asynParamInt32,         &P_MST );  // motor stop
  createParam( P_TMCM142_MVP_STRING,    asynParamInt32,         &P_MVP );  // move to position
  createParam( P_TMCM142_SAP_STRING,    asynParamInt32,         &P_SAP );  // set axis parameter
  createParam( P_TMCM142_GAP_STRING,    asynParamInt32,         &P_GAP );  // get axis parameter
  createParam( P_TMCM142_STAP_STRING,   asynParamInt32,         &P_STAP ); // store axis parameter
  createParam( P_TMCM142_RSAP_STRING,   asynParamInt32,         &P_RSAP ); // restore axis parameter
  createParam( P_TMCM142_SGP_STRING,    asynParamInt32,         &P_SGP );  // set global parameter
  createParam( P_TMCM142_GGP_STRING,    asynParamInt32,         &P_GGP );  // get global parameter
  createParam( P_TMCM142_STGP_STRING,   asynParamInt32,         &P_STGP ); // store global parameter
  createParam( P_TMCM142_RSGP_STRING,   asynParamInt32,         &P_RSGP ); // restore global parameter
  createParam( P_TMCM142_RFS_STRING,    asynParamInt32,         &P_RFS );  // reference search
  createParam( P_TMCM142_SIO_STRING,    asynParamInt32,         &P_SIO );  // set digital io
  createParam( P_TMCM142_GIO_STRING,    asynParamInt32,         &P_GIO );  // get digital io
  createParam( P_TMCM142_CALC_STRING,   asynParamInt32,         &P_CALC ); // process accumulator
  createParam( P_TMCM142_COMP_STRING,   asynParamInt32,         &P_COMP ); // compare accumulator & value
  createParam( P_TMCM142_JC_STRING,     asynParamInt32,         &P_JC );   // jump conditional
  createParam( P_TMCM142_JA_STRING,     asynParamInt32,         &P_JA );   // jump absolute
  createParam( P_TMCM142_CSUB_STRING,   asynParamInt32,         &P_CSUB ); // call subroutine
  createParam( P_TMCM142_RSUB_STRING,   asynParamInt32,         &P_RSUB ); // return from subroutine
  createParam( P_TMCM142_WAIT_STRING,   asynParamInt32,         &P_WAIT ); // wait with further program execution
  createParam( P_TMCM142_STOP_STRING,   asynParamInt32,         &P_STOP ); // stop program execution
  createParam( P_TMCM142_SCO_STRING,    asynParamInt32,         &P_SCO );  // set coordinate
  createParam( P_TMCM142_GCO_STRING,    asynParamInt32,         &P_GCO );  // get coordinate
  createParam( P_TMCM142_CCO_STRING,    asynParamInt32,         &P_CCO );  // caputre coordinate
  createParam( P_TMCM142_CALCX_STRING,  asynParamInt32,         &P_CALCX ); // process accumulator & X-register
  createParam( P_TMCM142_AAP_STRING,    asynParamInt32,         &P_AAP );  // accumulator to axis parameter
  createParam( P_TMCM142_AGP_STRING,    asynParamInt32,         &P_AGP );  // accumulator to global parameter
  createParam( P_TMCM142_CLE_STRING,    asynParamInt32,         &P_CLE );  // clear error flags
  createParam( P_TMCM142_ACO_STRING,    asynParamInt32,         &P_ACO );  // accumulator to coordinate

  createParam( P_TMCM142_STATUS_STRING, asynParamInt32,         &P_STATUS );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: can't connect to asynGenericPointer on port '%s'\033[0m\n", 
             driverName, deviceName_, functionName, CanPort );
    return;
  }

  // These Axis parameters will be initalized:
  epicsUInt8 addr[] = { 1,  // actual position
                        4,  // max positioning speed
                        5,  // max acceleration and deceleration
                        12, // right limit switch disable
                        13, // left limit switch disable
                        14, // switch mode
                        15, // stop deceleration
                        27  // microstep resolution
  };
  can_frame_t pframe;
  for ( int i = 0; i < 8; i++ ){
    // initialize axis and global parameters
    pframe.can_id = can_id_w_;
    pframe.can_dlc = 7;
    pframe.data[0] = 6; // Get axis parameter
    pframe.data[1] = addr[i];
    pframe.data[2] = 0;
    pframe.data[3] = 0;
    pframe.data[4] = 0;
    pframe.data[5] = 0;
    pframe.data[6] = 0;
    
    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, 0.5 );
    if ( asynTimeout == status ){
      fprintf( stderr, "%s:%s:%s: status=%d, function=%d, No reply from device within %f s\n", 
               driverName, deviceName_, functionName, status, P_GAP, 1. );
      return;
    }
    if ( status ){
      fprintf( stderr, "%s:%s:%s: status=%d, function=%d %s\n", 
               driverName, deviceName_, functionName, status, P_GAP,
               pAsynUserGenericPointer_->errorMessage );
      return;
    }
    if ( ( pframe.can_id  != can_id_r_ ) ||
         ( pframe.can_dlc != 7 ) ||
         ( pframe.data[0] != ( can_id_w_ & 0xff ) ) ||
         ( pframe.data[2] != 6 ) 
         ){
      fprintf( stderr, "\033[31;1m%s:%s:%s: Mismatch in reply.\nGot %08x %d %02x %02x %02x where %08x 7 %02x XX %02x was expected\033[0m\n", 
               driverName, deviceName_, functionName,
               pframe.can_id, pframe.can_dlc, pframe.data[0], pframe.data[1], pframe.data[2],
               can_id_r_, ( can_id_w_ & 0xff ), 6 );
      return;
    }
    // Update status
    // status = (asynStatus) setIntegerParam( 0, P_STATUS, pframe.data[1] );
    
    // update value of parameter
    split_t myValue;
    myValue.val8[3] = pframe.data[3];
    myValue.val8[2] = pframe.data[4];
    myValue.val8[1] = pframe.data[5];
    myValue.val8[0] = pframe.data[6];
    status = (asynStatus) setIntegerParam( addr[i], P_SAP, myValue.val32 );
    status = (asynStatus) setIntegerParam( addr[i], P_GAP, myValue.val32 );
  }

  cmds_w_.insert( std::make_pair( P_ROR, 1 ) );
  cmds_w_.insert( std::make_pair( P_ROL, 2 ) );
  cmds_w_.insert( std::make_pair( P_MST, 3 ) );
  cmds_w_.insert( std::make_pair( P_MVP, 4 ) );
  cmds_w_.insert( std::make_pair( P_SAP, 5 ) );
  cmds_r_.insert( std::make_pair( P_GAP, 6 ) );
  cmds_w_.insert( std::make_pair( P_STAP, 7 ) );
  cmds_w_.insert( std::make_pair( P_RSAP, 8 ) );
  cmds_w_.insert( std::make_pair( P_SGP, 9 ) );
  cmds_r_.insert( std::make_pair( P_GGP, 10 ) );
  cmds_w_.insert( std::make_pair( P_STGP, 11 ) );
  cmds_w_.insert( std::make_pair( P_RSGP, 12 ) );
  cmds_w_.insert( std::make_pair( P_RFS, 13 ) );
  cmds_w_.insert( std::make_pair( P_SIO, 14 ) );
  cmds_r_.insert( std::make_pair( P_GIO, 15 ) );
  cmds_w_.insert( std::make_pair( P_CALC, 19 ) );
  cmds_w_.insert( std::make_pair( P_COMP, 20 ) );
  cmds_w_.insert( std::make_pair( P_JC, 21 ) );
  cmds_w_.insert( std::make_pair( P_JA, 22 ) );
  cmds_w_.insert( std::make_pair( P_CSUB, 23 ) );
  cmds_w_.insert( std::make_pair( P_RSUB, 24 ) );
  cmds_w_.insert( std::make_pair( P_WAIT, 27 ) );
  cmds_w_.insert( std::make_pair( P_STOP, 28 ) );
  cmds_w_.insert( std::make_pair( P_SCO, 30 ) );
  cmds_r_.insert( std::make_pair( P_GCO, 31 ) );
  cmds_w_.insert( std::make_pair( P_CCO, 32 ) );
  cmds_w_.insert( std::make_pair( P_CALCX, 33 ) );
  cmds_w_.insert( std::make_pair( P_AAP, 34 ) );
  cmds_w_.insert( std::make_pair( P_AGP, 35 ) );
  cmds_w_.insert( std::make_pair( P_CLE, 36 ) );
  cmds_w_.insert( std::make_pair( P_ACO, 39 ) );

}

//******************************************************************************
//! EOF
//******************************************************************************
