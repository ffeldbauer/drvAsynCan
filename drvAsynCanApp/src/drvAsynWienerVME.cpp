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
// brief   Asyn driver for Wiener VME crate remote control
//
// version 1.0.0; Nov. 27, 2012
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

#include "drvAsynWienerVME.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynWienerVMEDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->read().
//!          
//!          If pasynUser->reason is equal
//!          - P_Vmom0       the measured voltages and currents of channels 0
//!                          and 4 are read out and stored in the corresponding
//!                          parameters.
//!          - P_Vmom1       the measured voltages and currents of channels 1
//!                          and 5 are read out and stored in the corresponding
//!                          parameters.
//!          - P_Vmom2       the measured voltages and currents of channels 2
//!                          and 6 are read out and stored in the corresponding
//!                          parameters.
//!          - P_Vmom3       the measured voltages and currents of channels 3
//!                          and 7 are read out and stored in the corresponding
//!                          parameters.
//!          - P_FanMiddle   the measured fan speeds, middle fan speed, and
//!                          nominal fan speed are read out and stored in
//!                          the corresponding parameters.
//!          - P_Temp1       the measured temperatures are read out and stored in
//!                          the corresponding parameters.
//!          - any other:    the value from the parameter library will be returned.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "readInt32";

  std::map<int, vme_cmd_t>::const_iterator it = cmds_.find( function );
  if( it != cmds_.end() ) {
    can_frame_t pframe;
    pframe.can_id  = it->second.cmd | crate_id_ | CAN_RTR_FLAG;
    pframe.can_dlc = 8;
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
    if ( pframe.can_id != ( it->second.cmd | crate_id_ ) || pframe.can_dlc != 8 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d...' where '%x08 8...' was expected.\033[0m", 
                     driverName, deviceName_, functionName, function, pframe.can_id, pframe.can_dlc, it->second.cmd | crate_id_ );
      return asynError;
    }
    
    if ( it->second.num == 4 ) {
      status = setIntegerParam( it->second.params[0], ( pframe.data[1] << 8 ) | pframe.data[0] );
      status = setIntegerParam( it->second.params[1], ( pframe.data[3] << 8 ) | pframe.data[2] );
      status = setIntegerParam( it->second.params[2], ( pframe.data[5] << 8 ) | pframe.data[4] );
      status = setIntegerParam( it->second.params[3], ( pframe.data[7] << 8 ) | pframe.data[6] );
    } else {
      status = setIntegerParam( it->second.params[0], pframe.data[0] );
      status = setIntegerParam( it->second.params[1], pframe.data[1] );
      status = setIntegerParam( it->second.params[2], pframe.data[2] );
      status = setIntegerParam( it->second.params[3], pframe.data[3] );
      status = setIntegerParam( it->second.params[4], pframe.data[4] );
      status = setIntegerParam( it->second.params[5], pframe.data[5] );
      status = setIntegerParam( it->second.params[6], pframe.data[6] );
      status = setIntegerParam( it->second.params[7], pframe.data[7] );
    }    
    
    status = (asynStatus) callParamCallbacks();
  }
  
  // read back parameter
  status = (asynStatus) getIntegerParam( function, value );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%u\n", 
               driverName, deviceName_, functionName, function, *value );
  
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          If pasynUser->reason is equal to
//!          - P_FanSpeed  the nominal fan speed will be changed to value
//!          - P_sysreset  a system reset of the crate will be performed
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  if ( ( function != P_FanSpeed ) && ( function != P_Sysreset ) ) return asynSuccess;
  
  status = (asynStatus) setIntegerParam( function, value );
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, deviceName_, functionName, function, value );
  
  can_frame_t pframe;
  pframe.can_id  = ( 1 << 7 ) | crate_id_;
  if ( function == P_FanSpeed ) {
    pframe.can_dlc = 2;
    pframe.data[0] = 0xc0;
    pframe.data[1] = (epicsUInt8)( value & 0xff );
  } else if ( function == P_Sysreset ) {
    pframe.can_dlc = 1;
    pframe.data[0] = 0x44;
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
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!
//!          If pasynUser->reason is equal
//!          - P_Status0     the status bytes are read out and stored in
//!                          the corresponding parameters.
//!          - any other:    the value from the parameter library will be returned.
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [out] value      Address of the value to read
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "readUInt32Digital";

  if ( function == P_Status0 ) {
    can_frame_t pframe;
    pframe.can_id  = crate_id_ | CAN_RTR_FLAG;
    pframe.can_dlc = 8;
    
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
    if ( pframe.can_id != crate_id_ || pframe.can_dlc != 8 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\n Got '%08x %d...' where '%x08 8...' was expected.\033[0m", 
                     driverName, deviceName_, functionName, function, pframe.can_id, pframe.can_dlc, crate_id_  );
      return asynError;
    }

    status = setUIntDigitalParam( P_Status0, pframe.data[0], mask );
    status = setUIntDigitalParam( P_Status1, pframe.data[1], mask );
    status = setUIntDigitalParam( P_Status2, pframe.data[2], mask );
    status = setUIntDigitalParam( P_Status3, pframe.data[3], mask );
    status = setUIntDigitalParam( P_Status4, pframe.data[4], mask );
    status = setUIntDigitalParam( P_Status5, pframe.data[5], mask );
    status = setUIntDigitalParam( P_Status6, pframe.data[6], mask );
    status = setUIntDigitalParam( P_Status7, pframe.data[7], mask );
    
    status = (asynStatus) callParamCallbacks();
  }
  
  // read back parameter
  status = (asynStatus) getUIntDigitalParam( function, value, mask );
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
//!          If pasynUser->reason is equal to P_Switch this function
//!          switches the crate on and off, respectively
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//! @param   [in]  mask       Mask value to use when reading the value.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  if ( function != P_Switch ) return asynSuccess;

  /* Set the parameter in the parameter library. */
  status = (asynStatus) setUIntDigitalParam( function, value, mask );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d, mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, deviceName_, functionName, function, value, mask );

  can_frame_t pframe;
  pframe.can_id  = ( 1 << 7 ) | crate_id_;
  pframe.can_dlc = 1;
  pframe.data[0] = value ? 0x67 : 0x65;

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
//! @brief   Constructor for the drvAsynWienerVme class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asynPortDriver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  crate_id    The id of the crate
//------------------------------------------------------------------------------
drvAsynWienerVme::drvAsynWienerVme( const char *portName, const char *CanPort, const int crate_id ) 
  : asynPortDriver( portName, 
                    1, /* maxAddr */ 
                    NUM_WIENERVME_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynWienerVme";
  asynStatus status = asynSuccess;

  deviceName_  = epicsStrDup( portName );
  crate_id_    = crate_id;
  
  // Create parameters
  createParam( P_WIENERVME_STATUS0_STRING,    asynParamUInt32Digital, &P_Status0 );
  createParam( P_WIENERVME_STATUS1_STRING,    asynParamUInt32Digital, &P_Status1 );
  createParam( P_WIENERVME_STATUS2_STRING,    asynParamUInt32Digital, &P_Status2 );
  createParam( P_WIENERVME_STATUS3_STRING,    asynParamUInt32Digital, &P_Status3 );
  createParam( P_WIENERVME_STATUS4_STRING,    asynParamUInt32Digital, &P_Status4 );
  createParam( P_WIENERVME_STATUS5_STRING,    asynParamUInt32Digital, &P_Status5 );
  createParam( P_WIENERVME_STATUS6_STRING,    asynParamUInt32Digital, &P_Status6 );
  createParam( P_WIENERVME_STATUS7_STRING,    asynParamUInt32Digital, &P_Status7 );
  createParam( P_WIENERVME_V0_STRING,         asynParamInt32,         &P_Vmom0 );
  createParam( P_WIENERVME_V1_STRING,         asynParamInt32,         &P_Vmom1 );
  createParam( P_WIENERVME_V2_STRING,         asynParamInt32,         &P_Vmom2 );
  createParam( P_WIENERVME_V3_STRING,         asynParamInt32,         &P_Vmom3 );
  createParam( P_WIENERVME_V4_STRING,         asynParamInt32,         &P_Vmom4 );
  createParam( P_WIENERVME_V5_STRING,         asynParamInt32,         &P_Vmom5 );
  createParam( P_WIENERVME_V6_STRING,         asynParamInt32,         &P_Vmom6 );
  createParam( P_WIENERVME_V7_STRING,         asynParamInt32,         &P_Vmom7 );
  createParam( P_WIENERVME_I0_STRING,         asynParamInt32,         &P_Imom0 );
  createParam( P_WIENERVME_I1_STRING,         asynParamInt32,         &P_Imom1 );
  createParam( P_WIENERVME_I2_STRING,         asynParamInt32,         &P_Imom2 );
  createParam( P_WIENERVME_I3_STRING,         asynParamInt32,         &P_Imom3 );
  createParam( P_WIENERVME_I4_STRING,         asynParamInt32,         &P_Imom4 );
  createParam( P_WIENERVME_I5_STRING,         asynParamInt32,         &P_Imom5 );
  createParam( P_WIENERVME_I6_STRING,         asynParamInt32,         &P_Imom6 );
  createParam( P_WIENERVME_I7_STRING,         asynParamInt32,         &P_Imom7 );
  createParam( P_WIENERVME_FANMIDDLE_STRING,  asynParamInt32,         &P_FanMiddle );
  createParam( P_WIENERVME_FANNOMINAL_STRING, asynParamInt32,         &P_FanNominal );
  createParam( P_WIENERVME_FAN1_STRING,       asynParamInt32,         &P_Fan1 );
  createParam( P_WIENERVME_FAN2_STRING,       asynParamInt32,         &P_Fan2 );
  createParam( P_WIENERVME_FAN3_STRING,       asynParamInt32,         &P_Fan3 );
  createParam( P_WIENERVME_FAN4_STRING,       asynParamInt32,         &P_Fan4 );
  createParam( P_WIENERVME_FAN5_STRING,       asynParamInt32,         &P_Fan5 );
  createParam( P_WIENERVME_FAN6_STRING,       asynParamInt32,         &P_Fan6 );
  createParam( P_WIENERVME_TEMP1_STRING,      asynParamInt32,         &P_Temp1 );
  createParam( P_WIENERVME_TEMP2_STRING,      asynParamInt32,         &P_Temp2 );
  createParam( P_WIENERVME_TEMP3_STRING,      asynParamInt32,         &P_Temp3 );
  createParam( P_WIENERVME_TEMP4_STRING,      asynParamInt32,         &P_Temp4 );
  createParam( P_WIENERVME_TEMP5_STRING,      asynParamInt32,         &P_Temp5 );
  createParam( P_WIENERVME_TEMP6_STRING,      asynParamInt32,         &P_Temp6 );
  createParam( P_WIENERVME_TEMP7_STRING,      asynParamInt32,         &P_Temp7 );
  createParam( P_WIENERVME_TEMP8_STRING,      asynParamInt32,         &P_Temp8 );
  createParam( P_WIENERVME_SWITCH_STRING,     asynParamUInt32Digital, &P_Switch );
  createParam( P_WIENERVME_SYSRESET_STRING,   asynParamInt32,         &P_Sysreset );
  createParam( P_WIENERVME_CHANGEFAN_STRING,  asynParamInt32,         &P_FanSpeed );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( CanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    printf( "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
            driverName, deviceName_, functionName, CanPort );
  }
  
  // Get inital value for Switch-Parameter
  can_frame_t pframe;
  pframe.can_id  = crate_id_ | CAN_RTR_FLAG;
  pframe.can_dlc = 1;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, .5 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: Init %s: No reply from device within 1 s!\033[0m\n",
             driverName, deviceName_, functionName, P_WIENERVME_SWITCH_STRING );
    return;
  }
  setUIntDigitalParam( P_Switch, pframe.data[0] & 0x01, 1 );

  // initial value for nominal fan speed
  pframe.can_id  = crate_id_ | ( 6 << 7 ) | CAN_RTR_FLAG;
  pframe.can_dlc = 2;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, &pframe, &pframe, .5 );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: Init %s: No reply from device within 1 s!\033[0m\n",
             driverName, deviceName_, functionName, P_WIENERVME_CHANGEFAN_STRING );
    return;
  }
  setIntegerParam( P_FanSpeed, pframe.data[1] );

  vme_cmd_t vc04_cmd = { ( 2 << 7 ), 4, { P_Vmom0, P_Imom0, P_Vmom4, P_Imom4 } };
  cmds_.insert( std::make_pair( P_Vmom0, vc04_cmd ) );
  vme_cmd_t vc15_cmd = { ( 3 << 7 ), 4, { P_Vmom1, P_Imom1, P_Vmom5, P_Imom5 } };
  cmds_.insert( std::make_pair( P_Vmom1, vc15_cmd ) );
  vme_cmd_t vc26_cmd = { ( 4 << 7 ), 4, { P_Vmom2, P_Imom2, P_Vmom6, P_Imom6 } };
  cmds_.insert( std::make_pair( P_Vmom2, vc26_cmd ) );
  vme_cmd_t vc37_cmd = { ( 5 << 7 ), 4, { P_Vmom3, P_Imom3, P_Vmom7, P_Imom7 } };
  cmds_.insert( std::make_pair( P_Vmom3, vc37_cmd ) );
  vme_cmd_t fan_cmd = { ( 6 << 7 ), 8, { P_FanMiddle, P_FanNominal, P_Fan1,
                                         P_Fan2, P_Fan3, P_Fan4, P_Fan5, P_Fan6 } };
  cmds_.insert( std::make_pair( P_FanMiddle, fan_cmd ) );
  vme_cmd_t temp_cmd = { ( 7 << 7 ), 8, { P_Temp1, P_Temp2, P_Temp3, P_Temp4,
                                          P_Temp5, P_Temp6, P_Temp7, P_Temp8 } };
  cmds_.insert( std::make_pair( P_Temp1, temp_cmd ) );

}

//******************************************************************************
//! EOF
//******************************************************************************
