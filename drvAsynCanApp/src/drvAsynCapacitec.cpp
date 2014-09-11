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
// brief   Asyn driver for Capacitec Digitizer Board
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>

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

#include "drvAsynCapacitec.h"
#include "ReadPoller.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef union{
  epicsInt32   val32;
  epicsUInt8   val8[4];
} can_conv_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynCapacitecDriver";

//_____ F U N C T I O N S ______________________________________________________

static void myInterruptCallbackGenericPointer( void *userPvt,
                                               asynUser *pasynUser,
                                               void *pointer ) {
  drvAsynCapacitec* interface = static_cast<drvAsynCapacitec*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

void drvAsynCapacitec::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  can_frame_t* pframe = (can_frame_t *)pointer;
  can_conv_t myValue;

  switch ( pframe->data[0] ) {
    
  case 0x01: // ADC Conversion
    if ( pframe->can_dlc != 6 ) {
      std::cerr << "\033[31;1m" << printTimestamp()
                << driverName << ":" << _deviceName << ":asynReadHandler: "
                << "invalid data length of frame for command 0x01: "
                << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m" << std::endl;
      break;
    }
    if ( pframe->data[1] >= 8 ) {
      std::cerr << "\033[31;1m" << printTimestamp()
                << driverName << ":" << _deviceName << ":asynReadHandler: "
                << "invalid channel number for command 0x01: "
                << pframe->data[1]
                << "\n" << *pframe
                << "\033[0m" << std::endl;
      break;
    } 
    myValue.val8[3] = pframe->data[2];
    myValue.val8[2] = pframe->data[3];
    myValue.val8[1] = pframe->data[4];
    myValue.val8[0] = pframe->data[5];

    status = (asynStatus) setIntegerParam( pframe->data[1], P_RawValue, myValue.val32 );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp()
                << driverName << ":" << _deviceName << ":asynReadHandler: "
                << "status=" << status << ", function=" << P_RawValue << ". value=" << myValue.val32
                << "\033[0m" << std::endl;
    else 
      asynPrint( _pasynUser, ASYN_TRACEIO_DRIVER, 
                 "%s:%s:asynReadHandler: function=%d, value=%d (%02x%02x%02x%02x)\n", 
                 driverName, _deviceName, P_RawValue, myValue.val32,
                 myValue.val8[3], myValue.val8[2], myValue.val8[1], myValue.val8[0] );
    
    callParamCallbacks( pframe->data[1], pframe->data[1] );
    break;
        
  case 0xe0: // Error message
    if ( pframe->can_dlc != 3 ) {
      std::cerr << "\033[31;1m" << printTimestamp()
                << driverName << ":" << _deviceName << ":asynReadHandler: "
                << "invalid data length of frame for command 0xe0: "
                << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m" << std::endl;
      break;
    }
    myValue.val8[3] = 0;
    myValue.val8[2] = 0;
    myValue.val8[1] = pframe->data[1];
    myValue.val8[0] = pframe->data[2];

    status = (asynStatus) setUIntDigitalParam( 0, P_Error, myValue.val32, 0xffff );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp()
                << driverName << ":" << _deviceName << ":asynReadHandler: "
                << "status=" << status << ", function=" << P_Error << ". value=" << myValue.val32
                << "\033[0m" << std::endl;
    callParamCallbacks( 0, 0 );
    break;
  }
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!
//!          Trigger read out raw values respectively
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//! @param   [in]  value      Value to write
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynCapacitec::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeInt32";
  
  if ( function == P_RawValue ) return asynSuccess;
  
  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  
  can_frame_t pframe;
  pframe.can_id = _can_id;
  if ( function == P_Trg_ADC ) {
    pframe.can_dlc = 1;
    pframe.data[0] = 0x01;
  } else if ( function == P_OS ) {
    pframe.can_dlc = 2;
    pframe.data[0] = 0xc0;
    pframe.data[1] = (epicsUInt8)(value & 0xff);
  } else {
    return asynError;
  }
  
  _pasynUser->timeout = pasynUser->timeout;
  pasynManager->lockPort( _pasynUser );
  status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
  pasynManager->unlockPort( _pasynUser );
  
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Could not send can frame.\033[0m", 
                   driverName, _deviceName, functionName, function );
    return asynError;
  }
  
  return status;
}

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
asynStatus drvAsynCapacitec::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  asynStatus unlockStatus = asynSuccess;
  static const char *functionName = "readInt32";
    
  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  if ( function == P_OS ) {
    can_frame_t pframe;
    pframe.can_id = _can_id;
    pframe.can_dlc = 1;
    pframe.data[0] = 0xc1;

    _pasynUser->timeout = pasynUser->timeout;
    status = pasynManager->queueLockPort( _pasynUser );
    if( asynSuccess != status) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueLockPort: status=%d, function=%d", 
                     driverName, _deviceName, functionName, status, function );
      return status;
    }
    status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
    if ( asynSuccess != status ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynGenericPointer->write: status=%d, function=%d", 
                     driverName, _deviceName, functionName, status, function );
      return status;
    } 
    status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &pframe );
    unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
    if( asynSuccess != unlockStatus ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: pasynManager->queueUnlockPort: status=%d, function=%d", 
                     driverName, _deviceName, functionName, status, function );
      return unlockStatus;
    }
    
    if ( asynTimeout == status ){
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "%s:%s:%s: status=%d, function=%d, No reply from device within %f s", 
                     driverName, _deviceName, functionName, status, function, pasynUser->timeout );
      return asynTimeout;
    }

    status = setIntegerParam( addr, function, pframe.data[1] );
  }

  status = (asynStatus) getIntegerParam( addr, function, value );
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%d", 
                   driverName, functionName, status, function, *value );
  else        
    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, *value);
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynCapacitec class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//! @param   [in]  CanPort     The name of the asynPortDriver of the CAN bus interface 
//! @param   [in]  can_id      The CAN id of this Digitizer Board
//------------------------------------------------------------------------------
drvAsynCapacitec::drvAsynCapacitec( const char *portName, const char *CanPort,
                                    const int can_id ) 
  : asynPortDriver( portName, 
                    8, // maxAddr 
                    NUM_CAPACITEC_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask,  // Interrupt mask 
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
                    1, // Autoconnect
                    0, // Default priority
                    0 ) // Default stack size
{
  asynStatus status;
  const char *functionName = "drvAsynCapacitec";

  // Create parameters
  createParam( P_CAPACITEC_ADCBUFFER_STRING,     asynParamInt32,         &P_RawValue );
  createParam( P_CAPACITEC_ERROR_STRING,         asynParamUInt32Digital, &P_Error );
  createParam( P_CAPACITEC_TRG_ADCBUFFER_STRING, asynParamInt32,         &P_Trg_ADC );
  createParam( P_CAPACITEC_OS_STRING,            asynParamInt32,         &P_OS );
  
  _deviceName  = epicsStrDup( portName );
  _can_id      = can_id;
  
  // Connect to asyn generic pointer port with asynGenericPointer interface
  _pasynUser = pasynManager->createAsynUser( NULL, NULL );
  _pasynUser->userPvt = this;

  status = pasynManager->connectDevice( _pasynUser, CanPort, 0 );
  if ( asynSuccess != status ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": Unable to connect Device"
              << std::endl;
    return;
  }
    
  asynInterface* pasynInterface;
    
  // find the asynCommon interface
  pasynInterface = pasynManager->findInterface( _pasynUser,
                                                asynCommonType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": bus " << CanPort << " does not support asynCommon interface"
              << std::endl;
    return;
  }
  _pasynCommon = static_cast<asynCommon*>( pasynInterface->pinterface );
  _pvtCommon   = pasynInterface->drvPvt;
  
  // find the asynGenericPointer interface
  pasynInterface = pasynManager->findInterface( _pasynUser,
                                                asynGenericPointerType,
                                                true );
  if( !pasynInterface ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": bus " << CanPort << " does not support asynGenericPointer interface"
              << std::endl;
    return;
  }
  _pasynGenericPointer = static_cast<asynGenericPointer*>( pasynInterface->pinterface );
  _pvtGenericPointer   = pasynInterface->drvPvt;
  _pasynUser->reason = _can_id;
  status = _pasynGenericPointer->registerInterruptUser( _pvtGenericPointer,
                                                        _pasynUser,
                                                        myInterruptCallbackGenericPointer,
                                                        this,
                                                        &_intrPvtGenericPointer
                                                        );
  if( asynSuccess != status  ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": failed to register interrupt"
              << std::endl;
    return;
  }

  { // Initialize P_OS
    can_frame_t pframe;
    pframe.can_id = _can_id;
    pframe.can_dlc = 1;
    pframe.data[0] = 0xc1;

    _pasynUser->timeout = 1.;
    status = pasynManager->queueLockPort( _pasynUser );
    if( asynSuccess != status) {
      std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
                << ": pasynManager->queueLockPort: status=" << status
                << std::endl;
      return;
    }
    status = _pasynGenericPointer->write( _pvtGenericPointer, _pasynUser, &pframe );
    if ( asynSuccess != status ) {
      std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
                << ": pasynGenericPointer->write: status=" << status
                << std::endl;
      return;
    } 
    status = _pasynGenericPointer->read( _pvtGenericPointer, _pasynUser, &pframe );
    asynStatus unlockStatus = pasynManager->queueUnlockPort( _pasynUser );
    if( asynSuccess != unlockStatus ) {
      std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
                << ": pasynManager->queueUnlockPort: status=" << status
                << std::endl;
      return;
    }
    
    if ( asynTimeout == status ){
      std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
                << "No reply from device within 1 s"
                << std::endl;
      return;
    }

    status = setIntegerParam( 0, P_OS, pframe.data[1] );
  }

  // Start polling
  ReadPoller::create( CanPort );
}

//******************************************************************************
//! EOF
//******************************************************************************
