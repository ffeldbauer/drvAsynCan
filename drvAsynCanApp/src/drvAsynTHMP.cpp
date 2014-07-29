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

#include "drvAsynTHMP.h"
#include "ReadPoller.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynTHMPDriver";

//_____ F U N C T I O N S ______________________________________________________
static void myInterruptCallbackGenericPointer( void *userPvt,
                                               asynUser *pasynUser,
                                               void *pointer ) {
  drvAsynTHMP* interface = static_cast<drvAsynTHMP*>( pasynUser->userPvt );
  interface->asynReadHandler( pointer );
}

void drvAsynTHMP::asynReadHandler( void* pointer ) {
  asynStatus status = asynSuccess;
  //  static const char *functionName = "asynReadHandler";
  can_frame_t* pframe = (can_frame_t *)pointer;

  switch ( pframe->data[0] ) {
    
  case 0x01: // ADC Conversion
    if ( pframe->can_dlc != 4 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid data length of frame for command 0x01: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    if ( pframe->data[1] >= 64 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid channel number for command 0x01: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    // epicsInt32 myValue = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
    status = (asynStatus) setIntegerParam( pframe->data[1], P_RawValue,
                                           ( pframe->data[2] << 8 ) | ( pframe->data[3]) );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": status=" << status << ", function=" << P_RawValue << ", value="
                << ( ( pframe->data[2] << 8 ) | ( pframe->data[3] ) )
                << "\033[0m"
                << std::endl;

    callParamCallbacks( pframe->data[1], pframe->data[1] );
    break;
    
  case 0x03: // I/O Board
    if ( pframe->can_dlc != 4 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid data length of frame for command 0x03: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    if ( pframe->data[1] >= 2 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid channel number for command 0x03: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    // epicsUInt32 myValue = ( pframe->data[3] << 8 ) | ( pframe->data[4]);
    status = (asynStatus) setUIntDigitalParam( pframe->data[1], P_IoBoard,
                                               ( pframe->data[2] << 8 ) | ( pframe->data[3]),
                                               0xffff );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": status=" << status << ", function=" << P_IoBoard << ", value="
                << ( ( pframe->data[2] << 8 ) | ( pframe->data[3] ) )
                << "\033[0m"
                << std::endl;
    callParamCallbacks( pframe->data[1], pframe->data[1] );
    break;
    
  case 0x04: // Serials
    if ( pframe->can_dlc != 8 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid data length of frame for command 0x04: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    if ( pframe->data[1] >= 9 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid channel number for command 0x04: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
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
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": status=" << status << ", function=" << P_Serials << ", value="
                << ( ( pframe->data[2] << 16 ) | ( pframe->data[3] << 8 ) | ( pframe->data[4] ) )
                << "\033[0m"
                << std::endl;
    callParamCallbacks( pframe->data[1], pframe->data[1] );
    break;
    
  case 0xff: // Firmware
    if ( pframe->can_dlc != 4 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid data length of frame for command 0xff: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    //epicsUInt32 myValue  = ( pframe->data[2] << 8 ) | ( pframe->data[3]);
    status = (asynStatus) setUIntDigitalParam( 0, P_Firmware,
                                               ( pframe->data[1] * 10000 ) + ( pframe->data[2] * 100 ) + ( pframe->data[3] ),
                                               0xffff );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": status=" << status << ", function=" << P_IoBoard << ", value="
                << pframe->data[1] << "." << pframe->data[2] << "." << pframe->data[3] 
                << "\033[0m"
                << std::endl;
    callParamCallbacks( 0, 0 );
    break;
    
  case 0xe0: // Error message
    if ( pframe->can_dlc != 3 ) {
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": invalid data length of frame for command 0xe0: " << pframe->can_dlc
                << "\n" << *pframe
                << "\033[0m"
                << std::endl;
      break;
    }
    //epicsUInt32 myValue  = ( pframe->data[2] << 2 ) | ( pframe->data[3]);
    status = (asynStatus) setUIntDigitalParam( 0, P_Error,
                                               ( pframe->data[1] << 8 ) | ( pframe->data[2]),
                                               0xffff );
    if( status ) 
      std::cerr << "\033[31;1m" << printTimestamp() << " "
                << driverName << ":" <<  _deviceName << ":asynReadHandler"
                << ": status=" << status << ", function=" << P_Error << ", value="
                << ( ( pframe->data[1] << 8 ) | ( pframe->data[2] ) )
                << "\033[0m"
                << std::endl;
    callParamCallbacks( 0, 0 );
    break;
  }
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
  static const char *functionName = "writeInt32";

  if ( function == P_RawValue ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  
  can_frame_t pframe;
  pframe.can_id = _can_id;
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

  _pasynUser->timeout = pasynUser->timeout;
  pasynManager->lockPort( _pasynUser );
  status = _pasynGenericPointer->write( _pvtPointerGeneric, _pasynUser, &pframe );
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
  static const char *functionName = "writeUInt32Digital";

  if ( function != P_IoBoard ) return asynSuccess;
  
  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( addr > 1 ) return asynSuccess;
  
  // Set the parameter in the parameter library.
  status = (asynStatus) setUIntDigitalParam( addr, function, value, mask );
  
  // Do callbacks so higher layers see any changes 
  status = (asynStatus) callParamCallbacks( addr, addr );
  
  if (status) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, _deviceName, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, _deviceName, functionName, function, value, mask );
  
  can_frame_t pframe;
  pframe.can_id = _can_id;
  pframe.can_dlc = 4;
  pframe.data[0] = 0x02;
  pframe.data[1] = (epicsUInt8)( addr & 0xff );
  pframe.data[2] = (epicsUInt8)( value >> 8 );
  pframe.data[3] = (epicsUInt8)( value & 0xff );
  
  _pasynUser->timeout = pasynUser->timeout;
  pasynManager->lockPort( _pasynUser );
  status = _pasynGenericPointer->write( _pvtPointerGeneric, _pasynUser, &pframe );
  pasynManager->unlockPort( _pasynUser );
  
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
                    64, // maxAddr 
                    NUM_THMP_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask,  // Interrupt mask 
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
                    1, // Autoconnect
                    0, // Default priority
                    0 ) // Default stack size
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
  _pvtPointerGeneric   = pasynInterface->drvPvt;
  _pasynUser->reason = _can_id;
  status = _pasynGenericPointer->registerInterruptUser( _pvtPointerGeneric,
                                                        _pasynUser,
                                                        myInterruptCallbackGenericPointer,
                                                        this,
                                                        &_intrPvtPointerGeneric
                                                        );
  if( asynSuccess != status  ) {
    std::cerr << driverName << ":" <<  _deviceName << ":" << functionName
              << ": failed to register interrupt"
              << std::endl;
    return;
  }
  // Start polling
  ReadPoller::create( CanPort );
}

//******************************************************************************
//! EOF
//******************************************************************************
