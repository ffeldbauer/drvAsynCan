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
// brief   AsynPortDriver for PANDA Raspberry Pi CAN interface
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C includes
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
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

#ifdef USELIBSOCKETCAN
#  include "libsocketcan.h"
#endif

// local includes
#include "drvAsynCan.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef  struct can_frame     can_frame_t;
typedef  struct sockaddr_can  sockaddr_can_t;
typedef  struct sockaddr      sockaddr_t;
typedef  struct ifreq         ifreq_t; 

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynCanDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynGenericPointer->read().
//!
//!          Try to read a CAN frame from the kernel module.
//!          pasynUser->timeout is used as timeout in seconds.
//!          genericPointer should be of type 'struct can_frame'.
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//! @param   [out] genericPointer  Pointer to the object to read.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//!
//! @sa      drvAsynCan::drvRPiCanRead
//------------------------------------------------------------------------------
asynStatus drvAsynCan::readGenericPointer( asynUser *pasynUser, void *genericPointer ) {
  static const char *functionName = "readGenericPointer";
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
  can_frame_t* pframe = (can_frame_t *)genericPointer;

  int nbytes = 0;
  if ( 0 > mytimeout ) {

    nbytes = read( _socket, pframe, sizeof(can_frame_t) );
    if ( 0 > nbytes ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "Error receiving message from device '%s': %d %s", 
                     _deviceName, errno, strerror( errno ) );
      return asynError;
    }

  } else {

    fd_set fdRead;
    struct timeval t;
    
    // calculate timeout values
    t.tv_sec  = mytimeout / 1000000L;
    t.tv_usec = mytimeout % 1000000L;
    
    FD_ZERO( &fdRead );
    FD_SET( _socket, &fdRead );
    
    // wait until timeout or a message is ready to get read
    int err = select( _socket + 1, &fdRead, NULL, NULL, &t );
    
    // the only one file descriptor is ready for read
    if ( 0 < err ) {
      nbytes = read( _socket, pframe, sizeof(can_frame_t) );
      if ( 0 > nbytes ) {
        epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                       "Error receiving message from device '%s': %d %s", 
                       _deviceName, errno, strerror( errno ) );
        return asynError;
      }
    }
    
    // nothing is ready, timeout occured
    if ( 0 == err ) return asynTimeout;
    if ( 0 > err )  {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "Error receiving message from device '%s': %d %s", 
                     _deviceName, errno, strerror( errno ) );
      return asynError;
    }

  }
  
  memcpy( &_frame, pframe, sizeof( can_frame_t ) );
  doCallbacksGenericPointer( &_frame, _frame.can_id & CAN_EFF_MASK, 0 );
  
  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s: received frame '0x%08x %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x'\n", 
             driverName, functionName, pframe->can_id, pframe->can_dlc,
             pframe->data[0], pframe->data[1], pframe->data[2], pframe->data[3],
             pframe->data[4], pframe->data[5], pframe->data[6], pframe->data[7] );
  
  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynGenericPointer->write().
//!
//!          Write a CAN frame to the kernel module.
//!          pasynUser->timeout is used as timeout in seconds.
//!          genericPointer should be of type 'struct can_frame'
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//! @param   [in]  genericPointer  Pointer to the object to write.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError or asynTimeout is returned. A error message is stored
//!          in pasynUser->errorMessage.
//!
//! @sa      drvAsynCan::drvRPiCanWrite
//------------------------------------------------------------------------------
asynStatus drvAsynCan::writeGenericPointer( asynUser *pasynUser, void *genericPointer ) {
  static const char *functionName = "writeGenericPointer";
  can_frame_t *myFrame = (can_frame_t *)genericPointer;
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
  
  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s: sending frame '0x%08x %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x'\n", 
             driverName, functionName, myFrame->can_id, myFrame->can_dlc,
             myFrame->data[0], myFrame->data[1], myFrame->data[2], myFrame->data[3],
             myFrame->data[4], myFrame->data[5], myFrame->data[6], myFrame->data[7] );
  
  int nbytes = 0;
  if ( 0 > mytimeout ) {
    nbytes = write( _socket, myFrame, sizeof(can_frame_t) );
    if ( 0 > nbytes ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "Error receiving message from device '%s': %d %s", 
                     _deviceName, errno, strerror( errno ) );
      return asynError;
    }

  } else {
    
    fd_set fdWrite;
    struct timeval t;
    
    // calculate timeout values
    t.tv_sec  = mytimeout / 1000000L;
    t.tv_usec = mytimeout % 1000000L;
    
    FD_ZERO( &fdWrite );
    FD_SET( _socket, &fdWrite );
    
    // wait until timeout or a message is ready to get written
    int err = select( _socket + 1, NULL, &fdWrite, NULL, &t );
    
    // the only one file descriptor is ready for write
    if ( 0 < err ) {
      nbytes = write( _socket, myFrame, sizeof(can_frame_t) );
      if ( 0 > nbytes ) {
        epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                       "Error receiving message from device '%s': %d %s", 
                       _deviceName, errno, strerror( errno ) );
        return asynError;
      }
    }

    // nothing is ready, timeout occured
    if ( err == 0 ) return asynTimeout;
    if ( 0 > err )  {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                     "Error receiving message from device '%s': %d %s", 
                     _deviceName, errno, strerror( errno ) );
      return asynError;
    }

  }
  return asynSuccess;
}
//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynOption->read()
//!
//!          If key is equal to "bitrate" the bitrate settings of the CAN bus
//!          interface is read out.
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
asynStatus drvAsynCan::readOption( asynUser *pasynUser, const char *key,
                                   char *value, int maxChars ) {
  static const char *functionName = "readOption";

#ifdef USELIBSOCKETCAN

  if( epicsStrCaseCmp( key, "bitrate" ) == 0 ) {
    
    struct can_bittiming bt; 
    int err = can_get_bittiming( _deviceName, &bt );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: failed to read bittiming",
                     driverName, functionName );
      return asynError;
    }
    char dummy[10];
    sprintf( dummy, "%u", bt.bitrate );
    strcpy(value, dummy);
    return asynSuccess;
  }

#endif
   
  // unknown option
  epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                 "%s:%s: Invalid option key '%s'",
                 driverName, functionName, key );
  return asynError;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynOption->write()
//!
//!          If key is equal to "bitrate" the bitrate settings of the CAN bus
//!          interface is changed to value.
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//! @param   [in]  key             Name of option
//! @param   [in]  value           String containing the value for the option
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynCan::writeOption( asynUser *pasynUser, const char *key, const char *value ) {
  static const char *functionName = "writeOption";

#ifdef USELIBSOCKETCAN

  if( epicsStrCaseCmp( key, "bitrate" ) == 0 ) {
    // Change Bitrate
    epicsUInt32 bitrate;
    if( sscanf( value, "%d", &bitrate ) != 1 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "Bad number");
      return asynError;
    }
    
    int err = can_set_bitrate( _deviceName, bitrate );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: Could not change bitrate for interface '%s'.",
                     driverName, functionName, _deviceName );
      return asynError;
    }
  }

#endif
  
  // unknown option
  epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                 "%s:%s: Invalid option key '%s'",
                 driverName, functionName, key );
  return asynError;
}

//------------------------------------------------------------------------------
//! @brief   Connects driver to device
//!
//! @param   [in]  pasynUser       pasynUser structure which contains information about port and address.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynCan::connect( asynUser *pasynUser ) {
  int addr;
  asynStatus status;
  static const char *functionName = "connect";
    
  status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);
    
  pasynManager->exceptionConnect(pasynUser);
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s:, pasynUser=%p\n", 
            driverName, functionName, pasynUser);
  return(asynSuccess);

  sockaddr_can_t addr;
  ifreq_t ifr;

  // open socket
  _socket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
  if( _socket < 0 ) {
    perror( "Error while opening socket" );
    return asynError;
  }
  
  strcpy( ifr.ifr_name, _deviceName );
  ioctl( _socket, SIOCGIFINDEX, &ifr );
 
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
 
  if( bind( _socket, (sockaddr_t*)&addr, sizeof( addr ) ) < 0 ) {
    perror( "Error in socket bind" );
    return asynError;
  }

  pasynManager->exceptionConnect( pasynUser );
  asynPrint( pasynUser, ASYN_TRACE_FLOW,
             "%s:%s:, pasynUser=%p\n", 
             driverName, functionName, pasynUser);
  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   Disconnects driver to device
//!
//! @param   [in]  pasynUser       pasynUser structure which contains information about port and address.
//!
//! @return  in case of no error occured asynSuccess is returned. Otherwise
//!          asynError is returned. A error message is stored
//!          in pasynUser->errorMessage.
//------------------------------------------------------------------------------
asynStatus drvAsynCan::disconnect( asynUser *pasynUser ) {
  int addr;
  asynStatus status;
  static const char *functionName = "disconnect";
   
  status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);

  pasynManager->exceptionDisconnect(pasynUser);
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s:, pasynUser=%p\n", 
            driverName, functionName, pasynUser);
  return(asynSuccess);
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynCan class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName The name of the asynPortDriver to be created.
//! @param   [in]  ttyName  The name of the device
//------------------------------------------------------------------------------
drvAsynCan::drvAsynCan( const char *portName, const char *ttyName ) 
  : asynPortDriver( portName,
                    1, // maxAddr
                    0,
                    asynCommonMask | asynGenericPointerMask | asynOptionMask | asynDrvUserMask, // Interface mask
                    asynCommonMask | asynGenericPointerMask,  // Interrupt mask
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags
                    1,  // Autoconnect
                    0,  // Default priority
                    0 ) // Default stack size
{
  //const char *functionName = "drvAsynCan";
    
  _deviceName = epicsStrDup( ttyName );
  P_GENERIC = 1;

  sockaddr_can_t addr;
  ifreq_t ifr;

  // open socket
  _socket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
  if( _socket < 0 ) {
    perror( "Error while opening socket" );
    return;
  }
  
  strcpy( ifr.ifr_name, _deviceName );
  ioctl( _socket, SIOCGIFINDEX, &ifr );
 
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
 
  if( bind( _socket, (sockaddr_t*)&addr, sizeof( addr ) ) < 0 ) {
    perror( "Error in socket bind" );
    return;
  }

  // Set size of CAN socket send buffer to minimum
  int sndBufNew = 0;
  if( setsockopt( _socket, SOL_SOCKET, SO_SNDBUF, (void *)&sndBufNew, sizeof( sndBufNew ) ) < 0 ){
    perror( "Error while set socket option SNDBUF" );
    return;
  }
  // Print minimum value of buffer size
  //int sndBuf = 0;
  //unsigned int sndBuf_len = 4;
  //getsockopt(Socketfd, SOL_SOCKET, SO_SNDBUF, (void *)&sndBuf, &sndBuf_len);

}

