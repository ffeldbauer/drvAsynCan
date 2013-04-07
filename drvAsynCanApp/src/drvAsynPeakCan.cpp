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
//! @brief   AsynPortDriver for PANDA Raspberry Pi CAN interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>

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

#include "drvAsynPeakCan.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef struct can_frame can_frame_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynPeakCanDriver";

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
  const char* functionName = "readGenericPointer";
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
  can_frame_t* pframe = (can_frame_t *)genericPointer;
  TPCANRdMsg rdmsg;

  int err = drvPeakCanRead( &rdmsg, mytimeout );
  
  if ( CAN_ERR_QRCVEMPTY == err )  return asynTimeout;

  if ( 0 != err ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "Error receiving message from device '%s': %d %s", 
                   deviceName_, err, strerror( err ) );
    return asynError;
  }  

  // convert PEAK msg to can_frame
  pframe->can_id = rdmsg.Msg.ID;
  if( rdmsg.Msg.MSGTYPE & MSGTYPE_RTR )       pframe->can_id |= CAN_RTR_FLAG;
  if( rdmsg.Msg.MSGTYPE & MSGTYPE_EXTENDED )  pframe->can_id |= CAN_EFF_FLAG;
  pframe->can_dlc = rdmsg.Msg.LEN;
  for ( int i = 0; i < 8; i++ ){
    pframe->data[i] = rdmsg.Msg.DATA[i];
  }

  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s: received frame '%08x %d %02x %02x %02x %02x %02x %02x %02x %02x'\n", 
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
  const char* functionName = "writeGenericPointer";
  can_frame_t *myFrame = (can_frame_t *)genericPointer;
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
 
  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s: sending frame '%08x %d %02x %02x %02x %02x %02x %02x %02x %02x'\n", 
             driverName, functionName, myFrame->can_id, myFrame->can_dlc,
             myFrame->data[0], myFrame->data[1], myFrame->data[2], myFrame->data[3],
             myFrame->data[4], myFrame->data[5], myFrame->data[6], myFrame->data[7] );


  // Convert can_frame to PEAK msg
  TPCANMsg msg;
  if( myFrame->can_id & CAN_RTR_FLAG )    msg.MSGTYPE = MSGTYPE_RTR;
  else                                    msg.MSGTYPE = MSGTYPE_STANDARD;
  if( myFrame->can_id & CAN_EFF_FLAG )    msg.MSGTYPE |= MSGTYPE_EXTENDED;
  msg.ID  = myFrame->can_id & CAN_EFF_MASK; // remove EFF/RTR/ERR flags
  msg.LEN = myFrame->can_dlc;
  for ( int i = 0; i < 8; i++ ){
    msg.DATA[i] = myFrame->data[i];
  }

  int err = drvPeakCanWrite( &msg, mytimeout );
  if ( 0 != err ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "Error sending message to device '%s': %s", 
                   deviceName_, strerror( err ) );
    return asynError;
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
  const char* functionName = "readOption";

  if( epicsStrCaseCmp( key, "bitrate" ) == 0 ) {
    strcpy( value, "Not supported by kernel module..." );
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
  const char* functionName = "writeOption";

  if( epicsStrCaseCmp( key, "bitrate" ) == 0 ) {
    // Change Bitrate
    epicsUInt32 bitrate;
    if( sscanf( value, "%u", &bitrate ) != 1 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "Bad number");
      return asynError;
    }
    
    TPBTR0BTR1 ratix = { bitrate, 0 };
    int err = ioctl( fd_, PCAN_BTR0BTR1, &ratix );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: Could not change bitrate for interface '%s'. %s",
                     driverName, functionName, deviceName_, strerror( err ) );
      return asynError;
    }
    TPCANInit myInit = { ratix.wBTR0BTR1, 
                         MSGTYPE_EXTENDED,
                         0 };
    err = ioctl( fd_, PCAN_INIT, &myInit );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: Could not change bitrate for interface '%s'. %s",
                     driverName, functionName, deviceName_, strerror( err ) );
      return asynError;
    }

  } else if ( epicsStrCaseCmp( key, "addfilter" ) == 0 ) {
    // Add new filter
    epicsUInt32 FromID = 0;
    epicsUInt32 ToID = 0;
    epicsUInt8  MSGTYPE = 0;
    if( sscanf( value, "%x:%x:%c", &FromID, &ToID, &MSGTYPE ) != 3 ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "Bad value");
      return asynError;
    }
    TPMSGFILTER filter = { FromID, ToID, MSGTYPE };
    int err = ioctl( fd_, PCAN_MSG_FILTER, &filter );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: Could not add filter for interface '%s'. %s",
                     driverName, functionName, deviceName_, strerror( err ) );
      return asynError;
    }

  } else if ( epicsStrCaseCmp( key, "delfilter" ) == 0 ) {
    // delete all existing filters
    int err = ioctl( fd_, PCAN_MSG_FILTER, NULL );
    if ( err ) {
      epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s:%s: Could not delete filters for interface '%s'. %s",
                     driverName, functionName, deviceName_, strerror( err ) );
      return asynError;
    }
  
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
//! @brief   This functions is the actual interface to the hardware for sending
//!          CAN frames
//!
//!          Using the PANDA Raspberry Pi CAN Extension Board this function
//!          uses ioctl funciton to access the kernel module
//!
//! @param   [in]  pframe     CAN frame to send
//! @param   [in]  timeout    timeout in microseconds
//!
//! @return  In case of no error occured 0 is returned. In case of a timeout
//!          CAN_ERR_QXMTFULL is returned. Otherwise ERRNO is returned
//------------------------------------------------------------------------------
int drvAsynCan::drvPeakCanWrite( TPCANMsg *pframe, int timeout ){

  if ( timeout < 0)
    return ioctl( fd_, PCAN_WRITE_MSG, pframe );
  
  fd_set fdWrite;
  struct timeval t;
  
  // calculate timeout values
  t.tv_sec  = timeout / 1000000L;
  t.tv_usec = timeout % 1000000L;
  
  FD_ZERO( &fdWrite );
  FD_SET( fd_, &fdWrite );
  
  // wait until timeout or a message is ready to get written
  int err = select( fd_ + 1, NULL, &fdWrite, NULL, &t );
  
  // the only one file descriptor is ready for write
  if ( err  > 0 )
    return ioctl( fd_, PCAN_WRITE_MSG, pframe );
  
  // nothing is ready, timeout occured
  if ( err == 0 )
    return CAN_ERR_QXMTFULL;
  return err;
}

//------------------------------------------------------------------------------
//! @brief   This functions is the actual interface to the hardware for reading
//!          CAN frames
//!
//!          Using the PANDA Raspberry Pi CAN Extension Board this function
//!          uses ioctl funciton to access the kernel module
//!
//! @param   [out] pframe     CAN frame read
//! @param   [in]  timeout    timeout in microseconds
//!
//! @return  In case of no error occured 0 is returned. In case of a timeout
//!          CAN_ERR_QRCVEMPTY is returned. Otherwise ERRNO is returned
//------------------------------------------------------------------------------
int drvAsynCan::drvPeakCanRead( TPCANRdMsg *pframe, int timeout ){
  if ( timeout < 0)
    return ioctl( fd_, PCAN_READ_MSG, pframe );

  fd_set fdRead;
  struct timeval t;
  
  // calculate timeout values
  t.tv_sec  = timeout / 1000000L;
  t.tv_usec = timeout % 1000000L;
  
  FD_ZERO( &fdRead );
  FD_SET( fd_, &fdRead );
  
  // wait until timeout or a message is ready to get read
  int err = select( fd_ + 1, &fdRead, NULL, NULL, &t );
  
  // the only one file descriptor is ready for read
  if ( err  > 0 )
    return ioctl( fd_, PCAN_READ_MSG, pframe );
  
  // nothing is ready, timeout occured
  if ( err == 0 )
    return CAN_ERR_QRCVEMPTY;
  return err;
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
                    1, /* maxAddr */ 
                    0,
                    asynCommonMask | asynGenericPointerMask | asynOptionMask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynGenericPointerMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynCan";
    
  deviceName_ = epicsStrDup( ttyName );
  
  // open interface
  fd_ = open( deviceName_, O_RDWR );
  if ( 0 > fd_ ){
    fprintf( stderr, "\033[31;1m %s:%s: Could not open interface '%s'. %s \033[0m \n",
             driverName, functionName, deviceName_, strerror( errno ) );
    return;
  }
      
}

/* Configuration routines.  Called directly, or from the iocsh function below */
extern "C" {
  
  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynCan class.
  //!
  //! @param   [in]  portName The name of the asyn port driver to be created.
  //!          [in]  ttyName  The name of the interface 
  //----------------------------------------------------------------------------
  int drvAsynCanConfigure( const char *portName, const char *ttyName ) {
    new drvAsynCan( portName, ttyName );
    return( asynSuccess );
  }
  static const iocshArg initPeakCanArg0 = { "portName", iocshArgString };
  static const iocshArg initPeakCanArg1 = { "ttyName",  iocshArgString };
  static const iocshArg * const initPeakCanArgs[] = { &initPeakCanArg0, &initPeakCanArg1 };
  static const iocshFuncDef initPeakCanFuncDef = { "drvAsynCanConfigure", 2, initPeakCanArgs };
  static void initPeakCanCallFunc( const iocshArgBuf *args ) {
    drvAsynCanConfigure( args[0].sval, args[1].sval );
  }
  
  //----------------------------------------------------------------------------
  //! @brief   Register functions to EPICS
  //----------------------------------------------------------------------------
  void drvAsynCanDrvRegister( void ) {
    static int firstTime = 1;
    if ( firstTime ) {
      iocshRegister( &initPeakCanFuncDef, initPeakCanCallFunc );
      firstTime = 0;
    }
  }
  
  epicsExportRegistrar( drvAsynCanDrvRegister );
}

//******************************************************************************
//! EOF
//******************************************************************************
