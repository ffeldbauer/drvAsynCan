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
//! @brief   Asyn port driver for Peak Systems CAN interfaces
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

#ifndef __ASYN_PEAK_CAN_H__
#define __ASYN_PEAK_CAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <pcan.h>

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
//#define P_GENERIC_String          "RPICAN_FRAME"        /* asynGenericPointer, r/w */
//#define P_BITRATE_String          "RPICAN_BITRATE"      /* asynOption, r/w */


//! @brief   asynPortDriver for PANDA Raspberry Pi CAN interface
//!
//! This is a lower lever asynPortDriver for communication with
//! the hardware CAN bus interface.\n
//! It provides the asynGenericPointer interface to higher level
//! asynPortDrivers.\n
//! The genericPointers should be of type struct can_frame.
class drvAsynRPiCan : public asynPortDriver {
 public:
  drvAsynRPiCan( const char *portName, const char *ttyName );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus writeGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus readOption( asynUser *pasynUser, const char *key, char *value, int maxChars );
  virtual asynStatus writeOption( asynUser *pasynUser, const char *key, const char *value );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  //  int P_GENERIC;
  //#define FIRST_RPICAN_COMMAND P_GENERIC
  //  int P_BITRATE;
  //#define LAST_RPICAN_COMMAND P_BITRATE

 private:
  int drvPeakCanWrite( TPCANMsg *pframe, int timeout );
  int drvPeakCanRead( TPCANRdMsg *pframe, int timeout );

  /* Our data */
  char *deviceName_;
  int   fd_;
};

//#define NUM_RPICAN_PARAMS (&LAST_RPICAN_COMMAND - &FIRST_RPICAN_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
