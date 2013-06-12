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
// brief   Asyn port driver for RPi CAN interface
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_RPICAN_H__
#define __ASYN_RPICAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/can.h>

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef struct can_frame can_frame_t;

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
//#define P_GENERIC_String          "RPICAN_FRAME"        /* asynGenericPointer, r/w */


//! @brief   asynPortDriver for PANDA Raspberry Pi CAN interface
//!
//! This is a lower lever asynPortDriver for communication with
//! the hardware CAN bus interface.\n
//! It provides the asynGenericPointer interface to higher level
//! asynPortDrivers.\n
//! The genericPointers should be of type struct can_frame.
class drvAsynCan : public asynPortDriver {
 public:
  drvAsynCan( const char *portName, const char *ttyName );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus writeGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus readOption( asynUser *pasynUser, const char *key, char *value, int maxChars );
  virtual asynStatus writeOption( asynUser *pasynUser, const char *key, const char *value );

 protected:
  /* Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_GENERIC;

 private:
  int drvRPiCanWrite( can_frame_t *pframe, int timeout );
  int drvRPiCanRead( can_frame_t *pframe, int timeout );

  /* Our data */
  char         *deviceName_;
  int          fd_;
  can_frame_t  frame_;
};


#endif

//******************************************************************************
//! EOF
//******************************************************************************
