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
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

#ifndef __ASYN_CAN_H__
#define __ASYN_CAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/can.h>

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

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

  // These are the methods that we override from asynPortDriver
  virtual asynStatus readGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus writeGenericPointer( asynUser *pasynUser, void *pointer );
  virtual asynStatus readOctet( asynUser *pasynUser, char *value, size_t maxChars,
                                size_t *nActual, int *eomReason );
  virtual asynStatus writeOctet( asynUser *pasynUser, const char *value, size_t maxChars,
                                 size_t *nActual );

 protected:
  // Values used for pasynUser->reason, and indexes into the parameter library.
  int P_GENERIC;

 private:

  // Our data
  char             *_deviceName;
  int               _socket;
  struct can_frame  _frame;
};


#endif

