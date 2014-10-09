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
// brief   Asyn driver for ISEG EHS/EDS high voltage modules using the RPi Can interface
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_ISEG_ECH44A_H__
#define __ASYN_ISEG_ECH44A_H__

//_____ I N C L U D E S _______________________________________________________
#include <map>
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_ISEGECH44A_CRATE_STATUS_STRING         "ISEGECH44A_STATUS"        /* asynUInt32Digital, r   */
#define P_ISEGECH44A_CRATE_CONTROL_STRING        "ISEGECH44A_CONTROL"       /* asynUInt32Digital, r/w */
#define P_ISEGECH44A_CRATE_EVENT_STATUS_STRING   "ISEGECH44A_EVT_STATUS"    /* asynUInt32Digital, r   */
#define P_ISEGECH44A_CRATE_EVENT_MASK_STRING     "ISEGECH44A_EVT_MASK"      /* asynUInt32Digital, r/w */
#define P_ISEGECH44A_CRATE_FAN_SPEED_STRING      "ISEGECH44A_FAN_SPEED"     /* asynFloat64,       r/w */
#define P_ISEGECH44A_CRATE_ON_OFF_STRING         "ISEGECH44A_ON_OFF"        /* asynInt32,         r/w */

//! @brief   asynPortDriver for ISEG ECH44A crate controller
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! ECH44A crate controller from ISEG Spezialelektronik GmbH.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegEch44a : public asynPortDriver {
 public:
  drvAsynIsegEch44a( const char *portName, const char *CanPort );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );

  void asynReadHandler( void* pframe );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */

  int P_CrateStatus;
#define FIRST_ISEGECH44A_COMMAND P_CrateStatus
  int P_CrateControl;
  int P_CrateEvtStatus;
  int P_CrateEvtMask;
  int P_CrateFanSpeed;
  int P_CrateOnOff;
#define LAST_ISEGECH44A_COMMAND P_CrateOnOff

 private:

  struct isegFrame {
    epicsUInt8    dlc;
    epicsUInt8    data0;
    epicsUInt8    data1;
  };

  std::map<int, isegFrame> _cmds;

  char                *_deviceName;
  asynUser            *_pasynUser;
  asynCommon          *_pasynCommon;
  void                *_pvtCommon;
  asynGenericPointer  *_pasynGenericPointer;
  void                *_pvtGenericPointer;
  void                *_intrPvtGenericPointer;
};

#define NUM_ISEGECH44A_PARAMS (&LAST_ISEGECH44A_COMMAND - &FIRST_ISEGECH44A_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
