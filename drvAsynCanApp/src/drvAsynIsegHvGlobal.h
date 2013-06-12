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

#ifndef __ASYN_ISEG_HV_GLOBAL_H__
#define __ASYN_ISEG_HV_GLOBAL_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_ISEGHV_EMO_STRING                "ISEGHV_EMERGENCYOFF"        /* asynInt32,          w   */
#define P_ISEGHV_SWITCH_STRING             "ISEGHV_SWITCHONOFF"         /* asynInt32,          w   */

//! @brief   asynPortDriver for ISEG EDS/EHS high voltage modules
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! EDS/EHS high voltage modules of ISEG Spezialelektronik GmbH.\n
//! It provides two parameters to switch all high voltage channels connected to
//! the CAN bus on and off, respectively.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegHvGlobal : public asynPortDriver {
 public:
  drvAsynIsegHvGlobal( const char *portName, const char *CanPort );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */

  int P_EmergencyOff;      //!< index of Parameter "ISEGHV_EMERGENCYOFF"
#define FIRST_ISEGHVGLOBAL_COMMAND P_EmergencyOff
  int P_SwitchOnOff;       //!< index of Parameter "ISEGHV_SWITCHONOFF"
#define LAST_ISEGHVGLOBAL_COMMAND P_SwitchOnOff

 private:
  char           *deviceName_;
  asynUser       *pAsynUserGenericPointer_;
};

#define NUM_ISEGHVGLOBAL_PARAMS (&LAST_ISEGHVGLOBAL_COMMAND - &FIRST_ISEGHVGLOBAL_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
