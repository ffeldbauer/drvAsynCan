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
// version 1.0.0; Nov. 27, 2012
//******************************************************************************

#ifndef __ASYN_ISEG_HV_H__
#define __ASYN_ISEG_HV_H__

//_____ I N C L U D E S _______________________________________________________
#include <map>
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define P_ISEGHV_MODSTATUS_STRING          "ISEGHV_MOD_STATUS"          /* asynUInt32Digital,  r   */
#define P_ISEGHV_MODEVTSTATUS_STRING       "ISEGHV_MOD_EVENT_STATUS"    /* asynUInt32Digital,  r   */
#define P_ISEGHV_TEMPERATURE_STRING        "ISEGHV_TEMPERATURE"         /* asynFloat64,        r   */
#define P_ISEGHV_RAMPSPEED_STRING          "ISEGHV_RAMPSPEED"           /* asynFloat64,        r/w */
#define P_ISEGHV_CLEAREVTSTATUS_STRING     "ISEGHV_CLEAR_EVENT_STATUS"  /* asynInt32,          w   */

#define P_ISEGHV_CHANSTATUS_STRING         "ISEGHV_CHAN_STATUS"         /* asynUInt32Digital,  r   */
#define P_ISEGHV_CHANEVTSTATUS_STRING      "ISEGHV_CHAN_EVENT_STATUS"   /* asynUInt32Digital,  r   */
#define P_ISEGHV_VMOM_STRING               "ISEGHV_CHAN_VMOM"           /* asynFloat64,        r   */
#define P_ISEGHV_IMOM_STRING               "ISEGHV_CHAN_IMOM"           /* asynFloat64,        r   */
#define P_ISEGHV_VSET_STRING               "ISEGHV_CHAN_VSET"           /* asynFloat64,        r/w */
#define P_ISEGHV_ISET_STRING               "ISEGHV_CHAN_ISET"           /* asynFloat64,        r/w */

#define P_ISEGHV_EMO_STRING                "ISEGHV_EMERGENCYOFF"        /* asynInt32,          w   */
#define P_ISEGHV_SWITCH_STRING             "ISEGHV_SWITCHONOFF"         /* asynInt32,          w   */

//! @brief   asynPortDriver for ISEG EDS/EHS high voltage modules
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! EDS/EHS high voltage modules of ISEG Spezialelektronik GmbH.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegHv : public asynPortDriver {
 public:
  drvAsynIsegHv( const char *portName, const char *CanPort, const int crate_id, const int module_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );
  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
  virtual asynStatus readOption( asynUser *pasynUser, const char *key, char *value, int maxChars );
  virtual asynStatus writeOption( asynUser *pasynUser, const char *key, const char *value );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */

  int P_Chan_status;       //!< index of Parameter "ISEGHV_CHAN_STATUS"
#define FIRST_ISEGHV_COMMAND P_Chan_status
  int P_Chan_Event_status; //!< index of Parameter "ISEGHV_CHAN_EVENT_STATUS"
  int P_Chan_Vmom;         //!< index of Parameter "ISEGHV_CHAN_VMOM"
  int P_Chan_Imom;         //!< index of Parameter "ISEGHV_CHAN_IMOM"
  int P_Chan_Vset;         //!< index of Parameter "ISEGHV_CHAN_VSET"
  int P_Chan_Iset;         //!< index of Parameter "ISEGHV_CHAN_ISET"
  int P_Mod_status;        //!< index of Parameter "ISEGHV_MOD_STATUS"
  int P_Mod_Event_status;  //!< index of Parameter "ISEGHV_MOD_EVENT_STATUS"
  int P_Temperature;       //!< index of Parameter "ISEGHV_TEMPERATURE"
  int P_RampSpeed;         //!< index of Parameter "ISEGHV_RAMPSPEED"
  int P_ClearEvtStatus;    //!< index of Parameter "ISEGHV_CLEAR_EVENT_STATUS"
  int P_EmergencyOff;      //!< index of Parameter "ISEGHV_EMERGENCYOFF"
  int P_SwitchOnOff;       //!< index of Parameter "ISEGHV_SWITCHONOFF"
#define LAST_ISEGHV_COMMAND P_SwitchOnOff

 private:
  struct isegFrame {
    epicsUInt8    dlc;
    epicsUInt8    data0;
    epicsUInt8    data1;
  };

  std::map<int, isegFrame> cmdsFloat64_;
  std::map<int, isegFrame> cmdsUIn32D_;

  char           *deviceName_;
  epicsUInt32     can_id_;
  asynUser       *pAsynUserGenericPointer_;
  epicsFloat64    conversion_;
};

#define NUM_ISEGHV_PARAMS (&LAST_ISEGHV_COMMAND - &FIRST_ISEGHV_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
