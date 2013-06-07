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

#ifndef __ASYN_ISEG_HV_CHAN_H__
#define __ASYN_ISEG_HV_CHAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <map>
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_ISEGHV_CHANSTATUS_STRING         "ISEGHV_CHAN_STATUS"             /* asynUInt32Digital,  r   */
#define P_ISEGHV_CHAN_CTRL_STRING          "ISEGHV_CHAN_CTRL"               /* asynUInt32Digital,  r/w */
#define P_ISEGHV_CHANEVTSTATUS_STRING      "ISEGHV_CHAN_EVENT_STATUS"       /* asynUInt32Digital,  r   */
#define P_ISEGHV_VSET_STRING               "ISEGHV_CHAN_VSET"               /* asynFloat64,        r/w */
#define P_ISEGHV_ISET_STRING               "ISEGHV_CHAN_ISET"               /* asynFloat64,        r/w */
#define P_ISEGHV_VMOM_STRING               "ISEGHV_CHAN_VMOM"               /* asynFloat64,        r   */
#define P_ISEGHV_IMOM_STRING               "ISEGHV_CHAN_IMOM"               /* asynFloat64,        r   */
#define P_ISEGHV_VBOUNDS_STRING            "ISEGHV_CHAN_VBOUNDS"            /* asynFloat64,        r/w */
#define P_ISEGHV_IBOUNDS_STRING            "ISEGHV_CHAN_IBOUNDS"            /* asynFloat64,        r/w */
#define P_ISEGHV_VNOM_STRING               "ISEGHV_CHAN_VNOMINAL"           /* asynFloat64,        r   */
#define P_ISEGHV_INOM_STRING               "ISEGHV_CHAN_INOMINAL"           /* asynFloat64,        r   */

//! @brief   asynPortDriver for ISEG EDS/EHS high voltage modules
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! EDS/EHS high voltage modules of ISEG Spezialelektronik GmbH.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegHvChannel : public asynPortDriver {
 public:
  drvAsynIsegHvChannel( const char *portName, const char *CanPort, const int module_id, const int channels );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );

  void asynReadHandler( void* pframe );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */

  int P_Chan_status;         //!< index of Parameter "ISEGHV_CHAN_STATUS"
  int P_Chan_Ctrl;           //!< index of Parameter "ISEGHV_CHAN_CTRL       "
  int P_Chan_Event_status;   //!< index of Parameter "ISEGHV_CHAN_EVENT_STATUS"
  int P_Chan_Vset;           //!< index of Parameter "ISEGHV_CHAN_VSET"
  int P_Chan_Iset;           //!< index of Parameter "ISEGHV_CHAN_ISET"
  int P_Chan_Vmom;           //!< index of Parameter "ISEGHV_CHAN_VMOM"
  int P_Chan_Imom;           //!< index of Parameter "ISEGHV_CHAN_IMOM"
  int P_Chan_Vbounds;        //!< index of Parameter "ISEGHV_CHAN_VBOUNDS"
  int P_Chan_Ibounds;        //!< index of Parameter "ISEGHV_CHAN_IBOUNDS"
  int P_Chan_Vnom;           //!< index of Parameter "ISEGHV_CHAN_VNOMINAL"
  int P_Chan_Inom;           //!< index of Parameter "ISEGHV_CHAN_INOMINAL"
#define FIRST_ISEGHV_CHAN_COMMAND P_Chan_status
#define LAST_ISEGHV_CHAN_COMMAND  P_Chan_Inom

 private:
  struct isegFrame {
    epicsUInt8    dlc;
    epicsUInt8    data0;
    epicsUInt8    data1;
  };

  std::map<int, isegFrame> cmds_;

  char                *deviceName_;
  const int            channels_;
  epicsUInt32          can_id_;
  asynUser            *pasynUser_;
  asynCommon          *pasynCommon_;
  void                *pvtCommon_;
  asynGenericPointer  *pasynGenericPointer_;
  void                *pvtGenericPointer_;
  void                *intrPvtGenericPointer_;

};

#define NUM_ISEGHV_CHAN_PARAMS (&LAST_ISEGHV_CHAN_COMMAND - &FIRST_ISEGHV_CHAN_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
