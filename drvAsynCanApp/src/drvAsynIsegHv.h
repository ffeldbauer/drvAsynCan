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

#ifndef __ASYN_ISEG_HV_H__
#define __ASYN_ISEG_HV_H__

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
#define P_ISEGHV_R_CHANSTATUS_STRING       "ISEGHV_READ_CHAN_STATUS"        /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_CHAN_CTRL_STRING        "ISEGHV_READ_CHAN_CTRL"          /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_CHANEVTSTATUS_STRING    "ISEGHV_READ_CHAN_EVENT_STATUS"  /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_VSET_STRING             "ISEGHV_READ_CHAN_VSET"          /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_ISET_STRING             "ISEGHV_READ_CHAN_ISET"          /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_VMOM_STRING             "ISEGHV_READ_CHAN_VMOM"          /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_IMOM_STRING             "ISEGHV_READ_CHAN_IMOM"          /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_VBOUNDS_STRING          "ISEGHV_READ_CHAN_VBOUNDS"       /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_IBOUNDS_STRING          "ISEGHV_READ_CHAN_IBOUNDS"       /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_VNOM_STRING             "ISEGHV_READ_CHAN_VNOMINAL"      /* asynUInt32Digital,  r/w */
#define P_ISEGHV_R_INOM_STRING             "ISEGHV_READ_CHAN_INOMINAL"      /* asynUInt32Digital,  r/w */

#define P_ISEGHV_MODSTATUS_STRING          "ISEGHV_MOD_STATUS"              /* asynUInt32Digital,  r   */
#define P_ISEGHV_MODCTRL_STRING            "ISEGHV_MOD_CTRL"                /* asynUInt32Digital,  r/w */
#define P_ISEGHV_MODEVTSTATUS_STRING       "ISEGHV_MOD_EVENT_STATUS"        /* asynUInt32Digital,  r   */
#define P_ISEGHV_VRAMPSPEED_STRING         "ISEGHV_V_RAMPSPEED"             /* asynFloat64,        r/w */
#define P_ISEGHV_IRAMPSPEED_STRING         "ISEGHV_I_RAMPSPEED"             /* asynFloat64,        r/w */
#define P_ISEGHV_SUPPLY24_STRING           "ISEGHV_SUPPLY24"                /* asynFloat64,        r   */
#define P_ISEGHV_SUPPLY5_STRING            "ISEGHV_SUPPLY5"                 /* asynFloat64,        r   */
#define P_ISEGHV_TEMPERATURE_STRING        "ISEGHV_TEMPERATURE"             /* asynFloat64,        r   */

//! @brief   asynPortDriver for ISEG EDS/EHS high voltage modules
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! EDS/EHS high voltage modules of ISEG Spezialelektronik GmbH.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegHv : public asynPortDriver {
 public:
  drvAsynIsegHv( const char *portName, const char *CanPort, const int module_id, const int channels );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
  virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );

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
  int P_R_Chan_status;       //!< index of Parameter "ISEGHV_READ_CHAN_STATUS"
  int P_R_Chan_Ctrl;         //!< index of Parameter "ISEGHV_READ_CHAN_CTRL"
  int P_R_Chan_Event_status; //!< index of Parameter "ISEGHV_READ_CHAN_EVENT_STATUS"
  int P_R_Chan_Vset;         //!< index of Parameter "ISEGHV_READ_CHAN_VSET"
  int P_R_Chan_Iset;         //!< index of Parameter "ISEGHV_READ_CHAN_ISET"
  int P_R_Chan_Vmom;         //!< index of Parameter "ISEGHV_READ_CHAN_VMOM"
  int P_R_Chan_Imom;         //!< index of Parameter "ISEGHV_READ_CHAN_IMOM"
  int P_R_Chan_Vbounds;      //!< index of Parameter "ISEGHV_READ_CHAN_VBOUNDS"
  int P_R_Chan_Ibounds;      //!< index of Parameter "ISEGHV_READ_CHAN_IBOUNDS"
  int P_R_Chan_Vnom;         //!< index of Parameter "ISEGHV_READ_CHAN_VNOMINAL"
  int P_R_Chan_Inom;         //!< index of Parameter "ISEGHV_READ_CHAN_INOMINAL"
  int P_Mod_status;          //!< index of Parameter "ISEGHV_MOD_STATUS"
  int P_Mod_Ctrl;            //!< index of Parameter "ISEGHV_MOD_CTRL"
  int P_Mod_Event_status;    //!< index of Parameter "ISEGHV_MOD_EVENT_STATUS"
  int P_VRampSpeed;          //!< index of Parameter "ISEGHV_V_RAMPSPEED"
  int P_IRampSpeed;          //!< index of Parameter "ISEGHV_I_RAMPSPEED"
  int P_Supply24;            //!< index of Parameter "ISEGHV_SUPPLY24"
  int P_Supply5;             //!< index of Parameter "ISEGHV_SUPPLY5"
  int P_Temperature;         //!< index of Parameter "ISEGHV_TEMPERATURE"
#define FIRST_ISEGHV_COMMAND P_Chan_status
#define LAST_ISEGHV_COMMAND  P_Temperature

 private:
  struct isegFrame {
    epicsUInt8    dlc;
    epicsUInt8    data0;
    epicsUInt8    data1;
  };

  std::map<int, isegFrame> cmds_;

  char                *deviceName_;
  epicsUInt32          can_id_;
  asynUser            *pasynUser_;
  asynCommon          *pasynCommon_;
  void                *pvtCommon_;
  asynGenericPointer  *pasynGenericPointer_;
  void                *pvtGenericPointer_;
  void                *intrPvtGenericPointer_;
//  epicsUInt16          chanMsk_;

};

#define NUM_ISEGHV_PARAMS (&LAST_ISEGHV_COMMAND - &FIRST_ISEGHV_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
