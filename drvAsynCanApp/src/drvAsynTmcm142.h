//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// brief   Asyn driver for TMCM142 1-axis stepper controller/driver
//         using the RPi Can interface
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

#ifndef __ASYN_MOTOR_DRIVER_H__
#define __ASYN_MOTOR_DRIVER_H__

//_____ I N C L U D E S _______________________________________________________
#include <map>

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_TMCM142_ROR_STRING       "ROR"        //!<  asynInt32,         r/w 
#define P_TMCM142_ROL_STRING       "ROL"        //!<  asynInt32,         r/w 
#define P_TMCM142_MST_STRING       "MST"        //!<  asynInt32,         r/w 
#define P_TMCM142_MVP_STRING       "MVP"        //!<  asynInt32,         r/w 
#define P_TMCM142_SAP_STRING       "SAP"        //!<  asynInt32,         r/w 
#define P_TMCM142_GAP_STRING       "GAP"        //!<  asynInt32,         r/w 
#define P_TMCM142_SAP_U_STRING     "SAP_U"      //!<  asynUInt32Digital, r/w 
#define P_TMCM142_GAP_U_STRING     "GAP_U"      //!<  asynUInt32Digital, r/w 
#define P_TMCM142_STAP_STRING      "STAP"       //!<  asynInt32,         r/w 
#define P_TMCM142_RSAP_STRING      "RSAP"       //!<  asynInt32,         r/w 
#define P_TMCM142_STAP_U_STRING    "STAP_U"     //!<  asynUInt32Digital, r/w 
#define P_TMCM142_RSAP_U_STRING    "RSAP_U"     //!<  asynUInt32Digital, r/w 
#define P_TMCM142_SGP_STRING       "SGP"        //!<  asynInt32,         r/w 
#define P_TMCM142_GGP_STRING       "GGP"        //!<  asynInt32,         r/w 
#define P_TMCM142_STGP_STRING      "STGP"       //!<  asynInt32,         r/w 
#define P_TMCM142_RSGP_STRING      "RSGP"       //!<  asynInt32,         r/w 
#define P_TMCM142_RFS_STRING       "RFS"        //!<  asynInt32,         r/w 
#define P_TMCM142_SIO_STRING       "SIO"        //!<  asynInt32,         r/w 
#define P_TMCM142_GIO_STRING       "GIO"        //!<  asynInt32,         r/w 
#define P_TMCM142_CALC_STRING      "CALC"       //!<  asynInt32,         r/w 
#define P_TMCM142_COMP_STRING      "COMP"       //!<  asynInt32,         r/w 
#define P_TMCM142_JC_STRING        "JC"         //!<  asynInt32,         r/w 
#define P_TMCM142_JA_STRING        "JA"         //!<  asynInt32,         r/w 
#define P_TMCM142_CSUB_STRING      "CSUB"       //!<  asynInt32,         r/w 
#define P_TMCM142_RSUB_STRING      "RSUB"       //!<  asynInt32,         r/w 
#define P_TMCM142_WAIT_STRING      "WAIT"       //!<  asynInt32,         r/w 
#define P_TMCM142_STOP_STRING      "STOP"       //!<  asynInt32,         r/w 
#define P_TMCM142_SCO_STRING       "SCO"        //!<  asynInt32,         r/w 
#define P_TMCM142_GCO_STRING       "GCO"        //!<  asynInt32,         r/w 
#define P_TMCM142_CCO_STRING       "CCO"        //!<  asynInt32,         r/w 
#define P_TMCM142_CALCX_STRING     "CALCX"      //!<  asynInt32,         r/w 
#define P_TMCM142_AAP_STRING       "AAP"        //!<  asynInt32,         r/w 
#define P_TMCM142_AGP_STRING       "AGP"        //!<  asynInt32,         r/w 
#define P_TMCM142_CLE_STRING       "CLE"        //!<  asynInt32,         r/w 
#define P_TMCM142_ACO_STRING       "ACO"        //!<  asynInt32,         r/w 

#define P_TMCM142_STATUS_STRING    "STATUS"     //!<  asynInt32,         r   

//! @brief   asynPortDriver for TMCM142 1-axis stepper controller/driver
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! TMCM142 1-axis stepper controller/driver.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynTmcm142 : public asynPortDriver {
 public:
  drvAsynTmcm142( const char *portName, const char *CanPort,
                  const int can_id_w, const int can_id_r );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );

 protected:

  /* Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_ROR;
  int P_ROL;
  int P_MVP;
  int P_MST;
  int P_RFS;
  int P_SCO;
  int P_CCO;
  int P_GCO;
  int P_SAP;
  int P_GAP;
  int P_SAP_U;
  int P_GAP_U;
  int P_STAP;
  int P_RSAP;
  int P_STAP_U;
  int P_RSAP_U;
  int P_SGP;
  int P_GGP;
  int P_STGP;
  int P_RSGP;
  int P_SIO;
  int P_GIO;
  int P_JA;
  int P_JC;
  int P_COMP;
  int P_CLE;
  int P_CSUB;
  int P_RSUB;
  int P_WAIT;
  int P_STOP;
  int P_CALC;
  int P_CALCX;
  int P_AAP;
  int P_AGP;
  int P_ACO;
  int P_STATUS;
#define FIRST_TMCM142_COMMAND  P_ROR
#define LAST_TMCM142_COMMAND   P_STATUS

 private:
  char           *_deviceName;
  epicsUInt32     _can_id_w;
  epicsUInt32     _can_id_r;
  asynUser       *_pasynGenericPointer;

  std::map<int, epicsUInt8> _cmds_r;
  std::map<int, epicsUInt8> _cmds_w;
};

#define NUM_TMCM142_PARAMS (&LAST_TMCM142_COMMAND - &FIRST_TMCM142_COMMAND + 1)

#endif

