//******************************************************************************
// Copyright (C) 2015 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Johannes-Gutenberg Universitaet Mainz
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
// brief   Asyn driver for Wiener VME crate remote control
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

#ifndef __ASYN_PANDA_FSC_HV_H__
#define __ASYN_PANDA_FSC_HV_H__

//_____ I N C L U D E S _______________________________________________________

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

// These are the drvInfo strings that are used to identify the parameters.
// They are used by asyn clients, including standard asyn device support
#define P_FSCHV_STATUS_STRING         "STATUS"         //!< asynUInt32Digital,  r   
#define P_FSCHV_SWITCH_STRING         "SWITCH"         //!< asynUInt32Digital,  r/w
#define P_FSCHV_VOLT0_STRING          "VOLTAGE0"       //!< asynFloat64,        r/w
#define P_FSCHV_VOLT1_STRING          "VOLTAGE1"       //!< asynFloat64,        r/w
#define P_FSCHV_VOLT2_STRING          "VOLTAGE2"       //!< asynFloat64,        r/w
#define P_FSCHV_CURR0_STRING          "CURRENT0"       //!< asynFloat64,        r/w
#define P_FSCHV_CURR1_STRING          "CURRENT1"       //!< asynFloat64,        r/w
#define P_FSCHV_CURR2_STRING          "CURRENT2"       //!< asynFloat64,        r/w
#define P_FSCHV_TEMP_STRING           "TEMPERATURE"    //!< asynInt32,          r

//! @brief   asynPortDriver for PANDA FSC HV developed at Protvino
//!
class drvAsynFscHV : public asynPortDriver {
 public:
  drvAsynFscHV( const char *portName, const char *CanPort, const int crate_id );

  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );

  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );

//  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
  virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );
 
 protected:

  int _status;
  int _switch;
  int _volt0;
  int _volt1;
  int _volt2;
  int _curr0;
  int _curr1;
  int _curr2;
  int _temp;

#define FIRST_PANDAFSCHV_COMMAND P_Status
#define LAST_PANDAFSCHV_COMMAND  P_temp

 private:
  char           *_deviceName;
  epicsUInt32     _crateId;
  asynUser       *_pcanif;

};

#define NUM_PANDAFSCHV_PARAMS (&LAST_PANDAFSCHV_COMMAND - &FIRST_PANDAFSCHV_COMMAND + 1)

#endif

