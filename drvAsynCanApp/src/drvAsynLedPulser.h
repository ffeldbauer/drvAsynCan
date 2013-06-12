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
// brief   Asyn driver for PANDA LED Pulser
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_LED_PULSER_H__
#define __ASYN_LED_PULSER_H__

//_____ I N C L U D E S _______________________________________________________
#include <vector>

#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_LEDPULSER_INTENSITY_STRING       "LEDPULSER_INTENSITY"        /* asynFloat64,       r/w */
#define P_LEDPULSER_FREQUENCY_STRING       "LEDPULSER_FREQUENCY"        /* asynFloat64,       r/w */
#define P_LEDPULSER_CYCLES_STRING          "LEDPULSER_CYCLES"           /* asynInt32,         r/w */
#define P_LEDPULSER_TRG_MODE_STRING        "LEDPULSER_TRG_MODE"         /* asynUInt32Digital, r/w */
#define P_LEDPULSER_COLOR_STRING           "LEDPULSER_COLOR"            /* asynUInt32Digital, r/w */
#define P_LEDPULSER_WRITE_STRING           "LEDPULSER_WRITE"            /* asynInt32,         r/w */
#define P_LEDPULSER_READ_STRING            "LEDPULSER_READ"             /* asynInt32,         r/w */

//! @brief   asynPortDriver for the PANDA EMC light pulser system
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! PANDA EMC light pulser system.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynLedPulser : public asynPortDriver {
 public:
  drvAsynLedPulser( const char *portName, const char *CanPort,
                    const int can_id, const char *filename );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_Intensity;    //!< index of parameter "LEDPULSER_INTENSITY"
#define FIRST_LEDPULSER_COMMAND P_Intensity
  int P_Frequency;    //!< index of parameter "LEDPULSER_FREQUENCY"
  int P_Cycles;       //!< index of parameter "LEDPULSER_CYCLES"
  int P_Trg_Mode;     //!< index of parameter "LEDPULSER_TRG_MODE"
  int P_Color;        //!< index of parameter "LEDPULSER_COLOR"
  int P_write;        //!< index of parameter "LEDPULSER_WRITE"
  int P_read;         //!< index of parameter "LEDPULSER_READ"
#define LAST_LEDPULSER_COMMAND P_read

 private:
  void         setFrequency( epicsFloat64 value, epicsUInt8 &high, epicsUInt8 &low );
  epicsFloat64 getFrequency( epicsUInt8 high, epicsUInt8 low );
  void         setIntensity( epicsFloat64 value, epicsUInt8 &high, epicsUInt8 &low );
  epicsFloat64 getIntensity( epicsUInt8 high, epicsUInt8 low );

  char           *deviceName_;
  epicsUInt32     can_id_;
  asynUser       *pAsynUserGenericPointer_;

  struct LCD_values {
    epicsUInt32  dac;
    epicsFloat64 intensity;
  };
  typedef struct LCD_values LCD_values;
  std::vector<LCD_values>  lcd_values_;
};

#define NUM_LEDPULSER_PARAMS (&LAST_LEDPULSER_COMMAND - &FIRST_LEDPULSER_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
