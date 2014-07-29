//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Ruhr-Universitaet Bochum, Lehrstuhl fuer Experimentalphysik I
// Extensions for sencond generation LED pulser:
// Copyright (C) 2014 Tobias Triffterer <tobias@ep1.ruhr-uni-bochum.de>
//               Ruhr-Universität Bochum, Insitut für Experimentalphysik I
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

#ifndef __ASYN_LED_PULSER2_H__
#define __ASYN_LED_PULSER2_H__

//_____ I N C L U D E S _______________________________________________________
#include <vector>

#include "asynPortDriver.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

// These are the drvInfo strings that are used to identify the parameters.
// They are used by asyn clients, including standard asyn device support
// Input variables, i.e. data read from the LED pulser
#define P_LEDPULSER2_INTENSITY_IN_STRING       "LEDPULSER2_INTENSITY_IN"        /* asynInt32,         r/o */
#define P_LEDPULSER2_FREQUENCY_IN_STRING       "LEDPULSER2_FREQUENCY_IN"        /* asynInt32,       r/o */
#define P_LEDPULSER2_CYCLES_IN_STRING          "LEDPULSER2_CYCLES_IN"           /* asynInt32,         r/o */
#define P_LEDPULSER2_TRG_MODE_IN_STRING        "LEDPULSER2_TRG_MODE_IN"         /* asynUInt32Digital, r/o */
#define P_LEDPULSER2_COLOR_IN_STRING           "LEDPULSER2_COLOR_IN"            /* asynUInt32Digital, r/o */
// Output variables, i.e. data to be written to the light pulser
#define P_LEDPULSER2_INTENSITY_OUT_STRING       "LEDPULSER2_INTENSITY_OUT"      /* asynInt32,         w/o */
#define P_LEDPULSER2_FREQUENCY_OUT_STRING       "LEDPULSER2_FREQUENCY_OUT"      /* asynInt32,       w/o */
#define P_LEDPULSER2_CYCLES_OUT_STRING          "LEDPULSER2_CYCLES_OUT"         /* asynInt32,         w/o */
#define P_LEDPULSER2_TRG_MODE_OUT_STRING        "LEDPULSER2_TRG_MODE_OUT"       /* asynUInt32Digital, w/o */
#define P_LEDPULSER2_COLOR_OUT_STRING           "LEDPULSER2_COLOR_OUT"          /* asynUInt32Digital, w/o */
// Command variables to tell the driver when to initiate CAN communication
#define P_LEDPULSER2_WRITE_STRING           "LEDPULSER2_WRITE"                  /* asynInt32,         r/w */
#define P_LEDPULSER2_READ_STRING            "LEDPULSER2_READ"                   /* asynInt32,         r/w */

//! @brief   asynPortDriver for the PANDA EMC light pulser system
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! second generation of the PANDA EMC light pulser system.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynLedPulser2 : public asynPortDriver {
 public:
  drvAsynLedPulser2( const char *portName, const char *CanPort,
                    const int can_id );

  // These are the methods that we override from asynPortDriver
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );

 protected:
  // Values used for pasynUser->reason, and indexes into the parameter library.
  // Input variables, i.e. data read from the LED pulser
  int P_Intensity_in;    //!< index of parameter "LEDPULSER2_INTENSITY"
#define FIRST_LEDPULSER2_COMMAND P_Intensity_in
  int P_Frequency_in;    //!< index of parameter "LEDPULSER2_FREQUENCY"
  int P_Cycles_in;       //!< index of parameter "LEDPULSER2_CYCLES"
  int P_Trg_Mode_in;     //!< index of parameter "LEDPULSER2_TRG_MODE"
  int P_Color_in;        //!< index of parameter "LEDPULSER2_COLOR"
  // Output variables, i.e. data to be written to the light pulser
  int P_Intensity_out;   //!< index of parameter "LEDPULSER2_INTENSITY"
  int P_Frequency_out;   //!< index of parameter "LEDPULSER2_FREQUENCY"
  int P_Cycles_out;      //!< index of parameter "LEDPULSER2_CYCLES"
  int P_Trg_Mode_out;    //!< index of parameter "LEDPULSER2_TRG_MODE"
  int P_Color_out;       //!< index of parameter "LEDPULSER2_COLOR"
  // Command variables to tell the driver when to initiate CAN communication
  int P_write;           //!< index of parameter "LEDPULSER2_WRITE"
  int P_read;            //!< index of parameter "LEDPULSER2_READ"
#define LAST_LEDPULSER2_COMMAND P_read

 private:
  void         setFrequency( epicsInt32 value, epicsUInt8 &high, epicsUInt8 &low );
  epicsInt32   getFrequency( epicsUInt8 high, epicsUInt8 low );
  void         setIntensity( epicsUInt16 value, epicsUInt8 &high, epicsUInt8 &low );
  epicsUInt16  getIntensity( epicsUInt8 high, epicsUInt8 low );

  char           *_deviceName;
  epicsUInt32     _can_id;
  asynUser       *_pasynGenericPointer;
  static const unsigned long int _ucClockFrequency = 8000000;
  static const unsigned int _ucPrescaler = 8;
  void parseReadCanFrame( const can_frame_t& pframe, const bool alsoAssignOutputParams = false );
};

#define NUM_LEDPULSER2_PARAMS (&LAST_LEDPULSER2_COMMAND - &FIRST_LEDPULSER2_COMMAND + 1)

#endif

