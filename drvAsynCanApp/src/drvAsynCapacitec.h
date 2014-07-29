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
// brief   Asyn driver for Capacitec DigitizerBoard using the RPi Can interface
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

#ifndef __ASYN_CAPACITEC_H__
#define __ASYN_CAPACITEC_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_CAPACITEC_ADCBUFFER_STRING       "Value"          //!< asynInt32,          r  
#define P_CAPACITEC_ERROR_STRING           "Error"             //!< asynUInt32Digital,  r  

#define P_CAPACITEC_TRG_ADCBUFFER_STRING   "Readout"           //!< asynInt32,          r/w
#define P_CAPACITEC_OS_STRING              "OS"                //!< asynInt32,          r/w

class drvAsynCapacitec : public asynPortDriver {
 public:
  drvAsynCapacitec( const char *portName, const char *CanPort, const int can_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );

  void asynReadHandler( void* pframe );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_RawValue;       //!< index of parameter "Value"
  int P_Error;          //!< index of parameter "Error"
  int P_Trg_ADC;        //!< index of parameter "Readout"
  int P_OS;             //!< index of parameter "OS"

#define FIRST_CAPACITEC_COMMAND P_RawValue
#define LAST_CAPACITEC_COMMAND  P_OS

 private:
  char                *_deviceName;
  epicsUInt32          _can_id;
  asynUser            *_pasynUser;
  asynCommon          *_pasynCommon;
  void                *_pvtCommon;
  asynGenericPointer  *_pasynGenericPointer;
  void                *_pvtGenericPointer;
  void                *_intrPvtGenericPointer;
};

#define NUM_CAPACITEC_PARAMS (&LAST_CAPACITEC_COMMAND - &FIRST_CAPACITEC_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
