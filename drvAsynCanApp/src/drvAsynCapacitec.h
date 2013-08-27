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
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_THMP_H__
#define __ASYN_THMP_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_CAPACITEC_ADCBUFFER_STRING       "CAPACITEC_RAWVALUE"          /* asynInt32,          r   */
#define P_CAPACITEC_ERROR_STRING           "CAPACITEC_ERROR"             /* asynUInt32Digital,  r   */

#define P_CAPACITEC_TRG_ADCBUFFER_STRING   "CAPACITEC_TRG_ADC"           /* asynInt32,          r/w */
#define P_CAPACITEC_OS_STRING              "CAPACITEC_OS"                /* asynInt32,          r/w */

class drvAsynCapacitec : public asynPortDriver {
 public:
  drvAsynCapacitec( const char *portName, const char *CanPort, const int can_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );

  void asynReadHandler( void* pframe );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_RawValue;       //!< index of parameter "CAPACITEC_RAWVALUE"
  int P_Error;          //!< index of parameter "CAPACITEC_ERROR"
  int P_Trg_ADC;        //!< index of parameter "CAPACITEC_TRG_ADC"
  int P_OS;             //!< index of parameter "CAPACITEC_OS"

#define FIRST_CAPACITEC_COMMAND P_RawValue
#define LAST_CAPACITEC_COMMAND  P_OS

 private:
  char                *deviceName_;
  epicsUInt32          can_id_;
  asynUser            *pasynUser_;
  asynCommon          *pasynCommon_;
  void                *pvtCommon_;
  asynGenericPointer  *pasynGenericPointer_;
  void                *pvtGenericPointer_;
  void                *intrPvtGenericPointer_;
};

#define NUM_CAPACITEC_PARAMS (&LAST_CAPACITEC_COMMAND - &FIRST_CAPACITEC_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
