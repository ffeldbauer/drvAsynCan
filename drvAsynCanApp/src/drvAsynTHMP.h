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
// brief   Asyn driver for THMP using the RPi Can interface
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

#ifndef __ASYN_THMP_H__
#define __ASYN_THMP_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_THMP_ADCBUFFER_STRING          "Value"                //!<  asynInt32,          r  
#define P_THMP_IOBUFFER_STRING           "IOBoard"              //!<  asynUInt32Digital,  r/w
#define P_THMP_SERIALS_STRING            "Serial"               //!<  asynUInt32Digital,  r  
#define P_THMP_ERROR_STRING              "Error"                //!<  asynUInt32Digital,  r  

#define P_THMP_CONFIGIO_STRING           "ConfigIO"             //!<  asynInt32,          r/w
#define P_THMP_TRG_ADCBUFFER_STRING      "ReadoutADC"           //!<  asynInt32,          r/w  
#define P_THMP_TRG_IOBUFFER_STRING       "ReadoutIO"            //!<  asynInt32,          r/w  
#define P_THMP_TRG_SERIALS_STRING        "ReadoutSerials"       //!<  asynInt32,          r/w  

//! @brief   asynPortDriver for the Temperature and Humidity Monitoring Board for PANDA
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! Temperature and Humidity Monitoring Board for PANDA.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynTHMP : public asynPortDriver {
 public:
  drvAsynTHMP( const char *portName, const char *CanPort, const int can_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );

  void asynReadHandler( void* pframe );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_RawValue;       //!< index of parameter "Value"
  int P_IoBoard;        //!< index of parameter "IOBoard"
  int P_Serials;        //!< index of parameter "Serial"
  int P_ConfigIO;       //!< index of parameter "ConfigIO"
  int P_Error;          //!< index of parameter "Error"
  int P_Trg_ADC;        //!< index of parameter "ReadoutADC"
  int P_Trg_IO;         //!< index of parameter "ReadoutIO"
  int P_Trg_Serials;    //!< index of parameter "ReadoutSerials"
#define FIRST_THMP_COMMAND P_RawValue
#define LAST_THMP_COMMAND  P_Trg_Serials

 private:
  char                *_deviceName;
  epicsUInt32          _can_id;
  asynUser            *_pasynUser;
  asynCommon          *_pasynCommon;
  void                *_pvtCommon;
  asynGenericPointer  *_pasynPointerGeneric;
  void                *_pvtPointerGeneric;
  void                *_intrPvtPointerGeneric;
};

#define NUM_THMP_PARAMS (&LAST_THMP_COMMAND - &FIRST_THMP_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
