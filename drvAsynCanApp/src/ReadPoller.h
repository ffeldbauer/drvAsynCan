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
// brief
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_READ_POLLER_H__
#define __ASYN_READ_POLLER_H__

//_____ I N C L U D E S _______________________________________________________

// EPICS includes
#include <epicsEvent.h>

// ASYN includes
#include "asynDriver.h"
#include "asynGenericPointer.h"
#include "asynStandardInterfaces.h"

//_____ D E F I N I T I O N S __________________________________________________

class ReadPoller {

 public:
  static void create( const char* );
  static ReadPoller* getInstance();

  void poll();

 private:
  ReadPoller( const char* );
  ReadPoller( const ReadPoller& );
  ~ReadPoller();

  static ReadPoller* pinstance_;

  const char*   portname_;
  asynUser*     pasynUser_;
  bool          initialized_;
  epicsEventId  eventId_;
  /* asynCommon* pasynCommon_; */
  /* asynGenericPointer* pasynGenricPointer_; */
};

#endif
