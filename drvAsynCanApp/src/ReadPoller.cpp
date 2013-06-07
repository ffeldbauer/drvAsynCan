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
// brief Poll for new CAN frames on the bus
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C++ includes
#include <iostream>

// EPICS includes
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

// ASYN includes
#include "asynDriver.h"
#include "asynGenericPointerSyncIO.h"
#include "asynStandardInterfaces.h"

// local includes
#include "ReadPoller.h"
#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________
ReadPoller* ReadPoller::pinstance_ = NULL;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Background process to periodically poll for CAN frames from CanPort
//!
//! @sa      ReadPoller::poll
//------------------------------------------------------------------------------
void CANreadPoller() {
  ReadPoller::getInstance()->poll();
}

ReadPoller::ReadPoller( const char* port )
  : portname_(port)
{
  if( pasynGenericPointerSyncIO->connect( port, 0, &pasynUser_, 0 ) != asynSuccess ) {
    initialized_ = false;
    return;
  }
    
  initialized_ = true;
  
  this->eventId_ = epicsEventCreate( epicsEventEmpty );

  // Create the thread that polls interface for received messages
  epicsThreadCreate( "ReadPollerTask",
                     epicsThreadPriorityMedium,
                     epicsThreadGetStackSize( epicsThreadStackMedium ),
                     (EPICSTHREADFUNC)::CANreadPoller,
                     this );

}

ReadPoller::ReadPoller( const ReadPoller& rother ){
  portname_ = epicsStrDup( rother.portname_ );
  pasynUser_ = rother.pasynUser_;
}

ReadPoller::~ReadPoller(){
}

void ReadPoller::create( const char* port ){
  if ( NULL == pinstance_ )  pinstance_ = new ReadPoller( port );
  return;
}

ReadPoller* ReadPoller::getInstance() {
  if ( NULL == pinstance_ ) std::cerr << "No instance of ReadPoller" << std::endl;
  return pinstance_; 
}

void ReadPoller::poll() {
  if ( !initialized_ ) {
    std::cerr << "Initialization of ReadPoller failed" << std::endl;
    return;
  }
  can_frame_t *pframe = new can_frame_t;
  // Loop forever
  while (1) {
    pasynGenericPointerSyncIO->read( pasynUser_, pframe, 1. );
  }
}

