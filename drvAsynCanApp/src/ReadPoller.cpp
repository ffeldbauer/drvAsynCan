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
// version 3.0.0; Jul. 29, 2014
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
ReadPoller* ReadPoller::_pinstance = NULL;

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

//------------------------------------------------------------------------------

ReadPoller::ReadPoller( const char* port )
  : _portname(port)
{
  if( pasynGenericPointerSyncIO->connect( port, 0, &_pasynUser, 0 ) != asynSuccess ) {
    _initialized = false;
    return;
  }
    
  _initialized = true;
  
  this->_eventId = epicsEventCreate( epicsEventEmpty );

  // Create the thread that polls interface for received messages
  epicsThreadCreate( "ReadPollerTask",
                     epicsThreadPriorityMedium,
                     epicsThreadGetStackSize( epicsThreadStackMedium ),
                     (EPICSTHREADFUNC)::CANreadPoller,
                     this );

}

//------------------------------------------------------------------------------

ReadPoller::ReadPoller( const ReadPoller& rother ){
  _portname = epicsStrDup( rother._portname );
  _pasynUser = rother._pasynUser;
}

//------------------------------------------------------------------------------

ReadPoller::~ReadPoller(){
}

//------------------------------------------------------------------------------

void ReadPoller::create( const char* port ){
  if ( NULL == _pinstance )  _pinstance = new ReadPoller( port );
  return;
}

//------------------------------------------------------------------------------

ReadPoller* ReadPoller::getInstance() {
  if ( NULL == _pinstance ) std::cerr << "No instance of ReadPoller" << std::endl;
  return _pinstance; 
}

//------------------------------------------------------------------------------

void ReadPoller::poll() {
  if ( !_initialized ) {
    std::cerr << "Initialization of ReadPoller failed" << std::endl;
    return;
  }
  can_frame_t *pframe = new can_frame_t;
  // Loop forever
  while (1) {
    pasynGenericPointerSyncIO->read( _pasynUser, pframe, .5 );
  }
}

