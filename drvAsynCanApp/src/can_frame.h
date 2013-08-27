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
// brief   Definition of socket CAN like can frame structure
//
// version 2.0.0; Jun. 05, 2013
//******************************************************************************

#ifndef __ASYN_CAN_FRAME_H__
#define __ASYN_CAN_FRAME_H__

//_____ I N C L U D E S _______________________________________________________
#include <iostream>
#include <string>

#include <epicsTypes.h>

//_____ D E F I N I T I O N S __________________________________________________
#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#endif

typedef struct {
  epicsUInt32 can_id;
  epicsUInt8  can_dlc;
  epicsUInt8  data[8] __attribute__ ((aligned(8)));
} can_frame_t;

std::ostream& operator<<( std::ostream& os, const can_frame_t& rframe );
std::string printTimestamp();

#endif

//******************************************************************************
//! EOF
//******************************************************************************
