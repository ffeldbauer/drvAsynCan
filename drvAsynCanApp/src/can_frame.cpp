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
// brief   Operator << for can_frame_t structure
//
// version 2.0.0; Aug. 23, 2013
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C includes
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>

#include "can_frame.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________
std::string printTimestamp() {
  time_t t;
  struct tm tm;
  char buffer[40];
  time( &t );
  localtime_r( &t, &tm );
  strftime( buffer, 40, "%Y/%m/%d %H:%M:%S", &tm );
  return buffer;
}

std::ostream& operator<<( std::ostream& os, const can_frame_t& rframe ) {
  os << std::hex 
     << "0x" << std::setw(8) << std::setfill('0') << rframe.can_id
     << std::setw(1) << rframe.can_dlc
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[0]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[1]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[2]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[3]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[4]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[5]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[6]
     << "0x" << std::setw(2) << std::setfill('0') << rframe.data[7];
  return os;
} 
