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
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C/C++ includes
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>

// local includes
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
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[0]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[1]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[2]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[3]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[4]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[5]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[6]
     << " 0x" << std::setw(2) << std::setfill('0') << rframe.data[7]
     << std::dec;
  return os;
}

bool operator==( const can_frame_t& lhs, const can_frame_t& rhs ) {
  if ( ( lhs.can_id != rhs.can_id ) || ( lhs.can_dlc != rhs.can_dlc ) )
    return false;

  for ( epicsUInt8 i = 0; i < lhs.can_dlc; i++ ) {
    if( lhs.data[i] != rhs.data[i] ) return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//! @brief   compare two can frames
//!
//! Compares two can frames with each other. Number of data bytes which should
//! be included in comparison can be set by the user.
//!
//! @param   [in]  lhs  Left hand sind of == operator
//! @param   [in]  rhs  Right hand side of == operator
//! @param   [in]  dlc  how many data bytes should be compared
//!
//! @return  true if can_id, can_dlc and up to `dlc` data bytes are equal.
//!          Otherwise false
//------------------------------------------------------------------------------
bool compCanFrame( const can_frame_t& lhs, const can_frame_t& rhs, epicsUInt8 dlc ){
  if ( ( lhs.can_id != rhs.can_id ) || ( lhs.can_dlc != rhs.can_dlc ) )
    return false;
  
  epicsUInt8 myDlc = ( lhs.can_dlc <= dlc ) ? lhs.can_dlc : dlc;
  for ( epicsUInt8 i = 0; i < myDlc; i++ ) {
    if( lhs.data[i] != rhs.data[i] ) return false;
  }
  return true;
}
