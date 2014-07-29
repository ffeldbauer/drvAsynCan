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
// brief   Definition of all IOC shell functions for configuration
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C++ includes
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

// local includes
#include "drvAsynCan.h"
#include "drvAsynCapacitec.h"
#include "drvAsynIsegEhsEds.h"
#include "drvAsynIsegHvGlobal.h"
#include "drvAsynWienerVME.h"
#include "drvAsynTHMP.h"
#include "drvAsynLedPulser.h"
#include "drvAsynLedPulser2.h"
#include "drvAsynTmcm142.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

// Configuration routines.  Called directly, or from the iocsh function below 
extern "C" {

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynCan class.
  //!
  //! @param   [in]  portName The name of the asyn port driver to be created.
  //!          [in]  ttyName  The name of the interface 
  //----------------------------------------------------------------------------
  int drvAsynCanConfigure( const char *portName, const char *ttyName ) {
    new drvAsynCan( portName, ttyName );
    return( asynSuccess );
  }
  static const iocshArg initCanArg0 = { "portName", iocshArgString };
  static const iocshArg initCanArg1 = { "ttyName",  iocshArgString };
  static const iocshArg * const initCanArgs[] = { &initCanArg0, &initCanArg1 };
  static const iocshFuncDef initCanFuncDef = { "drvAsynCanConfigure", 2, initCanArgs };
  static void initCanCallFunc( const iocshArgBuf *args ) {
    drvAsynCanConfigure( args[0].sval, args[1].sval );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynIsegEhsEds class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  module_id   The id of the module
  //----------------------------------------------------------------------------
  int drvAsynIsegEhsEdsConfigure( const char *portName, const char *CanPort,
                                  const int module_id, const int channels ) {
    if ( module_id < 0 || module_id > 129 ) {
      printf("ERROR: Could not configure drvAsynIsegHv: invalid module id: %d\n", module_id );
      return( asynError );
    }
    char portNameChan[20]; strcpy( portNameChan, portName ); strcat( portNameChan, "C" );
    new drvAsynIsegHv( portName, CanPort, module_id, channels );
    return( asynSuccess );
  }
  static const iocshArg initIsegEhsEdsArg0 = { "portName",  iocshArgString };
  static const iocshArg initIsegEhsEdsArg1 = { "CanPort",   iocshArgString };
  static const iocshArg initIsegEhsEdsArg2 = { "module_id", iocshArgInt };
  static const iocshArg initIsegEhsEdsArg3 = { "channels",  iocshArgInt };
  static const iocshArg * const initIsegEhsEdsArgs[] = { &initIsegEhsEdsArg0, &initIsegEhsEdsArg1,
                                                         &initIsegEhsEdsArg2, &initIsegEhsEdsArg3 };
  static const iocshFuncDef initIsegEhsEdsFuncDef = { "drvAsynIsegEhsEdsConfigure", 4, initIsegEhsEdsArgs };
  static void initIsegEhsEdsCallFunc( const iocshArgBuf *args ) {
    drvAsynIsegEhsEdsConfigure( args[0].sval, args[1].sval, args[2].ival, args[3].ival );
  }
  
  // For global ISEG HV driver
  int drvAsynIsegHvGlobalConfigure( const char *portName, const char *CanPort ) {
    new drvAsynIsegHvGlobal( portName, CanPort );
    return( asynSuccess );
  }
  static const iocshArg * const initIsegHvGlobalArgs[] = { &initIsegEhsEdsArg0, &initIsegEhsEdsArg1 };
  static const iocshFuncDef initIsegHvGlobalFuncDef = { "drvAsynIsegHvGlobalConfigure", 2, initIsegHvArgs };
  static void initIsegHvGlobalCallFunc( const iocshArgBuf *args ) {
    drvAsynIsegHvGlobalConfigure( args[0].sval, args[1].sval );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynWienerVme class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  crate_id    The id of the crate
  //----------------------------------------------------------------------------
  int drvAsynWienerVmeConfigure( const char *portName, const char *CanPort,
                                 const int crate_id ) {
    if ( crate_id < 1 || crate_id > 127 ) {
      printf("ERROR: Could not configure drvAsynWienerVme: invalid crate id: %d", crate_id );
      return( asynError );
    }
    new drvAsynWienerVme( portName, CanPort, crate_id );
    return( asynSuccess );
  }
  static const iocshArg initWienerVmeArg0 = { "portName",   iocshArgString };
  static const iocshArg initWienerVmeArg1 = { "CanPort",    iocshArgString };
  static const iocshArg initWienerVmeArg2 = { "crate_id",   iocshArgInt };
  static const iocshArg * const initWienerVmeArgs[] = { &initWienerVmeArg0, &initWienerVmeArg1,
                                                        &initWienerVmeArg2 };
  static const iocshFuncDef initWienerVmeFuncDef = { "drvAsynWienerVmeConfigure", 3, initWienerVmeArgs };
  static void initWienerVmeCallFunc( const iocshArgBuf *args ) {
    drvAsynWienerVmeConfigure( args[0].sval, args[1].sval, args[2].ival );
  }
  
  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynTHMP class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  can_id      The CAN id of this THMP
  //----------------------------------------------------------------------------
  int drvAsynTHMPConfigure( const char *portName, const char *CanPort,
                            const int can_id ) {
    if ( can_id < 0x700 || can_id > 0x7ff ) {
      printf("ERROR: Could not configure drvAsynTHMP: invalid CAN id: %x", can_id );
      return( asynError );
    }
    new drvAsynTHMP( portName, CanPort, can_id );
    return( asynSuccess );
  }
  static const iocshArg initThmpArg0 = { "portName",   iocshArgString };
  static const iocshArg initThmpArg1 = { "CanPort",    iocshArgString };
  static const iocshArg initThmpArg2 = { "can_id",     iocshArgInt };
  static const iocshArg * const initThmpArgs[] = { &initThmpArg0, &initThmpArg1, &initThmpArg2 };
  static const iocshFuncDef initThmpFuncDef = { "drvAsynTHMPConfigure", 3, initThmpArgs };
  static void initThmpCallFunc( const iocshArgBuf *args ) {
    drvAsynTHMPConfigure( args[0].sval, args[1].sval, args[2].ival );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynCapacitec class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  can_id      The CAN id of this DigitizerBoard
  //----------------------------------------------------------------------------
  int drvAsynCapacitecConfigure( const char *portName, const char *CanPort,
                                 const int can_id ) {
    new drvAsynCapacitec( portName, CanPort, can_id );
    return( asynSuccess );
  }
  static const iocshArg initCapacitecArg0 = { "portName",   iocshArgString };
  static const iocshArg initCapacitecArg1 = { "CanPort",    iocshArgString };
  static const iocshArg initCapacitecArg2 = { "can_id",     iocshArgInt };
  static const iocshArg * const initCapacitecArgs[] = { &initCapacitecArg0, &initCapacitecArg1, &initCapacitecArg2 };
  static const iocshFuncDef initCapacitecFuncDef = { "drvAsynCapacitecConfigure", 3, initCapacitecArgs };
  static void initCapacitecCallFunc( const iocshArgBuf *args ) {
    drvAsynCapacitecConfigure( args[0].sval, args[1].sval, args[2].ival );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynLedPulser class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  can_id      The CAN id of this Led Pulser
  //!          [in]  filename    The Name of the file containing DAC and Intensity values
  //----------------------------------------------------------------------------
  int drvAsynLedPulserConfigure( const char *portName, const char *CanPort,
                                 const int can_id, const char *filename ) {
    new drvAsynLedPulser( portName, CanPort, can_id, filename );
    return( asynSuccess );
  }
  static const iocshArg initLedPulserArg0 = { "portName",   iocshArgString };
  static const iocshArg initLedPulserArg1 = { "CanPort",    iocshArgString };
  static const iocshArg initLedPulserArg2 = { "can_id",     iocshArgInt };
  static const iocshArg initLedPulserArg3 = { "filename",   iocshArgString };
  static const iocshArg * const initLedPulserArgs[] = { &initLedPulserArg0, &initLedPulserArg1,
                                                        &initLedPulserArg2, &initLedPulserArg3 };
  static const iocshFuncDef initLedPulserFuncDef = { "drvAsynLedPulserConfigure", 4, initLedPulserArgs };
  static void initLedPulserCallFunc( const iocshArgBuf *args ) {
    drvAsynLedPulserConfigure( args[0].sval, args[1].sval, args[2].ival, args[3].sval );
  }
  
    //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynLedPulser2 class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  can_id      The CAN id of this Led Pulser
  //!          [in]  filename    The Name of the file containing DAC and Intensity values
  //----------------------------------------------------------------------------
  int drvAsynLedPulser2Configure( const char *portName, const char *CanPort,
                                 const int can_id ) {
    new drvAsynLedPulser2( portName, CanPort, can_id );
    return( asynSuccess );
  }
  static const iocshArg initLedPulser2Arg0 = { "portName",   iocshArgString };
  static const iocshArg initLedPulser2Arg1 = { "CanPort",    iocshArgString };
  static const iocshArg initLedPulser2Arg2 = { "can_id",     iocshArgInt };
  static const iocshArg * const initLedPulser2Args[] = { &initLedPulser2Arg0, &initLedPulser2Arg1,
                                                        &initLedPulser2Arg2 };
  static const iocshFuncDef initLedPulser2FuncDef = { "drvAsynLedPulser2Configure", 4, initLedPulser2Args };
  static void initLedPulser2CallFunc( const iocshArgBuf *args ) {
    drvAsynLedPulser2Configure( args[0].sval, args[1].sval, args[2].ival );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynTmcm142 class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  CanPort     The name of the interface 
  //!          [in]  can_id_w    The CAN id of this TMCM142 driver
  //!          [in]  can_id_r    The CAN Reply id of this TMCM142 driver
  //----------------------------------------------------------------------------
  int drvAsynTmcm142Configure( const char *portName, const char *CanPort,
                               const int can_id_w, const int can_id_r ) {
    new drvAsynTmcm142( portName, CanPort, can_id_w, can_id_r );
    return( asynSuccess );
  }
  static const iocshArg initTmcm142Arg0 = { "portName",   iocshArgString };
  static const iocshArg initTmcm142Arg1 = { "CanPort",    iocshArgString };
  static const iocshArg initTmcm142Arg2 = { "can_id_w",   iocshArgInt };
  static const iocshArg initTmcm142Arg3 = { "can_id_r",   iocshArgInt };
  static const iocshArg * const initTmcm142Args[] = { &initTmcm142Arg0, &initTmcm142Arg1,
                                                      &initTmcm142Arg2, &initTmcm142Arg3 };
  static const iocshFuncDef initTmcm142FuncDef = { "drvAsynTmcm142Configure", 4, initTmcm142Args };
  static void initTmcm142CallFunc( const iocshArgBuf *args ) {
    drvAsynTmcm142Configure( args[0].sval, args[1].sval, args[2].ival, args[3].ival );
  }

  //----------------------------------------------------------------------------
  //! @brief   Register functions to EPICS
  //----------------------------------------------------------------------------
  void drvAsynCanRegister( void ) {
    static int firstTime = 1;
    if ( firstTime ) {
      iocshRegister( &initCanFuncDef,          initCanCallFunc );
      iocshRegister( &initIsegEhsEdsFuncDef,   initIsegEhsEdsCallFunc );
      iocshRegister( &initIsegHvGlobalFuncDef, initIsegHvGlobalCallFunc );
      iocshRegister( &initWienerVmeFuncDef,    initWienerVmeCallFunc );
      iocshRegister( &initThmpFuncDef,         initThmpCallFunc );
      iocshRegister( &initCapacitecFuncDef,    initCapacitecCallFunc );
      iocshRegister( &initLedPulserFuncDef,    initLedPulserCallFunc );
      iocshRegister( &initLedPulser2FuncDef,   initLedPulser2CallFunc );
      iocshRegister( &initTmcm142FuncDef,      initTmcm142CallFunc );
      firstTime = 0;
    }
  }
  
  epicsExportRegistrar( drvAsynCanRegister );
}
