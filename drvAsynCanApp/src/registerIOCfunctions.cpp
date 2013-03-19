//******************************************************************************
//! Copyright (C) 2012 Florian Feldbauer
//!
//! This program is free software; you can redistribute it and/or modify
//! it under the terms of the GNU General Public License as published by
//! the Free Software Foundation; either version 3 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//! GNU General Public License for more details.
//!
//! You should have received a copy of the GNU General Public License
//! along with this program; if not, write to the Free Software
//! Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//!
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   Definition of all IOC shell functions for configuration
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cstdio>
#include <cstdlib>
#include <cstring>

/* EPICS includes */
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

/* local includes */
#include "drvAsynIsegHv.h"
#include "drvAsynIsegHvGlobal.h"
#include "drvAsynWienerVME.h"
#include "drvAsynTHMP.h"
#include "drvAsynLedPulser.h"
#include "drvAsynTmcm142.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

/* Configuration routines.  Called directly, or from the iocsh function below */
extern "C" {

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynIsegHv class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  RPiCanPort  The name of the interface 
  //!          [in]  crate_id    The id of the crate
  //!          [in]  module_id   The id of the module inside the crate
  //----------------------------------------------------------------------------
  int drvAsynIsegHvConfigure( const char *portName, const char *RPiCanPort,
                              const int crate_id, const int module_id ) {
    if ( crate_id < 0 || crate_id > 7 ||
         module_id < 0 || module_id > 7 ) {
      printf("ERROR: Could not configure drvAsynIsegHv: invalid crate/module id: %d, %d", crate_id, module_id );
      return( asynError );
    }
    new drvAsynIsegHv( portName, RPiCanPort, crate_id, module_id );
    return( asynSuccess );
  }
  static const iocshArg initIsegHvArg0 = { "portName",   iocshArgString };
  static const iocshArg initIsegHvArg1 = { "RPiCanPort", iocshArgString };
  static const iocshArg initIsegHvArg2 = { "crate_id",   iocshArgInt };
  static const iocshArg initIsegHvArg3 = { "module_id",  iocshArgInt };
  static const iocshArg * const initIsegHvArgs[] = { &initIsegHvArg0, &initIsegHvArg1,
                                                     &initIsegHvArg2, &initIsegHvArg3 };
  static const iocshFuncDef initIsegHvFuncDef = { "drvAsynIsegHvConfigure", 4, initIsegHvArgs };
  static void initIsegHvCallFunc( const iocshArgBuf *args ) {
    drvAsynIsegHvConfigure( args[0].sval, args[1].sval, args[2].ival, args[3].ival );
  }

  // For global ISEG HV driver
  int drvAsynIsegHvGlobalConfigure( const char *portName, const char *RPiCanPort ) {
    new drvAsynIsegHvGlobal( portName, RPiCanPort );
    return( asynSuccess );
  }
  static const iocshArg * const initIsegHvGlobalArgs[] = { &initIsegHvArg0, &initIsegHvArg1 };
  static const iocshFuncDef initIsegHvGlobalFuncDef = { "drvAsynIsegHvGlobalConfigure", 2, initIsegHvArgs };
  static void initIsegHvGlobalCallFunc( const iocshArgBuf *args ) {
    drvAsynIsegHvGlobalConfigure( args[0].sval, args[1].sval );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynWienerVme class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  RPiCanPort  The name of the interface 
  //!          [in]  crate_id    The id of the crate
  //----------------------------------------------------------------------------
  int drvAsynWienerVmeConfigure( const char *portName, const char *RPiCanPort,
                                 const int crate_id ) {
    if ( crate_id < 1 || crate_id > 127 ) {
      printf("ERROR: Could not configure drvAsynWienerVme: invalid crate id: %d", crate_id );
      return( asynError );
    }
    new drvAsynWienerVme( portName, RPiCanPort, crate_id );
    return( asynSuccess );
  }
  static const iocshArg initWienerVmeArg0 = { "portName",   iocshArgString };
  static const iocshArg initWienerVmeArg1 = { "RPiCanPort", iocshArgString };
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
  //!          [in]  RPiCanPort  The name of the interface 
  //!          [in]  can_id      The CAN id of this THMP
  //----------------------------------------------------------------------------
  int drvAsynTHMPConfigure( const char *portName, const char *RPiCanPort,
                            const int can_id ) {
    if ( can_id < 0x700 || can_id > 0x7ff ) {
      printf("ERROR: Could not configure drvAsynTHMP: invalid CAN id: %x", can_id );
      return( asynError );
    }
    new drvAsynTHMP( portName, RPiCanPort, can_id );
    return( asynSuccess );
  }
  static const iocshArg initThmpArg0 = { "portName",   iocshArgString };
  static const iocshArg initThmpArg1 = { "RPiCanPort", iocshArgString };
  static const iocshArg initThmpArg2 = { "can_id",     iocshArgInt };
  static const iocshArg * const initThmpArgs[] = { &initThmpArg0, &initThmpArg1, &initThmpArg2 };
  static const iocshFuncDef initThmpFuncDef = { "drvAsynTHMPConfigure", 3, initThmpArgs };
  static void initThmpCallFunc( const iocshArgBuf *args ) {
    drvAsynTHMPConfigure( args[0].sval, args[1].sval, args[2].ival );
  }

  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynLedPulser class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  RPiCanPort  The name of the interface 
  //!          [in]  can_id      The CAN id of this Led Pulser
  //!          [in]  filename    The Name of the file containing DAC and Intensity values
  //----------------------------------------------------------------------------
  int drvAsynLedPulserConfigure( const char *portName, const char *RPiCanPort,
                                 const int can_id, const char *filename ) {
    new drvAsynLedPulser( portName, RPiCanPort, can_id, filename );
    return( asynSuccess );
  }
  static const iocshArg initLedPulserArg0 = { "portName",   iocshArgString };
  static const iocshArg initLedPulserArg1 = { "RPiCanPort", iocshArgString };
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
  //!          for the drvAsynTmcm142 class.
  //!
  //! @param   [in]  portName    The name of the asyn port driver to be created.
  //!          [in]  RPiCanPort  The name of the interface 
  //!          [in]  can_id_w    The CAN id of this TMCM142 driver
  //!          [in]  can_id_r    The CAN Reply id of this TMCM142 driver
  //----------------------------------------------------------------------------
  int drvAsynTmcm142Configure( const char *portName, const char *RPiCanPort,
                               const int can_id_w, const int can_id_r ) {
    new drvAsynTmcm142( portName, RPiCanPort, can_id_w, can_id_r );
    return( asynSuccess );
  }
  static const iocshArg initTmcm142Arg0 = { "portName",   iocshArgString };
  static const iocshArg initTmcm142Arg1 = { "RPiCanPort", iocshArgString };
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
      iocshRegister( &initIsegHvFuncDef,       initIsegHvCallFunc );
      iocshRegister( &initIsegHvGlobalFuncDef, initIsegHvGlobalCallFunc );
      iocshRegister( &initWienerVmeFuncDef,    initWienerVmeCallFunc );
      iocshRegister( &initThmpFuncDef,         initThmpCallFunc );
      iocshRegister( &initLedPulserFuncDef,    initLedPulserCallFunc );
      iocshRegister( &initTmcm142FuncDef,      initTmcm142CallFunc );
      firstTime = 0;
    }
  }
  
  epicsExportRegistrar( drvAsynCanRegister );
}
