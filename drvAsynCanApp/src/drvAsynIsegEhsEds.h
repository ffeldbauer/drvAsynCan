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
// brief   Asyn driver for ISEG EHS/EDS high voltage modules
//
// version 3.0.0; Jul. 29, 2014
//******************************************************************************
//! @file 
#ifndef __ASYN_ISEG_EHS_EDS_H__
#define __ASYN_ISEG_EHS_EDS_H__

//_____ I N C L U D E S _______________________________________________________
#include <map>
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

//! @defgroup IsegEhsEdsParameters
//! These are the drvInfo strings that are used to identify the parameters.
//! They are used by asyn clients, including standard asyn device support
//! @{
#define P_ISEGEHSEDS_CHANSTATUS_STRING         "ChannelStatus"            //!< asynUInt32Digital,  r   
#define P_ISEGEHSEDS_CHAN_CTRL_STRING          "ChannelControl"           //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_CHANEVTSTATUS_STRING      "ChannelEventStatus"       //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_CHANEVTMASK_STRING        "ChannelEventMask"         //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_VSET_STRING               "VoltageSet"               //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_ISET_STRING               "CurrentSet"               //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_VMOM_STRING               "VoltageMeasure"           //!< asynFloat64,        r   
#define P_ISEGEHSEDS_IMOM_STRING               "CurrentMeasure"           //!< asynFloat64,        r   
#define P_ISEGEHSEDS_VBOUNDS_STRING            "VoltageBounds"            //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_IBOUNDS_STRING            "CurrentBounds"            //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_VNOM_STRING               "VoltageNominal"           //!< asynFloat64,        r   
#define P_ISEGEHSEDS_INOM_STRING               "CurrentNominal"           //!< asynFloat64,        r   
#define P_ISEGEHSEDS_IMOM_RANGE_STRING         "CurrentMeasureRange"      //!< asynUInt32Digital,  r   

#define P_ISEGEHSEDS_R_CHANSTATUS_STRING       "ReadChannelStatus"        //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_CHAN_CTRL_STRING        "ReadChannelControl"       //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_CHANEVTSTATUS_STRING    "ReadChannelEventStatus"   //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_CHANEVTMASK_STRING      "ReadChannelEventMask"     //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_VSET_STRING             "ReadVoltageSet"           //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_ISET_STRING             "ReadCurrentSet"           //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_VMOM_STRING             "ReadVoltageMeasure"       //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_IMOM_STRING             "ReadCurrentMeasure"       //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_VBOUNDS_STRING          "ReadVoltageBounds"        //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_IBOUNDS_STRING          "ReadCurrentBounds"        //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_VNOM_STRING             "ReadVoltageNominal"       //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_INOM_STRING             "ReadCurrentNominal"       //!< asynInt32,          r/w 
#define P_ISEGEHSEDS_R_IMOM_RANGE_STRING       "ReadCurrentMeasureRange"  //!< asynInt32,          r/w 

#define P_ISEGEHSEDS_MODSTATUS_STRING          "ModuleStatus"             //!< asynUInt32Digital,  r   
#define P_ISEGEHSEDS_MODCTRL_STRING            "ModuleControl"            //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTSTATUS_STRING       "ModuleEventStatus"        //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTMASK_STRING         "ModuleEventMask"          //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTCHANSTATUS_STRING   "ModuleEventChannelStatus" //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTCHANMASK_STRING     "ModuleEventChannelMask"   //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTGRPSTATUS_STRING    "ModuleEventGroupStatus"   //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_MODEVTGRPMASK_STRING      "ModuleEventGroupMask"     //!< asynUInt32Digital,  r/w 
#define P_ISEGEHSEDS_VRAMPSPEED_STRING         "VoltageRampSpeed"         //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_IRAMPSPEED_STRING         "CurrentRampSpeed"         //!< asynFloat64,        r/w 
#define P_ISEGEHSEDS_VMAX_STRING               "VoltageMax"               //!< asynFloat64,        r 
#define P_ISEGEHSEDS_IMAX_STRING               "CurrentMax"               //!< asynFloat64,        r 
#define P_ISEGEHSEDS_SUPPLY24_STRING           "Supply24"                 //!< asynFloat64,        r   
#define P_ISEGEHSEDS_SUPPLY5_STRING            "Supply5"                  //!< asynFloat64,        r   
#define P_ISEGEHSEDS_TEMPERATURE_STRING        "BoardTemperature"         //!< asynFloat64,        r

//! @}

//! @brief   asynPortDriver for ISEG EHS/EDS high voltage modules
//!
//! This asynPortDriver is a higher level driver used as device support for the
//! EHS/EDS high voltage modules of ISEG Spezialelektronik GmbH.\n
//! It needs a lower level driver with a asynGenericPointer interface for
//! accessing the hardware of the CAN bus interface.
class drvAsynIsegEhsEds : public asynPortDriver {
 public:
  drvAsynIsegEhsEds( const char *portName, const char *CanPort, const int module_id, const int channels );

  // These are the methods that we override from asynPortDriver
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
  virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );
  virtual asynStatus getBoundsFloat( asynUser *pasynUser, epicsFloat64 *low, epicsFloat64 *high );

  void asynReadHandler( void* pframe );

 protected:
  // Values used for pasynUser->reason, and indexes into the parameter library.
  int P_ChanStatus;          //!< index of Parameter "ChannelStatus"
  int P_ChanCtrl;            //!< index of Parameter "ChannelControl"
  int P_ChanEventStatus;     //!< index of Parameter "ChannelEventStatus"
  int P_ChanEventMask;       //!< index of Parameter "ChannelEventMask"
  int P_ChanVset;            //!< index of Parameter "VoltageSet"
  int P_ChanIset;            //!< index of Parameter "CurrentSet"
  int P_ChanVmom;            //!< index of Parameter "VoltageMeasure"
  int P_ChanImom;            //!< index of Parameter "CurrentMeasure"
  int P_ChanVbounds;         //!< index of Parameter "VoltageBounds"
  int P_ChanIbounds;         //!< index of Parameter "CurrentBounds"
  int P_ChanVnom;            //!< index of Parameter "VoltageNominal"
  int P_ChanInom;            //!< index of Parameter "CurrentNominal"
  int P_ChanImomRange;       //!< index of Parameter "CurrentMeasureRange"

  int P_R_ChanStatus;        //!< index of Parameter "ReadChannelStatus"
  int P_R_ChanCtrl;          //!< index of Parameter "ReadChannelControl"
  int P_R_ChanEventStatus;   //!< index of Parameter "ReadChannelEventStatus"
  int P_R_ChanEventMask;     //!< index of Parameter "ReadChannelEventMask"
  int P_R_ChanVset;          //!< index of Parameter "ReadVoltageSet"
  int P_R_ChanIset;          //!< index of Parameter "ReadCurrentSet"
  int P_R_ChanVmom;          //!< index of Parameter "ReadVoltageMeasure"
  int P_R_ChanImom;          //!< index of Parameter "ReadCurrentMeasure"
  int P_R_ChanVbounds;       //!< index of Parameter "ReadVoltageBounds"
  int P_R_ChanIbounds;       //!< index of Parameter "ReadCurrentBounds"
  int P_R_ChanVnom;          //!< index of Parameter "ReadVoltageNominal"
  int P_R_ChanInom;          //!< index of Parameter "ReadCurrentNominal"
  int P_R_ChanImomRange;     //!< index of Parameter "ReadCurrentMeasureRange"
  
  int P_ModStatus;           //!< index of Parameter "ModuleStatus"
  int P_ModCtrl;             //!< index of Parameter "ModuleControl"
  int P_ModEventStatus;      //!< index of Parameter "ModuleEventStatus"
  int P_ModEventMask;        //!< index of Parameter "ModuleEventMask"
  int P_ModEventChanStatus;  //!< index of Parameter "ModuleEventChannelStatus"
  int P_ModEventChanMask;    //!< index of Parameter "ModuleEventChannelMask"
  int P_ModEventGrpStatus;   //!< index of Parameter "ModuleEventGroupStatus"
  int P_ModEventGrpMask;     //!< index of Parameter "ModuleEventGroupMask"
  int P_VRampSpeed;          //!< index of Parameter "VoltageRampSpeed"
  int P_IRampSpeed;          //!< index of Parameter "CurrentRampSpeed"
  int P_Vmax;                //!< index of Parameter "VoltageMax"
  int P_Imax;                //!< index of Parameter "CurrentMax"
  int P_Supply24;            //!< index of Parameter "Supply24"
  int P_Supply5;             //!< index of Parameter "Supply5"
  int P_Temperature;         //!< index of Parameter "BoardTemperature"
#define FIRST_ISEGEHSEDS_COMMAND P_ChanStatus
#define LAST_ISEGEHSEDS_COMMAND  P_Temperature

 private:
  struct isegFrame {
    epicsUInt8    dlc;
    epicsUInt8    data0;
    epicsUInt8    data1;
  };

  asynStatus getFirmware();
  asynStatus initEhsEds( std::map<int, isegFrame>::const_iterator& it );
  asynStatus initEhsEdsModule( std::map<int, isegFrame>::const_iterator& it );

  char                     *_deviceName;
  epicsUInt32               _can_id;
  asynUser                 *_pasynUser;
  asynCommon               *_pasynCommon;
  void                     *_pvtCommon;
  asynGenericPointer       *_pasynGenericPointer;
  void                     *_pvtGenericPointer;
  void                     *_intrPvtGenericPointer;
  bool                      _shortMBR;
  epicsUInt8                _firmwareRelease[4];
  char                      _firmwareName[6];
  std::map<int, isegFrame>  _cmds;
};

#define NUM_ISEGEHSEDS_PARAMS (&LAST_ISEGEHSEDS_COMMAND - &FIRST_ISEGEHSEDS_COMMAND + 1)

#endif

