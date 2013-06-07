TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
DBD += drvAsynCan.dbd

LIBRARY_IOC += drvAsynCan
ifeq ($(T_A), linux-arm)
  drvAsynCan_SRCS += drvAsynRPiCan.cpp
else
  drvAsynCan_SRCS += drvAsynPeakCan.cpp
endif
drvAsynCan_SRCS += drvAsynIsegHvModule.cpp
drvAsynCan_SRCS += drvAsynIsegHvChannel.cpp
#drvAsynCan_SRCS += drvAsynIsegHv.cpp
drvAsynCan_SRCS += drvAsynIsegHvGlobal.cpp
drvAsynCan_SRCS += drvAsynWienerVME.cpp
drvAsynCan_SRCS += drvAsynTHMP.cpp
drvAsynCan_SRCS += drvAsynLedPulser.cpp
drvAsynCan_SRCS += drvAsynTmcm142.cpp
drvAsynCan_SRCS += ReadPoller.cpp
drvAsynCan_SRCS += registerIOCfunctions.cpp
drvAsynCan_LIBS += asyn
drvAsynCan_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

#PROD_IOC += drvAsynCanTest

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
#drvAsynCanTest_SRCS_DEFAULT += drvAsynCan_registerRecordDeviceDriver.cpp AsynRPiCanMain.cpp

#drvAsynCanTest_LIBS += drvAsynCan asyn
#drvAsynCanTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
