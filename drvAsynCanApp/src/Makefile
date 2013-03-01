TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += drvAsynCan.dbd

drvAsynCan_DBD += base.dbd
drvAsynCan_DBD += asyn.dbd
drvAsynCan_DBD += drvAsynCan.dbd
ifeq ($(T_A), linux-arm)
drvAsynCan_DBD += drvAsynRPiCan.dbd
endif 

LIBRARY_IOC += drvAsynCanSupport

ifeq ($(T_A), linux-arm)
drvAsynCanSupport_SRCS += drvAsynRPiCan.cpp
endif 
drvAsynCanSupport_SRCS += drvAsynIsegHv.cpp
drvAsynCanSupport_SRCS += drvAsynIsegHvGlobal.cpp
drvAsynCanSupport_SRCS += drvAsynWienerVME.cpp
drvAsynCanSupport_SRCS += drvAsynTHMP.cpp
drvAsynCanSupport_SRCS += drvAsynLedPulser.cpp
drvAsynCanSupport_SRCS += drvAsynTmcm142.cpp
drvAsynCanSupport_SRCS += registerIOCfunctions.cpp
drvAsynCanSupport_LIBS += asyn
drvAsynCanSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

PROD_IOC += drvAsynCan

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
drvAsynCan_SRCS_DEFAULT += drvAsynCan_registerRecordDeviceDriver.cpp AsynRPiCanMain.cpp

drvAsynCan_LIBS += drvAsynCanSupport asyn
drvAsynCan_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
