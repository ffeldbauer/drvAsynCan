TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += drvAsynCan.dbd

LIBRARY_IOC += drvAsynCan

drvAsynCan_SRCS += drvAsynCan.cpp
drvAsynCan_SRCS += can_frame.cpp
drvAsynCan_SRCS += drvAsynCapacitec.cpp
drvAsynCan_SRCS += drvAsynIsegEhsEds.cpp
drvAsynCan_SRCS += drvAsynIsegHvGlobal.cpp
drvAsynCan_SRCS += drvAsynIsegEch44a.cpp
ifeq ($(USE_WIENERVME),yes)
drvAsynCan_SRCS += drvAsynWienerVME.cpp
endif
drvAsynCan_SRCS += drvAsynTHMP.cpp
drvAsynCan_SRCS += drvAsynLedPulser.cpp
drvAsynCan_SRCS += drvAsynLedPulser2.cpp
drvAsynCan_SRCS += drvAsynTmcm142.cpp
drvAsynCan_SRCS += ReadPoller.cpp
drvAsynCan_SRCS += registerIOCfunctions.cpp

drvAsynCan_LIBS += asyn
drvAsynCan_LIBS += $(EPICS_BASE_IOC_LIBS)

ifdef $(USE_WIENERVME)
	@echo "Building Wiener VME driver"
endif
#ifneq ($(words $(LIBSOCKETCAN_LIB) $(LIBSOCKETCAN_INCLUDE)),0)
#drvAsynCan_CXXFLAGS += -DUSELIBSOCKETCAN
#drvAsynCan_INCLUDES += -I$(LIBSOCKETCAN_INCLUDE)
#drvAsynCan_SYS_LIBS += socketcan
#SHRLIB_DEPLIB_DIRS  += $(LIBSOCKETCAN_LIB)
#endif

#=============================

#PROD_IOC += drvAsynCanTest

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
#drvAsynCanTest_SRCS_DEFAULT += drvAsynCan_registerRecordDeviceDriver.cpp AsynCanMain.cpp

#drvAsynCanTest_LIBS += drvAsynCan asyn
#drvAsynCanTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
