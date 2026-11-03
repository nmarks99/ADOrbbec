TOP=../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += ADOrbbec
LIB_SRCS += orbbec_driver.cpp

DBD += orbbecSupport.dbd

USR_INCLUDES += -I$(ORBBEC_SDK)/include
USR_LDFLAGS += -L$(ORBBEC_SDK)/lib
LIB_SYS_LIBS += OrbbecSDK

include $(ADCORE)/ADApp/commonDriverMakefile

include $(TOP)/configure/RULES
