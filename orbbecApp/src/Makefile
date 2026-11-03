TOP=../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += ADOrbbec
LIB_SRCS += orbbec_driver.cpp

DBD += orbbecSupport.dbd

USR_INCLUDES += -I$(ORBBEC_SDK)/include -I$(ORBBEC_SDK)/build/src/generated
USR_LDFLAGS += -L$(ORBBEC_SDK)/build/linux_x86_64/lib
LIB_SYS_LIBS += OrbbecSDK

include $(ADCORE)/ADApp/commonDriverMakefile

include $(TOP)/configure/RULES
