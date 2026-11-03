# ../../bin/${EPICS_HOST_ARCH}/orbbecTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocorbbecTestLinux.dbd")
iocorbbecTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "orbbecTest:")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(EPICS_DB_INCLUDE_PATH=.):$(ADCORE)/db")
OrbbecConfig("ORBBEC1", "164.54.104.159", 0, 0, 0, 0)
dbLoadRecords("$(ADORBBEC)/db/ADOrbbec.template", "P=$(PREFIX),R=cam1:,PORT=ORBBEC1")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
