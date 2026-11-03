# ../../bin/${EPICS_HOST_ARCH}/orbbecTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocorbbecTestLinux.dbd")
iocorbbecTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "orbbecTest:")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(EPICS_DB_INCLUDE_PATH=.):$(ADCORE)/db")

OrbbecConfig("ORBBEC1", "164.54.104.159", 0, 0, 0, 0)
dbLoadRecords("$(ADORBBEC)/db/ADOrbbec.template", "P=$(PREFIX),R=cam1:,PORT=ORBBEC1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "ORBBEC1", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1,TYPE=Int16,FTVL=SHORT,NELEMENTS=256000")

# Load NDPluginPva plugin
NDPvaConfigure("PVA1", "$(QSIZE=20)", 0, "ORBBEC1", 0, "$(PREFIX)Pva1:Image", 0, 0, 0)
dbLoadRecords("NDPva.template",  "P=$(PREFIX),R=Pva1:, PORT=PVA1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1")
startPVAServer

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
