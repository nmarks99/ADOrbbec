# ../../bin/${EPICS_HOST_ARCH}/orbbecTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocorbbecTestLinux.dbd")
iocorbbecTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "orbbecTest:")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(EPICS_DB_INCLUDE_PATH=.):$(ADCORE)/db")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

OrbbecConfig("ORBBEC1", "", 0, 0, 0, 0)
dbLoadRecords("$(ADORBBEC)/db/ADOrbbec.template", "P=$(PREFIX),R=cam1:,PORT=ORBBEC1")
dbLoadRecords("$(ADORBBEC)/db/ADOrbbecArray.template", "P=$(PREFIX),R=cam1:Color:,PORT=ORBBEC1,ADDR=0")
dbLoadRecords("$(ADORBBEC)/db/ADOrbbecArray.template", "P=$(PREFIX),R=cam1:Depth:,PORT=ORBBEC1,ADDR=1")

# Color image (RGB)
NDStdArraysConfigure("Image1", 5, 0, "ORBBEC1", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1,NDARRAY_ADDR=0,TYPE=Int8,FTVL=UCHAR,NELEMENTS=768000")
NDPvaConfigure("PVA1", "$(QSIZE=20)", 0, "ORBBEC1", 0, "$(PREFIX)Pva1:Image", 0, 0, 0)
dbLoadRecords("NDPva.template",  "P=$(PREFIX),R=Pva1:, PORT=PVA1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1,NDARRAY_ADDR=0")

# Depth image (mm)
NDStdArraysConfigure("Image2", 5, 0, "ORBBEC1", 1, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1,NDARRAY_ADDR=1,TYPE=Int16,FTVL=USHORT,NELEMENTS=256000")
NDPvaConfigure("PVA2", "$(QSIZE=20)", 0, "ORBBEC1", 1, "$(PREFIX)Pva2:Image", 0, 0, 0)
dbLoadRecords("NDPva.template",  "P=$(PREFIX),R=Pva2:, PORT=PVA2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ORBBEC1,NDARRAY_ADDR=1")
startPVAServer

###############################################################################
iocInit
###############################################################################

dbpf "orbbecTest:Pva1:EnableCallbacks" 1
dbpf "orbbecTest:Pva2:EnableCallbacks" 1
dbpf "orbbecTest:cam1:AcquirePeriod" 1.0

# print the time our boot was finished
date
