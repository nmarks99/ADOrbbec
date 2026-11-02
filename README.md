# ADOrbbec

Minimal EPICS areaDetector driver support for Orbbec cameras.

This first implementation connects to one selected camera during `OrbbecConfig` and publishes device identity only. It does not acquire or publish images yet.

## Configuration

```text
OrbbecConfig(portName, deviceId, maxBuffers, maxMemory, priority, stackSize)
```

`deviceId` may be empty when exactly one camera is visible. Otherwise use the camera IPv4 address, serial number, or UID.

The SDK location is configured with `ORBBEC_SDK`; the local development default is in `configure/RELEASE.local`.

The SDK shared library, `OrbbecSDKConfig.xml`, and its `extensions/` directory must be available at runtime. The example startup script is in `iocs/orbbecTest/iocBoot/iocorbbecTest/st.cmd`.
