# ADOrbbec

Relies on Orbbec SDK which is included as a git submodule but must be built manually
with cmake first before building ADOrbbec. See orbbecSupport/README.md

Port 0: RGB color image
Port 1: Depth

---

**ADOrbbec.template**
```
stringin  $(P)$(R)DeviceSelector_RBV
stringin  $(P)$(R)DeviceUID_RBV
stringin  $(P)$(R)DeviceIP_RBV
stringin  $(P)$(R)ConnectionType_RBV
longin    $(P)$(R)DeviceVID_RBV
longin    $(P)$(R)DevicePID_RBV
stringin  $(P)$(R)LastError_RBV
bo        $(P)$(R)Depth:AlignToColor
bi        $(P)$(R)Depth:AlignToColor_RBV
```

**ADOrbbecArray.template**
```
longin  $(P)$(R)ArraySizeX_RBV
longin  $(P)$(R)ArraySizeY_RBV
longin  $(P)$(R)ArraySizeZ_RBV
longin  $(P)$(R)ArraySize_RBV
longin  $(P)$(R)NDimensions_RBV
mbbi    $(P)$(R)DataType_RBV
mbbi    $(P)$(R)ColorMode_RBV
longin  $(P)$(R)ArrayCounter_RBV
longin  $(P)$(R)UniqueId_RBV
ai      $(P)$(R)TimeStamp_RBV
ai      $(P)$(R)Intrinsic:Fx
ai      $(P)$(R)Intrinsic:Fy
ai      $(P)$(R)Intrinsic:Cx
ai      $(P)$(R)Intrinsic:Cy
ai      $(P)$(R)Intrinsic:K1
ai      $(P)$(R)Intrinsic:K2
ai      $(P)$(R)Intrinsic:K3
ai      $(P)$(R)Intrinsic:K4
ai      $(P)$(R)Intrinsic:K5
ai      $(P)$(R)Intrinsic:K6
ai      $(P)$(R)Intrinsic:P1
ai      $(P)$(R)Intrinsic:P2
mbbi    $(P)$(R)Intrinsic:DistortionModel
```
