# Depth Stream Acquisition Plan

## Scope

Implement the first working Orbbec acquisition path for a single depth stream.

Writing `1` to `ADAcquire` will start acquisition. Writing `0` will stop it. Frames will be published at asyn address 0 as two-dimensional `NDUInt16`, `NDColorModeMono` NDArrays.

The implementation will initially use the default SDK depth profile selected by:

```cpp
config->enableVideoStream(OB_STREAM_DEPTH);
```

This milestone does not add profile-selection PVs, color or IR streams, exposure controls, reconnection, alignment, or SDK format conversion.

## Reference Patterns

### areaDetector

Reference: `/home/beams0/NMARKS/devel/orbbec/ADTimePix3/tpx3App/src/ADTimePix.cpp`

Useful patterns:

- Override `writeInt32()` and handle `ADAcquire` separately from ordinary parameter writes.
- Publish acquisition state through `ADAcquire`, `ADStatus`, and `ADStatusMessage`.
- Make repeated start and stop requests idempotent.
- Roll back all acquisition state if startup fails.
- Call parameter callbacks after state changes.

ADTimePix3 has substantially more acquisition coordination than this driver currently needs. Do not copy its coordinator, HTTP monitoring, or stream-worker framework. Use the standard areaDetector start-event, stop-event, and dedicated acquisition-thread pattern.

### Orbbec SDK

Reference: `/home/beams0/NMARKS/devel/orbbec/OrbbecSDK_v2/examples/1.stream.depth/depth.cpp`

Required SDK sequence:

1. Construct an `ob::Pipeline` bound to the already selected `ob::Device`.
2. Construct an `ob::Config`.
3. Enable the default depth stream with `enableVideoStream(OB_STREAM_DEPTH)`.
4. Start the pipeline with the configuration.
5. Repeatedly call `waitForFrameset(timeoutMs)`.
6. Extract `OB_FRAME_DEPTH` from each frame set.
7. Convert it to `ob::DepthFrame`.
8. Read width, height, format, data size, frame index, timestamps, and value scale.
9. Stop the pipeline when acquisition finishes or is cancelled.

## Design

### Threading model

Create one acquisition thread in the driver constructor. The thread lives for the lifetime of the driver.

Add two EPICS events:

- `start_event_`: wakes the idle acquisition thread after `ADAcquire=1`.
- `stop_event_`: notifies acquisition waits and stop paths after `ADAcquire=0`.

The acquisition thread will:

1. Wait on `start_event_` while idle.
2. Read the requested image mode and image count under the driver lock.
3. Reset acquisition counters and publish the starting state.
4. Release the driver lock.
5. Create/configure/start the SDK pipeline.
6. Wait for depth frames with a bounded timeout.
7. Reacquire the driver lock to publish each frame and update parameters.
8. Check stop and image-mode completion conditions.
9. Release the lock before the next blocking SDK call.
10. Stop the pipeline without holding the driver lock.
11. Publish the final idle, aborted, or error state.

Do not call `waitForFrameset()` or `Pipeline::stop()` while holding the driver lock. A finite wait timeout, initially 100 ms, ensures a stop request is observed promptly even if no frames arrive.

Keep pipeline ownership in the acquisition thread. This avoids concurrent `start()`, `waitForFrameset()`, and `stop()` calls from `writeInt32()` and the worker.

### `writeInt32()` behavior

Add:

```cpp
asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
```

Behavior for `ADAcquire`:

#### Start request: `value != 0`

- Read the current `ADAcquire` value before changing it.
- If acquisition is already requested, return success without starting a second pipeline.
- Set `ADAcquire=1` through the parameter library.
- Set `ADStatus=ADStatusWaiting` or `ADStatusAcquire` with a clear status message. Prefer `ADStatusWaiting` until `Pipeline::start()` succeeds.
- Reset `ADNumImagesCounter` to zero.
- Clear `ORBBEC_LAST_ERROR`.
- Call parameter callbacks.
- Signal `start_event_`.
- Return promptly. Do not start the pipeline or wait for a frame in the asyn write callback.

#### Stop request: `value == 0`

- Set `ADAcquire=0` through the parameter library.
- If acquisition was active or starting, set `ADStatus=ADStatusAborting` and an appropriate message.
- Call parameter callbacks.
- Signal `stop_event_` so timed waits or future delay waits can terminate promptly.
- Return promptly. The acquisition thread owns pipeline shutdown and publishes the final state.

#### Other integer parameters

- Delegate to `ADDriver::writeInt32()`.
- Do not duplicate base-class handling.

The implementation must preserve an error returned by `ADDriver::writeInt32()` or parameter-library operations.

### Driver members

Add the minimum state needed by the first acquisition implementation:

```cpp
epicsEventId start_event_;
epicsEventId stop_event_;
epicsThreadId acquisition_thread_;
bool shutting_down_;
```

Add private methods similar to:

```cpp
static void acquisition_task_c(void* driver);
void acquisition_task();
asynStatus publish_depth_frame(const std::shared_ptr<ob::DepthFrame>& frame);
void publish_acquisition_error(const std::string& message);
```

Include the required EPICS thread/event headers and Orbbec pipeline/frame headers.

If the driver is destructible, add a destructor that:

1. Sets `shutting_down_` under the driver lock.
2. Sets `ADAcquire=0`.
3. Signals both events.
4. Waits for or joins the acquisition thread if the EPICS thread API and configured thread type support joining.
5. Destroys events only after the thread can no longer access them.

Do not add partial teardown that can free events while the acquisition thread is running. If safe joinable teardown cannot be completed in this milestone, document the limitation and do not destroy live synchronization objects.

### Pipeline startup

For each acquisition run:

```cpp
auto pipeline = std::make_shared<ob::Pipeline>(device_);
auto config = std::make_shared<ob::Config>();
config->enableVideoStream(OB_STREAM_DEPTH);
pipeline->start(config);
```

Create the pipeline only after confirming `device_` is valid. Catch `ob::Error` and `std::exception` around all pipeline and frame operations.

After `start()` succeeds:

- Set `ADStatus=ADStatusAcquire`.
- Set `ADStatusMessage="Acquiring depth frames"`.
- Keep `ADAcquire=1`.
- Call parameter callbacks.

On startup failure:

- Set `ADAcquire=0`.
- Set `ADStatus=ADStatusError`.
- Set `ADStatusMessage` and `ORBBEC_LAST_ERROR` to the exception details.
- Call parameter callbacks.
- Return to waiting for the next start request.

### Frame validation

For every returned frame set:

1. Ignore a null frame set unless acquisition has been stopped.
2. Get `OB_FRAME_DEPTH`.
3. Ignore a frame set with no depth frame.
4. Cast to `ob::DepthFrame`.
5. Require a 16-bit uncompressed depth format supported by the initial implementation, expected to be `OB_FORMAT_Y16` or `OB_FORMAT_Z16`.
6. Read width and height and reject zero dimensions.
7. Compute `expected_bytes = width * height * sizeof(epicsUInt16)` with overflow-safe size arithmetic.
8. Require `frame->getData()` to be non-null.
9. Require `frame->getDataSize()` to be at least `expected_bytes`.

An unsupported format or malformed frame is an acquisition error, not data to reinterpret silently.

Confirm the actual default depth format on the Gemini 335Le during IOC validation. If it is not Y16/Z16, add an explicit supported profile or conversion as a separate change.

### NDArray publication

For a valid frame:

1. Allocate a two-dimensional array:

   ```cpp
   size_t dims[2] = {width, height};
   NDArray* array = pNDArrayPool->alloc(2, dims, NDUInt16, 0, nullptr);
   ```

2. Copy exactly `width * height * sizeof(epicsUInt16)` bytes from the SDK frame.
3. Increment `NDArrayCounter` and `ADNumImagesCounter`.
4. Set `array->uniqueId` to the incremented areaDetector array counter. Preserve the SDK frame index as an attribute because the SDK index is 64-bit and may not fit `NDUniqueId`.
5. Timestamp the array with EPICS host time using `updateTimeStamp(&array->epicsTS)` and derive `array->timeStamp` consistently with ADCore conventions.
6. Call `getAttributes(array->pAttributeList)`.
7. Add these initial attributes:
   - `OrbbecFrameIndex`, SDK frame index
   - `OrbbecDeviceTimestampUs`, `getTimeStampUs()`
   - `OrbbecSystemTimestampUs`, `getSystemTimeStampUs()`
   - `OrbbecDepthScale`, `getValueScale()`
   - `OrbbecDepthUnits`, string value `mm per raw unit`
   - `OrbbecPixelFormat`, SDK format name or numeric value
8. Update standard parameters:
   - `ADSizeX`, `ADSizeY`
   - `ADMaxSizeX`, `ADMaxSizeY` for the selected profile
   - `NDArraySizeX`, `NDArraySizeY`
   - `NDArraySize`
   - `NDNDimensions=2`
   - `NDDataType=NDUInt16`
   - `NDColorMode=NDColorModeMono`
   - `NDUniqueId`
   - `NDTimeStamp`
   - `NDArrayCounter`
   - `ADNumImagesCounter`
9. Call parameter callbacks.
10. If `NDArrayCallbacks` is enabled, call:

    ```cpp
    doCallbacksGenericPointer(array, NDArrayData, 0);
    ```

11. Call `array->release()` on every path after successful allocation, including callback-disabled and error paths.

Use a small RAII guard or one cleanup block if needed to guarantee release. Do not retain the SDK frame data pointer after processing. Copy it into the NDArray before the SDK frame goes out of scope.

Raw depth values remain unchanged. Physical distance in millimeters is:

```text
raw pixel value * OrbbecDepthScale
```

### Image modes

Read `ADImageMode` and `ADNumImages` when a run starts.

- `ADImageSingle`: stop after one successfully published depth frame.
- `ADImageMultiple`: stop after `ADNumImages` successfully published depth frames.
- `ADImageContinuous`: continue until `ADAcquire=0` or an error occurs.

Count only frames that were successfully copied and published as NDArrays. Null frame sets and frame sets without depth data do not increment counters.

On normal Single or Multiple completion:

- Stop the SDK pipeline.
- Set `ADAcquire=0`.
- Set `ADStatus=ADStatusIdle`.
- Set `ADStatusMessage="Acquisition complete"`.
- Call parameter callbacks.

On an operator stop:

- Stop the SDK pipeline.
- Set `ADAcquire=0`.
- Set `ADStatus=ADStatusIdle` or `ADStatusAborted`, following the conventions used by the installed ADCore version.
- Set `ADStatusMessage="Acquisition stopped"`.
- Call parameter callbacks.

### Error handling

Reuse or extend `set_error()` so acquisition exceptions publish:

- `ADAcquire=0`
- `ADStatus=ADStatusError`
- `ADStatusMessage`
- `ORBBEC_LAST_ERROR`
- `ASYN_TRACE_ERROR` output

For `ob::Error`, include:

- `what()`
- SDK function
- SDK arguments
- SDK status

After an error, make a best-effort `pipeline->stop()` outside the driver lock. Catch and log a stop exception without replacing the original error message.

A temporary 100 ms frame timeout is not itself an error. Continue waiting while `ADAcquire` remains set. A later change can add a configurable no-frame timeout.

## Implementation Order

1. **Complete naming consistency first**
   - Ensure source, header, library, DBD, IOC Makefile, and startup script consistently use the intended `ADOrbbec`/`orbbecSupport` names.
   - Do not mix the rename with unrelated refactoring.

2. **Add declarations and SDK/EPICS includes**
   - Declare `writeInt32()`.
   - Add acquisition events, thread state, entry point, worker, and frame-publication declarations.
   - Include `Pipeline.hpp`, `Frame.hpp`, EPICS event/thread/time headers, and `<cstring>`.

3. **Initialize standard parameters**
   - Set depth defaults: `NDUInt16`, `NDColorModeMono`, two dimensions, counters zero, and a default image mode.
   - Create events only after device initialization state is known.
   - Start the persistent acquisition thread.

4. **Implement `writeInt32()`**
   - Add the `ADAcquire` start/stop branches.
   - Delegate unrelated writes to `ADDriver::writeInt32()`.
   - Verify repeated writes do not create duplicate starts or invalid stops.

5. **Implement the acquisition worker**
   - Wait for start.
   - Construct the device-bound pipeline and depth config.
   - Start without holding the driver lock.
   - Wait for frames using a bounded timeout.
   - Observe stop requests and image-mode completion.
   - Stop the pipeline outside the lock.

6. **Implement depth-frame publication**
   - Validate format, dimensions, and byte size.
   - Allocate and fill `NDUInt16` NDArrays.
   - Add timestamps and attributes.
   - Update parameters and counters.
   - Publish and always release the array.

7. **Add robust rollback and shutdown**
   - Handle SDK and allocation errors.
   - Ensure all terminal paths clear `ADAcquire`.
   - Ensure pipeline stop is attempted exactly once per successful start.
   - Ensure driver destruction cannot race the worker.

8. **Validate in the IOC**
   - Perform a clean rebuild manually.
   - Start the IOC with the Gemini 335Le IP selector.
   - Enable `NDArrayCallbacks`.
   - Test Single, Multiple, Continuous, and operator stop.

## Verification Checklist

### Static review

- `writeInt32()` does not perform blocking SDK operations.
- Only the acquisition thread owns a running `ob::Pipeline`.
- The driver lock is released around `start()`, `waitForFrameset()`, and `stop()`.
- Every allocated NDArray is released exactly once.
- SDK frame data is copied before SDK frame destruction.
- Array dimensions are `[width, height]`.
- Depth data is `NDUInt16` and monochrome.
- Unsupported formats fail explicitly.
- Every acquisition terminal path sets `ADAcquire=0`.

### IOC tests

1. **Startup**
   - Device identity remains available.
   - `ADStatus` becomes Idle after connection.
   - No stream starts until `ADAcquire=1`.

2. **Single mode**
   - Set `ADImageMode=Single`.
   - Write `ADAcquire=1`.
   - Exactly one NDArray callback is observed.
   - `ADNumImagesCounter=1`.
   - `ADAcquire` returns to zero.
   - `ADStatus` returns to Idle.

3. **Multiple mode**
   - Set `ADImageMode=Multiple` and `ADNumImages=5`.
   - Exactly five arrays are published.
   - Counters reach five.
   - Acquisition stops automatically.

4. **Continuous mode**
   - Set `ADImageMode=Continuous`.
   - Arrays continue to arrive and counters increase.
   - Writing `ADAcquire=0` stops callbacks promptly.
   - The IOC remains responsive while waiting for frames.

5. **Array metadata**
   - `NDDataType=NDUInt16`.
   - `NDColorMode=Mono`.
   - Dimensions match the selected SDK profile.
   - `NDArraySize=width*height*2`.
   - Unique IDs and counters increase monotonically.
   - Depth-scale and SDK timestamp attributes are present.

6. **Error behavior**
   - Disconnecting or making the camera unavailable produces `ADStatusError` and a useful message.
   - `ADAcquire` returns to zero after failure.
   - A later start request can attempt acquisition again without restarting the IOC.

7. **Plugin path**
   - An `NDStdArrays` or display plugin receives the depth arrays.
   - A file plugin can write frames without pool exhaustion.
   - Pool usage remains bounded, confirming arrays are released.

## Environment Prerequisites

Before judging network-stream reliability:

- Increase the host UDP receive-buffer limit so the SDK can obtain its requested 128 MB RTP buffer.
- Note that camera firmware `1.6.00` is below the SDK warning threshold and below the ROS wrapper's recommended Gemini 335Le firmware. Firmware limitations must be distinguished from driver defects.

## Deferred Work

The following are explicitly outside this milestone:

- Configurable depth width, height, FPS, and format
- Exposure, gain, laser, and depth precision controls
- Color and IR streams
- Multiple asyn addresses
- Depth-to-color alignment and frame synchronization
- SDK global timestamp enablement
- Configurable no-frame timeout
- Device-added/device-removed callbacks and automatic reconnect
- Zero-copy publication
