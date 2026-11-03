#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <epicsExport.h>
#include <epicsString.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Version.hpp>

#include "orbbec_driver.hpp"

static const char* driver_name = "OrbbecDriver";
static const char* driver_version = "0.1.0";

// class OrbbecDriver : public ADDriver {
  // public:
    // OrbbecDriver(const char* port_name, const char* device_id, int max_buffers, size_t max_memory,
                 // int priority, int stack_size);
//
    // void report(FILE* fp, int details) override;
//
  // protected:
    // int device_selector_;
    // int device_uid_;
    // int device_ip_;
    // int connection_type_;
    // int device_vid_;
    // int device_pid_;
    // int last_error_;
//
  // private:
    // asynStatus connect_device(const char* device_id);
    // void set_error(const std::string& message);
//
    // std::shared_ptr<ob::Context> context_;
    // std::shared_ptr<ob::Device> device_;
// };

void OrbbecDriver::set_error(const std::string& message) {
    setIntegerParam(ADAcquire, 0);
    setStringParam(last_error_, message.c_str());
    setStringParam(ADStatusMessage, message.c_str());
    setIntegerParam(ADStatus, ADStatusError);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: %s\n", driver_name, message.c_str());
}

void OrbbecDriver::acquisition_task_c(void* driver) {
    static_cast<OrbbecDriver*>(driver)->acquisition_task();
}

asynStatus OrbbecDriver::publish_depth_frame(const std::shared_ptr<ob::DepthFrame>& frame) {
    const OBFormat format = frame->getFormat();
    if (format != OB_FORMAT_Y16 && format != OB_FORMAT_Z16) {
        char message[128];
        epicsSnprintf(message, sizeof(message), "Unsupported depth format: %d", static_cast<int>(format));
        set_error(message);
        return asynError;
    }

    const size_t width = frame->getWidth();
    const size_t height = frame->getHeight();
    if (width == 0 || height == 0 || width > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        width > std::numeric_limits<size_t>::max() / height ||
        width * height > std::numeric_limits<size_t>::max() / sizeof(epicsUInt16) ||
        width * height * sizeof(epicsUInt16) > static_cast<size_t>(std::numeric_limits<int>::max())) {
        set_error("Invalid depth frame dimensions");
        return asynError;
    }

    const size_t data_size = width * height * sizeof(epicsUInt16);
    if (frame->getData() == nullptr || frame->getDataSize() < data_size) {
        set_error("Depth frame data is missing or shorter than its dimensions");
        return asynError;
    }

    size_t dimensions[2] = {width, height};
    NDArray* array = pNDArrayPool->alloc(2, dimensions, NDUInt16, 0, nullptr);
    if (array == nullptr) {
        set_error("Failed to allocate depth NDArray");
        return asynError;
    }

    std::unique_ptr<NDArray, void (*)(NDArray*)> array_guard(array, [](NDArray* value) { value->release(); });
    std::memcpy(array->pData, frame->getData(), data_size);

    int array_counter = 0;
    int image_counter = 0;
    getIntegerParam(NDArrayCounter, &array_counter);
    getIntegerParam(ADNumImagesCounter, &image_counter);
    ++array_counter;
    ++image_counter;

    updateTimeStamp(&array->epicsTS);
    array->timeStamp = array->epicsTS.secPastEpoch + array->epicsTS.nsec / 1.e9;
    array->uniqueId = array_counter;

    getAttributes(array->pAttributeList);
    epicsUInt64 frame_index = frame->getIndex();
    epicsUInt64 device_timestamp = frame->getTimeStampUs();
    epicsUInt64 system_timestamp = frame->getSystemTimeStampUs();
    epicsFloat32 depth_scale = frame->getValueScale();
    epicsInt32 pixel_format = static_cast<epicsInt32>(format);
    char depth_units[] = "mm per raw unit";
    array->pAttributeList->add("OrbbecFrameIndex", "Orbbec SDK frame index", NDAttrUInt64, &frame_index);
    array->pAttributeList->add("OrbbecDeviceTimestampUs", "Device timestamp in microseconds", NDAttrUInt64,
                               &device_timestamp);
    array->pAttributeList->add("OrbbecSystemTimestampUs", "Host timestamp in microseconds", NDAttrUInt64,
                               &system_timestamp);
    array->pAttributeList->add("OrbbecDepthScale", "Millimeters per raw depth unit", NDAttrFloat32,
                               &depth_scale);
    array->pAttributeList->add("OrbbecDepthUnits", "Depth scale units", NDAttrString, depth_units);
    array->pAttributeList->add("OrbbecPixelFormat", "Orbbec SDK pixel format", NDAttrInt32, &pixel_format);

    setIntegerParam(ADSizeX, static_cast<int>(width));
    setIntegerParam(ADSizeY, static_cast<int>(height));
    setIntegerParam(ADMaxSizeX, static_cast<int>(width));
    setIntegerParam(ADMaxSizeY, static_cast<int>(height));
    setIntegerParam(NDArraySizeX, static_cast<int>(width));
    setIntegerParam(NDArraySizeY, static_cast<int>(height));
    setIntegerParam(NDArraySize, static_cast<int>(data_size));
    setIntegerParam(NDNDimensions, 2);
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDUniqueId, array_counter);
    setDoubleParam(NDTimeStamp, array->timeStamp);
    setIntegerParam(NDArrayCounter, array_counter);
    setIntegerParam(ADNumImagesCounter, image_counter);
    callParamCallbacks();

    int array_callbacks = 0;
    getIntegerParam(NDArrayCallbacks, &array_callbacks);
    if (array_callbacks) {
        unlock();
        doCallbacksGenericPointer(array, NDArrayData, 0);
        lock();
    }
    return asynSuccess;
}

void OrbbecDriver::acquisition_task() {
    while (true) {
        epicsEventWait(start_event_);

        lock();
        if (shutting_down_) {
            unlock();
            return;
        }

        int acquire = 0;
        int image_mode = ADImageContinuous;
        int num_images = 1;
        getIntegerParam(ADAcquire, &acquire);
        getIntegerParam(ADImageMode, &image_mode);
        getIntegerParam(ADNumImages, &num_images);
        if (!acquire) {
            unlock();
            continue;
        }
        while (epicsEventTryWait(stop_event_) == epicsEventWaitOK) {
        }
        acquisition_running_ = true;
        unlock();

        std::shared_ptr<ob::Pipeline> pipeline;
        bool pipeline_started = false;
        bool operator_stopped = false;
        bool acquisition_complete = false;
        std::string error_message;

        try {
            if (!device_) {
                throw std::runtime_error("No Orbbec device is connected");
            }
            pipeline = std::make_shared<ob::Pipeline>(device_);
            auto config = std::make_shared<ob::Config>();
            config->enableVideoStream(OB_STREAM_DEPTH);
            pipeline->start(config);
            pipeline_started = true;

            lock();
            getIntegerParam(ADAcquire, &acquire);
            if (acquire && !shutting_down_) {
                setIntegerParam(ADStatus, ADStatusAcquire);
                setStringParam(ADStatusMessage, "Acquiring depth frames");
                callParamCallbacks();
            } else {
                operator_stopped = !shutting_down_;
            }
            unlock();

            while (acquire && !shutting_down_) {
                std::shared_ptr<ob::FrameSet> frame_set = pipeline->waitForFrameset(100);

                const bool stop_requested = epicsEventTryWait(stop_event_) == epicsEventWaitOK;
                lock();
                getIntegerParam(ADAcquire, &acquire);
                if (stop_requested || !acquire || shutting_down_) {
                    operator_stopped = !shutting_down_;
                    unlock();
                    break;
                }
                unlock();

                if (!frame_set) {
                    continue;
                }
                std::shared_ptr<ob::Frame> depth_raw = frame_set->getFrame(OB_FRAME_DEPTH);
                if (!depth_raw) {
                    continue;
                }
                std::shared_ptr<ob::DepthFrame> depth_frame = depth_raw->as<ob::DepthFrame>();

                lock();
                asynStatus status;
                try {
                    status = publish_depth_frame(depth_frame);
                } catch (...) {
                    unlock();
                    throw;
                }
                int image_counter = 0;
                getIntegerParam(ADNumImagesCounter, &image_counter);
                getIntegerParam(ADAcquire, &acquire);
                const bool complete = image_mode == ADImageSingle ||
                                      (image_mode == ADImageMultiple &&
                                       image_counter >= std::max(1, num_images));
                unlock();

                if (status != asynSuccess) {
                    char message[512];
                    getStringParam(last_error_, sizeof(message), message);
                    error_message = message[0] != '\0' ? message : "Failed to publish depth frame";
                    break;
                }
                if (complete) {
                    acquisition_complete = true;
                    break;
                }
            }
            if (!acquire && !acquisition_complete && !shutting_down_) {
                operator_stopped = true;
            }
        } catch (const ob::Error& error) {
            char message[512];
            epicsSnprintf(message, sizeof(message), "SDK error: %s; function=%s; arguments=%s; status=%d",
                          error.what(), error.getFunction(), error.getArgs(), error.getStatus());
            error_message = message;
        } catch (const std::exception& error) {
            error_message = std::string("Depth acquisition error: ") + error.what();
        }

        if (pipeline_started) {
            try {
                pipeline->stop();
            } catch (const std::exception& error) {
                if (error_message.empty()) {
                    error_message = std::string("Failed to stop depth pipeline: ") + error.what();
                } else {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: pipeline stop failed: %s\n", driver_name,
                              error.what());
                }
            }
        }

        lock();
        acquisition_running_ = false;
        if (!error_message.empty()) {
            set_error(error_message);
        } else if (!shutting_down_) {
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, operator_stopped ? ADStatusAborted : ADStatusIdle);
            setStringParam(ADStatusMessage, operator_stopped ? "Acquisition stopped" : "Acquisition complete");
        }
        callParamCallbacks();
        unlock();
    }
}

asynStatus OrbbecDriver::connect_device(const char* device_id) {
    try {
        context_ = std::make_shared<ob::Context>();
        std::string selector = device_id ? device_id : "";

        if (selector.empty()) {
            std::shared_ptr<ob::DeviceList> devices = context_->queryDeviceList();
            if (devices->getCount() != 1) {
                char message[128];
                epicsSnprintf(message, sizeof(message),
                              "Expected exactly one Orbbec device, found %u. Set deviceId explicitly.",
                              devices->getCount());
                set_error(message);
                return asynError;
            }
            device_ = devices->getDevice(0);
        } else {
            std::shared_ptr<ob::DeviceList> devices = context_->queryDeviceList();
            for (uint32_t index = 0; index < devices->getCount(); index++) {
                if (selector == devices->getSerialNumber(index) || selector == devices->getUid(index) ||
                    selector == devices->getIpAddress(index)) {
                    device_ = devices->getDevice(index);
                    break;
                }
            }
            if (!device_) {
                device_ = context_->createNetDevice(selector.c_str(), 8090);
            }
        }

        std::shared_ptr<ob::DeviceInfo> info = device_->getDeviceInfo();
        char sdk_version[32];
        epicsSnprintf(sdk_version, sizeof(sdk_version), "%d.%d.%d", ob::Version::getMajor(),
                      ob::Version::getMinor(), ob::Version::getPatch());

        setStringParam(ADManufacturer, "Orbbec");
        setStringParam(ADModel, info->getName());
        setStringParam(ADSerialNumber, info->getSerialNumber());
        setStringParam(ADFirmwareVersion, info->getFirmwareVersion());
        setStringParam(ADSDKVersion, sdk_version);
        setStringParam(NDDriverVersion, driver_version);
        setStringParam(device_uid_, info->getUid());
        setStringParam(device_ip_, info->getIpAddress());
        setStringParam(connection_type_, info->getConnectionType());
        setIntegerParam(device_vid_, info->getVid());
        setIntegerParam(device_pid_, info->getPid());
        setStringParam(last_error_, "");
        setStringParam(ADStatusMessage, "Connected");
        setIntegerParam(ADStatus, ADStatusIdle);
        return asynSuccess;
    } catch (const ob::Error& error) {
        char message[512];
        epicsSnprintf(message, sizeof(message), "SDK error: %s; function=%s; arguments=%s; status=%d",
                      error.what(), error.getFunction(), error.getArgs(), error.getStatus());
        set_error(message);
    } catch (const std::exception& error) {
        set_error(std::string("Error connecting to device: ") + error.what());
    }
    return asynError;
}

constexpr unsigned int MAX_ADDR = 1;
constexpr unsigned int NUM_PARAMS = 7;

OrbbecDriver::OrbbecDriver(const char* port_name, const char* device_id, int max_buffers, size_t max_memory,
                           int priority, int stack_size)
    : ADDriver(port_name, MAX_ADDR, NUM_PARAMS, max_buffers, max_memory, asynGenericPointerMask,
               asynGenericPointerMask, ASYN_CANBLOCK, 1, priority, stack_size),
      start_event_(nullptr), stop_event_(nullptr), acquisition_thread_(nullptr), acquisition_running_(false),
      shutting_down_(false) {

    createParam("ORBBEC_DEVICE_SELECTOR", asynParamOctet, &device_selector_);
    createParam("ORBBEC_DEVICE_UID", asynParamOctet, &device_uid_);
    createParam("ORBBEC_DEVICE_IP", asynParamOctet, &device_ip_);
    createParam("ORBBEC_CONNECTION_TYPE", asynParamOctet, &connection_type_);
    createParam("ORBBEC_DEVICE_VID", asynParamInt32, &device_vid_);
    createParam("ORBBEC_DEVICE_PID", asynParamInt32, &device_pid_);
    createParam("ORBBEC_LAST_ERROR", asynParamOctet, &last_error_);

    setStringParam(device_selector_, device_id ? device_id : "");
    setStringParam(NDDriverVersion, driver_version);
    setIntegerParam(ADStatus, ADStatusInitializing);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADImageMode, ADImageContinuous);
    setIntegerParam(ADNumImages, 1);
    setIntegerParam(ADNumImagesCounter, 0);
    setIntegerParam(NDArrayCounter, 0);
    setIntegerParam(NDNDimensions, 2);
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);

    if (connect_device(device_id) == asynSuccess) {
        start_event_ = epicsEventMustCreate(epicsEventEmpty);
        stop_event_ = epicsEventMustCreate(epicsEventEmpty);
        epicsThreadOpts options = EPICS_THREAD_OPTS_INIT;
        options.priority = priority > 0 ? priority : epicsThreadPriorityMedium;
        options.stackSize = stack_size > 0 ? stack_size : epicsThreadGetStackSize(epicsThreadStackMedium);
        options.joinable = 1;
        acquisition_thread_ = epicsThreadCreateOpt("orbbecAcquire", acquisition_task_c, this, &options);
        if (acquisition_thread_ == nullptr) {
            set_error("Failed to create acquisition thread");
        }
    }
    callParamCallbacks();
}

OrbbecDriver::~OrbbecDriver() {
    lock();
    shutting_down_ = true;
    setIntegerParam(ADAcquire, 0);
    unlock();

    if (stop_event_ != nullptr) {
        epicsEventSignal(stop_event_);
    }
    if (start_event_ != nullptr) {
        epicsEventSignal(start_event_);
    }
    if (acquisition_thread_ != nullptr) {
        epicsThreadMustJoin(acquisition_thread_);
    }
    if (start_event_ != nullptr) {
        epicsEventDestroy(start_event_);
    }
    if (stop_event_ != nullptr) {
        epicsEventDestroy(stop_event_);
    }
}

asynStatus OrbbecDriver::writeInt32(asynUser* pasyn_user, epicsInt32 value) {
    const int function = pasyn_user->reason;
    if (function != ADAcquire) {
        return ADDriver::writeInt32(pasyn_user, value);
    }

    int previous = 0;
    getIntegerParam(ADAcquire, &previous);
    if ((value != 0) == (previous != 0)) {
        return asynSuccess;
    }
    if (value != 0 && (device_ == nullptr || acquisition_thread_ == nullptr)) {
        set_error("Cannot start acquisition without a connected device and acquisition thread");
        callParamCallbacks();
        return asynError;
    }
    if (value != 0 && acquisition_running_) {
        epicsSnprintf(pasyn_user->errorMessage, pasyn_user->errorMessageSize,
                      "Cannot restart acquisition until the previous pipeline has stopped");
        return asynError;
    }

    setIntegerParam(ADAcquire, value != 0);
    if (value != 0) {
        setIntegerParam(ADNumImagesCounter, 0);
        setIntegerParam(ADStatus, ADStatusWaiting);
        setStringParam(ADStatusMessage, "Waiting to start depth acquisition");
        setStringParam(last_error_, "");
        callParamCallbacks();
        epicsEventSignal(start_event_);
    } else {
        if (acquisition_running_) {
            setIntegerParam(ADStatus, ADStatusAborting);
            setStringParam(ADStatusMessage, "Stopping depth acquisition");
        }
        callParamCallbacks();
        epicsEventSignal(stop_event_);
    }
    return asynSuccess;
}

void OrbbecDriver::report(FILE* fp, int details) {
    char model[128];
    char serial_number[128];
    char uid[128];
    char ip_address[64];
    char connection[64];

    getStringParam(ADModel, sizeof(model), model);
    getStringParam(ADSerialNumber, sizeof(serial_number), serial_number);
    getStringParam(device_uid_, sizeof(uid), uid);
    getStringParam(device_ip_, sizeof(ip_address), ip_address);
    getStringParam(connection_type_, sizeof(connection), connection);
    fprintf(fp, "OrbbecDriver %s\n", this->portName);
    if (details > 0) {
        fprintf(fp,
                "  Model: %s\n  Serial number: %s\n  UID: %s\n  IP address: %s\n"
                "  Connection: %s\n",
                model, serial_number, uid, ip_address, connection);
    }
    ADDriver::report(fp, details);
}

extern "C" int OrbbecConfig(const char* port_name, const char* device_id, int max_buffers, size_t max_memory,
                            int priority, int stack_size) {
    new OrbbecDriver(port_name, device_id, max_buffers, max_memory, priority, stack_size);
    return asynSuccess;
}

static const iocshArg config_arg_0 = {"Port name", iocshArgString};
static const iocshArg config_arg_1 = {"Device ID or IPv4 address", iocshArgString};
static const iocshArg config_arg_2 = {"Max buffers", iocshArgInt};
static const iocshArg config_arg_3 = {"Max memory", iocshArgInt};
static const iocshArg config_arg_4 = {"Priority", iocshArgInt};
static const iocshArg config_arg_5 = {"Stack size", iocshArgInt};
static const iocshArg* const config_args[] = {&config_arg_0, &config_arg_1, &config_arg_2,
                                              &config_arg_3, &config_arg_4, &config_arg_5};
static const iocshFuncDef config_def = {"OrbbecConfig", 6, config_args};

static void config_call_func(const iocshArgBuf* args) {
    OrbbecConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void OrbbecDriverRegister(void) { iocshRegister(&config_def, config_call_func); }

epicsExportRegistrar(OrbbecDriverRegister);
