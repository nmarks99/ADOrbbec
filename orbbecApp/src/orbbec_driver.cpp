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
#include <libobsensor/hpp/TypeHelper.hpp>
#include <libobsensor/hpp/Version.hpp>

#include "orbbec_driver.hpp"

static const char* driver_name = "OrbbecDriver";
static const char* driver_version = "0.1.0";

constexpr int COLOR_ADDR = 0;
constexpr int DEPTH_ADDR = 1;

static const char* intrinsic_names[4] = {"Fx", "Fy", "Cx", "Cy"};
static const char* distortion_names[8] = {"K1", "K2", "K3", "K4", "K5", "K6", "P1", "P2"};
static const char* intrinsic_params[4] = {"ORBBEC_FX", "ORBBEC_FY", "ORBBEC_CX", "ORBBEC_CY"};
static const char* distortion_params[8] = {"ORBBEC_K1", "ORBBEC_K2", "ORBBEC_K3", "ORBBEC_K4",
                                           "ORBBEC_K5", "ORBBEC_K6", "ORBBEC_P1", "ORBBEC_P2"};

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

static bool valid_dimensions(size_t width, size_t height, size_t bytes_per_pixel) {
    const size_t int_max = static_cast<size_t>(std::numeric_limits<int>::max());
    return width != 0 && height != 0 && width <= int_max && height <= int_max &&
           width <= std::numeric_limits<size_t>::max() / height &&
           width * height <= int_max / bytes_per_pixel;
}

static bool rgb_convert_type(OBFormat format, OBConvertFormat* type) {
    switch (format) {
    case OB_FORMAT_YUYV:
        *type = FORMAT_YUYV_TO_RGB;
        return true;
    case OB_FORMAT_UYVY:
        *type = FORMAT_UYVY_TO_RGB;
        return true;
    case OB_FORMAT_I420:
        *type = FORMAT_I420_TO_RGB;
        return true;
    case OB_FORMAT_NV12:
        *type = FORMAT_NV12_TO_RGB;
        return true;
    case OB_FORMAT_NV21:
        *type = FORMAT_NV21_TO_RGB;
        return true;
    case OB_FORMAT_MJPG:
        *type = FORMAT_MJPG_TO_RGB;
        return true;
    case OB_FORMAT_BGR:
        *type = FORMAT_BGR_TO_RGB;
        return true;
    case OB_FORMAT_RGBA:
        *type = FORMAT_RGBA_TO_RGB;
        return true;
    default:
        return false;
    }
}

static std::string format_name(OBFormat format) { return ob::TypeHelper::convertOBFormatTypeToString(format); }

void OrbbecDriver::add_frame_attributes(NDArray* array, const std::shared_ptr<ob::Frame>& frame,
                                        OBFormat source_format, NDColorMode_t color_mode) {
    getAttributes(array->pAttributeList);
    epicsUInt64 frame_index = frame->getIndex();
    epicsUInt64 device_timestamp = frame->getTimeStampUs();
    epicsUInt64 system_timestamp = frame->getSystemTimeStampUs();
    epicsInt32 pixel_format = static_cast<epicsInt32>(source_format);
    epicsInt32 nd_color_mode = color_mode;
    array->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &nd_color_mode);
    array->pAttributeList->add("OrbbecFrameIndex", "Orbbec SDK frame index", NDAttrUInt64, &frame_index);
    array->pAttributeList->add("OrbbecDeviceTimestampUs", "Device timestamp in microseconds", NDAttrUInt64,
                               &device_timestamp);
    array->pAttributeList->add("OrbbecSystemTimestampUs", "Host timestamp in microseconds", NDAttrUInt64,
                               &system_timestamp);
    array->pAttributeList->add("OrbbecPixelFormat", "Orbbec SDK source pixel format", NDAttrInt32,
                               &pixel_format);
}

NDArray* OrbbecDriver::make_color_array(const std::shared_ptr<ob::ColorFrame>& color_frame,
                                        const std::shared_ptr<ob::Frame>& color_rgb) {
    if (color_rgb->getFormat() != OB_FORMAT_RGB) {
        set_error("Color frame was not converted to RGB: " + format_name(color_rgb->getFormat()));
        return nullptr;
    }

    const size_t width = color_frame->getWidth();
    const size_t height = color_frame->getHeight();
    if (!valid_dimensions(width, height, 3)) {
        set_error("Invalid color frame dimensions");
        return nullptr;
    }

    const size_t data_size = width * height * 3;
    if (color_rgb->getData() == nullptr || color_rgb->getDataSize() < data_size) {
        set_error("Color frame data is missing or shorter than its dimensions");
        return nullptr;
    }

    size_t dimensions[3] = {3, width, height};
    NDArray* array = pNDArrayPool->alloc(3, dimensions, NDUInt8, 0, nullptr);
    if (array == nullptr) {
        set_error("Failed to allocate color NDArray");
        return nullptr;
    }
    std::memcpy(array->pData, color_rgb->getData(), data_size);
    add_frame_attributes(array, color_frame, color_frame->getFormat(), NDColorModeRGB1);
    return array;
}

NDArray* OrbbecDriver::make_depth_array(const std::shared_ptr<ob::DepthFrame>& frame, bool aligned) {
    const OBFormat format = frame->getFormat();
    if (format != OB_FORMAT_Y16 && format != OB_FORMAT_Z16) {
        set_error("Unsupported depth format: " + format_name(format));
        return nullptr;
    }

    const size_t width = frame->getWidth();
    const size_t height = frame->getHeight();
    if (!valid_dimensions(width, height, sizeof(epicsUInt16))) {
        set_error("Invalid depth frame dimensions");
        return nullptr;
    }

    const size_t data_size = width * height * sizeof(epicsUInt16);
    if (frame->getData() == nullptr || frame->getDataSize() < data_size) {
        set_error("Depth frame data is missing or shorter than its dimensions");
        return nullptr;
    }

    size_t dimensions[2] = {width, height};
    NDArray* array = pNDArrayPool->alloc(2, dimensions, NDUInt16, 0, nullptr);
    if (array == nullptr) {
        set_error("Failed to allocate depth NDArray");
        return nullptr;
    }
    std::memcpy(array->pData, frame->getData(), data_size);
    add_frame_attributes(array, frame, format, NDColorModeMono);

    epicsFloat32 depth_scale = frame->getValueScale();
    char depth_units[] = "mm per raw unit";
    array->pAttributeList->add("OrbbecDepthScale", "Millimeters per raw depth unit", NDAttrFloat32,
                               &depth_scale);
    array->pAttributeList->add("OrbbecDepthUnits", "Depth scale units", NDAttrString, depth_units);
    epicsInt32 depth_aligned = aligned ? 1 : 0;
    array->pAttributeList->add("OrbbecDepthAligned", "Depth aligned to color camera", NDAttrInt32,
                               &depth_aligned);
    return array;
}

void OrbbecDriver::set_array_params(NDArray* array, int addr, int unique_id, const epicsTimeStamp& time_stamp) {
    array->uniqueId = unique_id;
    array->epicsTS = time_stamp;
    array->timeStamp = time_stamp.secPastEpoch + time_stamp.nsec / 1.e9;

    NDArrayInfo_t info;
    array->getInfo(&info);
    setIntegerParam(addr, NDArraySizeX, static_cast<int>(info.xSize));
    setIntegerParam(addr, NDArraySizeY, static_cast<int>(info.ySize));
    setIntegerParam(addr, NDArraySizeZ, static_cast<int>(info.colorSize));
    setIntegerParam(addr, NDArraySize, static_cast<int>(info.totalBytes));
    setIntegerParam(addr, NDNDimensions, array->ndims);
    setIntegerParam(addr, NDDataType, array->dataType);
    setIntegerParam(addr, NDColorMode, info.colorMode);
    setIntegerParam(addr, NDUniqueId, unique_id);
    setDoubleParam(addr, NDTimeStamp, array->timeStamp);
    setIntegerParam(addr, NDArrayCounter, unique_id);
}

void OrbbecDriver::set_calibration(NDArray* array, int addr, const std::shared_ptr<ob::Frame>& frame) {
    auto profile = frame->getStreamProfile()->as<ob::VideoStreamProfile>();
    const OBCameraIntrinsic intrinsic = profile->getIntrinsic();
    const OBCameraDistortion distortion = profile->getDistortion();
    const double intrinsic_values[4] = {intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy};
    const double distortion_values[8] = {distortion.k1, distortion.k2, distortion.k3, distortion.k4,
                                         distortion.k5, distortion.k6, distortion.p1, distortion.p2};

    for (int i = 0; i < 4; ++i) {
        epicsFloat64 value = intrinsic_values[i];
        setDoubleParam(addr, intrinsic_[i], value);
        array->pAttributeList->add((std::string("Orbbec") + intrinsic_names[i]).c_str(),
                                   "Camera intrinsic (pixels)", NDAttrFloat64, &value);
    }
    for (int i = 0; i < 8; ++i) {
        epicsFloat64 value = distortion_values[i];
        setDoubleParam(addr, distortion_[i], value);
        array->pAttributeList->add((std::string("Orbbec") + distortion_names[i]).c_str(),
                                   "Camera distortion coefficient", NDAttrFloat64, &value);
    }
    epicsInt32 model = distortion.model;
    setIntegerParam(addr, distortion_model_, model);
    array->pAttributeList->add("OrbbecDistortionModel", "OBCameraDistortionModel", NDAttrInt32, &model);
}

asynStatus OrbbecDriver::publish_frame_pair(const std::shared_ptr<ob::ColorFrame>& color_frame,
                                            const std::shared_ptr<ob::Frame>& color_rgb,
                                            const std::shared_ptr<ob::DepthFrame>& depth_frame,
                                            bool depth_aligned) {
    using ArrayGuard = std::unique_ptr<NDArray, void (*)(NDArray*)>;
    auto release = [](NDArray* value) { value->release(); };

    NDArray* color_array = make_color_array(color_frame, color_rgb);
    if (color_array == nullptr) {
        return asynError;
    }
    ArrayGuard color_guard(color_array, release);

    NDArray* depth_array = make_depth_array(depth_frame, depth_aligned);
    if (depth_array == nullptr) {
        return asynError;
    }
    ArrayGuard depth_guard(depth_array, release);

    int array_counter = 0;
    int image_counter = 0;
    getIntegerParam(NDArrayCounter, &array_counter);
    getIntegerParam(ADNumImagesCounter, &image_counter);
    ++array_counter;
    ++image_counter;

    epicsTimeStamp time_stamp;
    updateTimeStamp(&time_stamp);
    set_array_params(color_array, COLOR_ADDR, array_counter, time_stamp);
    set_array_params(depth_array, DEPTH_ADDR, array_counter, time_stamp);
    set_calibration(color_array, COLOR_ADDR, color_frame);
    set_calibration(depth_array, DEPTH_ADDR, depth_frame);

    setIntegerParam(ADSizeX, static_cast<int>(color_frame->getWidth()));
    setIntegerParam(ADSizeY, static_cast<int>(color_frame->getHeight()));
    setIntegerParam(ADMaxSizeX, static_cast<int>(color_frame->getWidth()));
    setIntegerParam(ADMaxSizeY, static_cast<int>(color_frame->getHeight()));
    setIntegerParam(ADNumImagesCounter, image_counter);
    callParamCallbacks(COLOR_ADDR);
    callParamCallbacks(DEPTH_ADDR);

    int array_callbacks = 0;
    getIntegerParam(NDArrayCallbacks, &array_callbacks);
    if (array_callbacks) {
        unlock();
        doCallbacksGenericPointer(color_array, NDArrayData, COLOR_ADDR);
        doCallbacksGenericPointer(depth_array, NDArrayData, DEPTH_ADDR);
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
        std::shared_ptr<ob::FormatConvertFilter> format_converter;
        std::shared_ptr<ob::Align> align_filter;
        std::string error_message;

        try {
            if (!device_) {
                throw std::runtime_error("No Orbbec device is connected");
            }
            pipeline = std::make_shared<ob::Pipeline>(device_);
            auto config = std::make_shared<ob::Config>();
            config->enableVideoStream(OB_STREAM_COLOR);
            config->enableVideoStream(OB_STREAM_DEPTH);
            config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            pipeline->enableFrameSync();
            pipeline->start(config);
            pipeline_started = true;

            lock();
            getIntegerParam(ADAcquire, &acquire);
            if (acquire && !shutting_down_) {
                setIntegerParam(ADStatus, ADStatusAcquire);
                setStringParam(ADStatusMessage, "Acquiring color+depth frames");
                callParamCallbacks();
            } else {
                operator_stopped = !shutting_down_;
            }
            unlock();

            while (acquire && !shutting_down_) {
                std::shared_ptr<ob::FrameSet> frame_set = pipeline->waitForFrameset(100);

                const bool stop_requested = epicsEventTryWait(stop_event_) == epicsEventWaitOK;
                int depth_align = 0;
                lock();
                getIntegerParam(ADAcquire, &acquire);
                getIntegerParam(depth_align_, &depth_align);
                if (stop_requested || !acquire || shutting_down_) {
                    operator_stopped = !shutting_down_;
                    unlock();
                    break;
                }
                unlock();

                if (!frame_set) {
                    continue;
                }
                if (depth_align) {
                    if (!align_filter) {
                        align_filter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
                    }
                    std::shared_ptr<ob::Frame> aligned = align_filter->process(frame_set);
                    if (!aligned || !aligned->is<ob::FrameSet>()) {
                        error_message = "Depth alignment failed";
                        break;
                    }
                    frame_set = aligned->as<ob::FrameSet>();
                }
                std::shared_ptr<ob::Frame> color_raw = frame_set->getFrame(OB_FRAME_COLOR);
                std::shared_ptr<ob::Frame> depth_raw = frame_set->getFrame(OB_FRAME_DEPTH);
                if (!color_raw || !depth_raw) {
                    continue;
                }
                std::shared_ptr<ob::ColorFrame> color_frame = color_raw->as<ob::ColorFrame>();
                std::shared_ptr<ob::DepthFrame> depth_frame = depth_raw->as<ob::DepthFrame>();

                const OBFormat color_format = color_frame->getFormat();
                std::shared_ptr<ob::Frame> color_rgb = color_frame;
                if (color_format != OB_FORMAT_RGB) {
                    OBConvertFormat convert_type;
                    if (!rgb_convert_type(color_format, &convert_type)) {
                        error_message = "Unsupported color format: " + format_name(color_format);
                        break;
                    }
                    if (!format_converter) {
                        format_converter = std::make_shared<ob::FormatConvertFilter>();
                    }
                    format_converter->setFormatConvertType(convert_type);
                    color_rgb = format_converter->process(color_frame);
                    if (!color_rgb) {
                        error_message = "Color format conversion failed for " + format_name(color_format);
                        break;
                    }
                }

                lock();
                asynStatus status;
                try {
                    status = publish_frame_pair(color_frame, color_rgb, depth_frame, depth_align != 0);
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
                    error_message = message[0] != '\0' ? message : "Failed to publish color+depth frames";
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
            error_message = std::string("Acquisition error: ") + error.what();
        }

        if (pipeline_started) {
            try {
                pipeline->stop();
            } catch (const std::exception& error) {
                if (error_message.empty()) {
                    error_message = std::string("Failed to stop pipeline: ") + error.what();
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

constexpr unsigned int MAX_ADDR = 2;
constexpr unsigned int NUM_PARAMS = 21;

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
    createParam("ORBBEC_DEPTH_ALIGN", asynParamInt32, &depth_align_);
    for (int i = 0; i < 4; ++i) {
        createParam(intrinsic_params[i], asynParamFloat64, &intrinsic_[i]);
    }
    for (int i = 0; i < 8; ++i) {
        createParam(distortion_params[i], asynParamFloat64, &distortion_[i]);
    }
    createParam("ORBBEC_DISTORTION_MODEL", asynParamInt32, &distortion_model_);

    setStringParam(device_selector_, device_id ? device_id : "");
    setStringParam(NDDriverVersion, driver_version);
    setIntegerParam(ADStatus, ADStatusInitializing);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADImageMode, ADImageContinuous);
    setIntegerParam(ADNumImages, 1);
    setIntegerParam(ADNumImagesCounter, 0);
    setIntegerParam(depth_align_, 1);
    setIntegerParam(COLOR_ADDR, NDArrayCounter, 0);
    setIntegerParam(COLOR_ADDR, NDNDimensions, 3);
    setIntegerParam(COLOR_ADDR, NDDataType, NDUInt8);
    setIntegerParam(COLOR_ADDR, NDColorMode, NDColorModeRGB1);
    setIntegerParam(DEPTH_ADDR, NDArrayCounter, 0);
    setIntegerParam(DEPTH_ADDR, NDNDimensions, 2);
    setIntegerParam(DEPTH_ADDR, NDDataType, NDUInt16);
    setIntegerParam(DEPTH_ADDR, NDColorMode, NDColorModeMono);

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
    callParamCallbacks(COLOR_ADDR);
    callParamCallbacks(DEPTH_ADDR);
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
        setStringParam(ADStatusMessage, "Waiting to start acquisition");
        setStringParam(last_error_, "");
        callParamCallbacks();
        epicsEventSignal(start_event_);
    } else {
        if (acquisition_running_) {
            setIntegerParam(ADStatus, ADStatusAborting);
            setStringParam(ADStatusMessage, "Stopping acquisition");
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
