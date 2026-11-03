#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include <epicsExport.h>
#include <epicsString.h>
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
    setStringParam(last_error_, message.c_str());
    setStringParam(ADStatusMessage, message.c_str());
    setIntegerParam(ADStatus, ADStatusError);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: %s\n", driver_name, message.c_str());
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
    : ADDriver(port_name, MAX_ADDR, NUM_PARAMS, max_buffers, max_memory, 0, 0, ASYN_CANBLOCK, 1, priority, stack_size) {

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
    connect_device(device_id);
    callParamCallbacks();
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
