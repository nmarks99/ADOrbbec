#include "ADDriver.h"
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Version.hpp>

// static const char* driver_name = "OrbbecDriver";
// static const char* driver_version = "0.1.0";

class OrbbecDriver : public ADDriver {
  public:
    OrbbecDriver(const char* port_name, const char* device_id, int max_buffers, size_t max_memory,
                 int priority, int stack_size);

    void report(FILE* fp, int details) override;

  protected:
    int device_selector_;
    int device_uid_;
    int device_ip_;
    int connection_type_;
    int device_vid_;
    int device_pid_;
    int last_error_;

  private:
    asynStatus connect_device(const char* device_id);
    void set_error(const std::string& message);

    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
};
