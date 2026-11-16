#include <atomic>
#include <memory>
#include <string>

#include "ADDriver.h"
#include <epicsEvent.h>
#include <epicsThread.h>
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Filter.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <libobsensor/hpp/Pipeline.hpp>
#include <libobsensor/hpp/Version.hpp>

class OrbbecDriver : public ADDriver {
  public:
    OrbbecDriver(const char* port_name, const char* device_id, int max_buffers, size_t max_memory,
                 int priority, int stack_size);
    ~OrbbecDriver() override;

    void report(FILE* fp, int details) override;
    asynStatus writeInt32(asynUser* pasyn_user, epicsInt32 value) override;

  protected:
    int device_selector_;
    int device_uid_;
    int device_ip_;
    int connection_type_;
    int device_vid_;
    int device_pid_;
    int last_error_;
    int depth_align_;

  private:
    static void acquisition_task_c(void* driver);
    void acquisition_task();
    asynStatus connect_device(const char* device_id);
    asynStatus publish_frame_pair(const std::shared_ptr<ob::ColorFrame>& color_frame,
                                  const std::shared_ptr<ob::Frame>& color_rgb,
                                  const std::shared_ptr<ob::DepthFrame>& depth_frame, bool depth_aligned);
    NDArray* make_color_array(const std::shared_ptr<ob::ColorFrame>& color_frame,
                              const std::shared_ptr<ob::Frame>& color_rgb);
    NDArray* make_depth_array(const std::shared_ptr<ob::DepthFrame>& frame, bool aligned);
    void add_frame_attributes(NDArray* array, const std::shared_ptr<ob::Frame>& frame, OBFormat source_format,
                              NDColorMode_t color_mode);
    void set_array_params(NDArray* array, int addr, int unique_id, const epicsTimeStamp& time_stamp);
    void set_error(const std::string& message);

    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    epicsEventId start_event_;
    epicsEventId stop_event_;
    epicsThreadId acquisition_thread_;
    bool acquisition_running_;
    std::atomic<bool> shutting_down_;
};
