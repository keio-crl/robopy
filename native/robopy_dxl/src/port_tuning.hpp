#pragma once

#include <string>

namespace robopy_dxl {

/// Outcome of an attempt to lower the USB latency timer of a serial port.
struct PortTuning {
  bool sysfs_written = false;
  bool low_latency_flag_set = false;
  std::string detail;

  bool ok() const { return sysfs_written || low_latency_flag_set; }
};

/// Current value of the ftdi_sio latency timer in ms, or -1 when unknown.
int get_latency_timer(const std::string& port_name);

/// Best-effort lowering of the USB latency timer.  Never throws.
PortTuning set_latency_timer(const std::string& port_name, int latency_ms);

}  // namespace robopy_dxl
