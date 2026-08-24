// Low level serial-port tuning helpers.
//
// The dominant cost of a Dynamixel sync-read is *not* the wire time of the
// packets, it is the USB latency timer of the FTDI bridge (U2D2 and friends).
// Its Linux default is 16 ms, which means every round trip is padded with up
// to 16 ms of pure waiting.  Both the Python and the C++ DynamixelSDK assume
// that value (`LATENCY_TIMER = 16`) but neither of them changes it, so tuning
// it is left to the application.
//
// Two independent mechanisms are used here, because which one is available
// depends on the driver and on the permissions of the caller:
//
//   1. `/sys/bus/usb-serial/devices/<tty>/latency_timer` - the ftdi_sio
//      attribute.  Writing it needs write permission on the sysfs file
//      (usually root, or a udev rule).
//   2. `TIOCSSERIAL` with `ASYNC_LOW_LATENCY` - works without extra
//      privileges and has the same practical effect for ftdi_sio.

#include "port_tuning.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace robopy_dxl {

namespace {

// "/dev/ttyUSB0" -> "ttyUSB0".  Anything without a '/' is returned unchanged.
std::string tty_basename(const std::string& port_name) {
  const std::size_t slash = port_name.find_last_of('/');
  return slash == std::string::npos ? port_name : port_name.substr(slash + 1);
}

std::string latency_sysfs_path(const std::string& port_name) {
  return "/sys/bus/usb-serial/devices/" + tty_basename(port_name) + "/latency_timer";
}

}  // namespace

int get_latency_timer(const std::string& port_name) {
#if defined(__linux__)
  std::ifstream in(latency_sysfs_path(port_name));
  if (!in.is_open()) return -1;
  int value = -1;
  in >> value;
  return in.fail() ? -1 : value;
#else
  (void)port_name;
  return -1;
#endif
}

PortTuning set_latency_timer(const std::string& port_name, int latency_ms) {
  PortTuning result;
#if defined(__linux__)
  {
    std::ofstream out(latency_sysfs_path(port_name));
    if (out.is_open()) {
      out << latency_ms;
      out.flush();
      result.sysfs_written = out.good();
      if (!result.sysfs_written) result.detail = "sysfs write failed";
    } else {
      result.detail = "no write permission on " + latency_sysfs_path(port_name);
    }
  }

  // ASYNC_LOW_LATENCY is only meaningful when we are actually asking for the
  // minimum; for a deliberately relaxed timer we leave the flag alone.
  if (latency_ms <= 1) {
    const int fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd >= 0) {
      struct serial_struct serial_info;
      std::memset(&serial_info, 0, sizeof(serial_info));
      if (::ioctl(fd, TIOCGSERIAL, &serial_info) == 0) {
        serial_info.flags |= ASYNC_LOW_LATENCY;
        result.low_latency_flag_set = ::ioctl(fd, TIOCSSERIAL, &serial_info) == 0;
        if (!result.low_latency_flag_set && result.detail.empty()) {
          result.detail = std::string("TIOCSSERIAL failed: ") + std::strerror(errno);
        }
      } else if (result.detail.empty()) {
        result.detail = std::string("TIOCGSERIAL failed: ") + std::strerror(errno);
      }
      ::close(fd);
    } else if (result.detail.empty()) {
      result.detail = std::string("open() failed: ") + std::strerror(errno);
    }
  }
#else
  (void)port_name;
  (void)latency_ms;
  result.detail = "latency timer tuning is only implemented on Linux";
#endif
  return result;
}

}  // namespace robopy_dxl
