#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "bus.hpp"
#include "port_tuning.hpp"

namespace py = pybind11;
using robopy_dxl::Bus;
using robopy_dxl::CommError;

namespace {

py::array_t<int32_t> to_array(const std::vector<int32_t>& values) {
  py::array_t<int32_t> array(static_cast<py::ssize_t>(values.size()));
  std::copy(values.begin(), values.end(), array.mutable_data());
  return array;
}

std::vector<int32_t> to_values(const py::object& obj) {
  py::array_t<int32_t, py::array::c_style | py::array::forcecast> array(obj);
  if (array.ndim() != 1) throw std::invalid_argument("values must be one dimensional");
  const int32_t* data = array.data();
  return std::vector<int32_t>(data, data + array.size());
}

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "C++ Dynamixel transport for robopy (DynamixelSDK C++ + pybind11).";

  py::register_exception<CommError>(m, "DxlCommError", PyExc_ConnectionError);

  py::class_<robopy_dxl::PortTuning>(m, "PortTuning")
      .def_readonly("sysfs_written", &robopy_dxl::PortTuning::sysfs_written)
      .def_readonly("low_latency_flag_set", &robopy_dxl::PortTuning::low_latency_flag_set)
      .def_readonly("detail", &robopy_dxl::PortTuning::detail)
      .def_property_readonly("ok", &robopy_dxl::PortTuning::ok)
      .def("__repr__", [](const robopy_dxl::PortTuning& self) {
        return "<PortTuning ok=" + std::string(self.ok() ? "True" : "False") +
               " sysfs=" + (self.sysfs_written ? "True" : "False") +
               " low_latency=" + (self.low_latency_flag_set ? "True" : "False") +
               (self.detail.empty() ? "" : " detail='" + self.detail + "'") + ">";
      });

  m.def("set_latency_timer", &robopy_dxl::set_latency_timer, py::arg("port_name"),
        py::arg("latency_ms") = 1,
        "Best-effort lowering of the USB latency timer of a serial port.");
  m.def("get_latency_timer", &robopy_dxl::get_latency_timer, py::arg("port_name"),
        "Current ftdi_sio latency timer in ms, or -1 when it cannot be read.");

  py::class_<Bus, std::shared_ptr<Bus>>(m, "Bus")
      .def(py::init<std::string, int, float>(), py::arg("port_name"),
           py::arg("baudrate") = 1000000, py::arg("protocol_version") = 2.0f)
      .def("open", &Bus::open, py::call_guard<py::gil_scoped_release>())
      .def("close", &Bus::close, py::call_guard<py::gil_scoped_release>())
      .def("set_baudrate", &Bus::set_baudrate, py::arg("baudrate"),
           py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("is_open", &Bus::is_open)
      .def_property_readonly("port_name", &Bus::port_name)
      .def_property_readonly("baudrate", &Bus::baudrate)
      .def("make_read_group", &Bus::make_read_group, py::arg("address"), py::arg("length"),
           py::arg("ids"), py::arg("is_signed") = false, py::arg("prefer_fast") = true)
      .def("make_write_group", &Bus::make_write_group, py::arg("address"), py::arg("length"),
           py::arg("ids"))
      .def("group_uses_fast", &Bus::group_uses_fast, py::arg("handle"))
      .def(
          "sync_read",
          [](Bus& self, int handle, int retries) {
            std::vector<int32_t> values;
            {
              py::gil_scoped_release release;
              std::lock_guard<std::mutex> guard(self.mutex());
              values = self.sync_read(handle, retries);
            }
            return to_array(values);
          },
          py::arg("handle"), py::arg("retries") = 3)
      .def(
          "sync_write",
          [](Bus& self, int handle, const py::object& values, int retries) {
            std::vector<int32_t> data = to_values(values);
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(self.mutex());
            self.sync_write(handle, data, retries);
          },
          py::arg("handle"), py::arg("values"), py::arg("retries") = 2)
      .def(
          "read",
          [](Bus& self, uint8_t id, uint16_t address, uint16_t length, bool is_signed,
             int retries) {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(self.mutex());
            return self.read(id, address, length, is_signed, retries);
          },
          py::arg("id"), py::arg("address"), py::arg("length"), py::arg("is_signed") = false,
          py::arg("retries") = 3)
      .def(
          "write",
          [](Bus& self, uint8_t id, uint16_t address, uint16_t length, int32_t value, int retries) {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(self.mutex());
            self.write(id, address, length, value, retries);
          },
          py::arg("id"), py::arg("address"), py::arg("length"), py::arg("value"),
          py::arg("retries") = 2)
      .def(
          "ping",
          [](Bus& self, uint8_t id, int retries) {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(self.mutex());
            return self.ping(id, retries);
          },
          py::arg("id"), py::arg("retries") = 3)
      .def(
          "broadcast_ping",
          [](Bus& self) {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> guard(self.mutex());
            return self.broadcast_ping();
          })
      .def("__repr__", [](const Bus& self) {
        return "<robopy_dxl.Bus port='" + self.port_name() +
               "' baudrate=" + std::to_string(self.baudrate()) +
               " open=" + (self.is_open() ? "True" : "False") + ">";
      });

  m.def(
      "sync_read_parallel",
      [](const std::vector<std::pair<std::shared_ptr<Bus>, int>>& targets, int retries) {
        std::vector<std::pair<Bus*, int>> raw;
        raw.reserve(targets.size());
        for (const auto& target : targets) raw.emplace_back(target.first.get(), target.second);

        std::vector<std::vector<int32_t>> results;
        {
          py::gil_scoped_release release;
          results = robopy_dxl::sync_read_parallel(raw, retries);
        }

        py::list out;
        for (const std::vector<int32_t>& values : results) out.append(to_array(values));
        return out;
      },
      py::arg("targets"), py::arg("retries") = 3,
      "Read one group per bus concurrently; returns one int32 array per target.");

  m.def(
      "sync_write_parallel",
      [](const std::vector<std::tuple<std::shared_ptr<Bus>, int, py::object>>& targets,
         int retries) {
        std::vector<std::tuple<Bus*, int, std::vector<int32_t>>> raw;
        raw.reserve(targets.size());
        for (const auto& target : targets) {
          raw.emplace_back(std::get<0>(target).get(), std::get<1>(target),
                           to_values(std::get<2>(target)));
        }
        py::gil_scoped_release release;
        robopy_dxl::sync_write_parallel(raw, retries);
      },
      py::arg("targets"), py::arg("retries") = 2,
      "Write one group per bus concurrently.");

#ifdef ROBOPY_DXL_VERSION
  m.attr("__version__") = ROBOPY_DXL_VERSION;
#else
  m.attr("__version__") = "0.0.0";
#endif
}
