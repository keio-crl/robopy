#include "bus.hpp"

#include <algorithm>
#include <exception>
#include <sstream>
#include <thread>
#include <tuple>
#include <utility>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace robopy_dxl {

namespace {

std::string describe(int comm_result) {
  dynamixel::PacketHandler* ph = dynamixel::PacketHandler::getPacketHandler(2.0);
  return ph->getTxRxResult(comm_result);
}

int32_t sign_extend(uint32_t raw, uint16_t length) {
  switch (length) {
    case 1:
      return static_cast<int32_t>(static_cast<int8_t>(raw & 0xFFu));
    case 2:
      return static_cast<int32_t>(static_cast<int16_t>(raw & 0xFFFFu));
    default:
      return static_cast<int32_t>(raw);
  }
}

void to_le_bytes(int32_t value, uint16_t length, uint8_t* out) {
  const uint32_t raw = static_cast<uint32_t>(value);
  for (uint16_t i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFFu);
  }
}

// Offsets inside a protocol 2.0 packet (the SDK keeps these private).
constexpr std::size_t PKT_ID = 4;
constexpr std::size_t PKT_LENGTH_L = 5;
constexpr std::size_t PKT_LENGTH_H = 6;
constexpr std::size_t PKT_INSTRUCTION = 7;
constexpr std::size_t PKT_PARAMETER0 = 8;
constexpr uint8_t STATUS_INSTRUCTION = 0x55;
constexpr uint8_t BROADCAST = 0xFE;
// Protocol2PacketHandler::rxPacket refuses anything longer than this.
constexpr std::size_t RX_PACKET_MAX_LEN = 1024;

std::string join_ids(const std::vector<uint8_t>& ids) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i != 0) oss << ", ";
    oss << static_cast<int>(ids[i]);
  }
  return oss.str();
}

}  // namespace

CommError::CommError(const std::string& message, int comm_result)
    : std::runtime_error(message + " [" + describe(comm_result) + "]"),
      comm_result_(comm_result) {}

Bus::Bus(std::string port_name, int baudrate, float protocol_version)
    : port_name_(std::move(port_name)), baudrate_(baudrate), protocol_version_(protocol_version) {
  if (protocol_version_ != 2.0f) {
    throw std::invalid_argument("robopy_dxl only supports Dynamixel protocol 2.0");
  }
  port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
  packet_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
}

Bus::~Bus() {
  // The groups hold raw pointers into `port_`, so they have to go first.
  read_groups_.clear();
  write_groups_.clear();
  if (port_ != nullptr) {
    if (is_open_) port_->closePort();
    delete port_;
    port_ = nullptr;
  }
  // `packet_` is a process-wide singleton owned by the SDK; do not delete it.
}

void Bus::open() {
  if (is_open_) return;
  if (!port_->openPort()) {
    throw std::runtime_error("Failed to open port " + port_name_);
  }
  if (!port_->setBaudRate(baudrate_)) {
    port_->closePort();
    throw std::runtime_error("Failed to set baudrate " + std::to_string(baudrate_) + " on " +
                             port_name_);
  }
  is_open_ = true;
}

void Bus::set_baudrate(int baudrate) {
  if (baudrate == baudrate_) return;
  baudrate_ = baudrate;
  if (is_open_ && !port_->setBaudRate(baudrate_)) {
    throw std::runtime_error("Failed to set baudrate " + std::to_string(baudrate_) + " on " +
                             port_name_);
  }
}

void Bus::close() {
  if (!is_open_) return;
  port_->closePort();
  is_open_ = false;
}

void Bus::require_open() const {
  if (!is_open_) throw std::runtime_error("Port " + port_name_ + " is not open");
}

int Bus::make_read_group(uint16_t address, uint16_t length, const std::vector<uint8_t>& ids,
                         bool is_signed, bool prefer_fast) {
  if (ids.empty()) throw std::invalid_argument("read group needs at least one motor id");

  auto group = std::make_unique<ReadGroup>();
  group->address = address;
  group->length = length;
  group->is_signed = is_signed;
  group->ids = ids;
  group->use_fast = prefer_fast;

  group->plain = std::make_unique<dynamixel::GroupSyncRead>(port_, packet_, address, length);
  for (uint8_t id : ids) {
    if (!group->plain->addParam(id)) {
      throw std::runtime_error("GroupSyncRead.addParam failed for id " + std::to_string(id));
    }
  }

  // A Fast Sync Read answer is one broadcast packet holding
  // <error, id, data, crc> per motor; refuse the fast path when that would
  // exceed what the SDK's receiver accepts.
  const std::size_t fast_packet_len = 7 + 1 + ids.size() * (length + 4);
  if (prefer_fast && fast_packet_len <= RX_PACKET_MAX_LEN) {
    group->rx_buffer.assign(RX_PACKET_MAX_LEN + 16, 0);
  } else {
    group->use_fast = false;
  }

  read_groups_.push_back(std::move(group));
  return static_cast<int>(read_groups_.size()) - 1;
}

int Bus::make_write_group(uint16_t address, uint16_t length, const std::vector<uint8_t>& ids) {
  if (ids.empty()) throw std::invalid_argument("write group needs at least one motor id");

  auto group = std::make_unique<WriteGroup>();
  group->address = address;
  group->length = length;
  group->ids = ids;
  group->group = std::make_unique<dynamixel::GroupSyncWrite>(port_, packet_, address, length);

  // Seed every id with zeroes so that the hot path only ever calls
  // changeParam(), which rewrites bytes in place instead of reallocating.
  std::vector<uint8_t> zeros(length, 0);
  for (uint8_t id : ids) {
    if (!group->group->addParam(id, zeros.data())) {
      throw std::runtime_error("GroupSyncWrite.addParam failed for id " + std::to_string(id));
    }
  }

  write_groups_.push_back(std::move(group));
  return static_cast<int>(write_groups_.size()) - 1;
}

Bus::ReadGroup& Bus::read_group(int handle) {
  if (handle < 0 || static_cast<std::size_t>(handle) >= read_groups_.size()) {
    throw std::out_of_range("unknown read group handle " + std::to_string(handle));
  }
  return *read_groups_[static_cast<std::size_t>(handle)];
}

const Bus::ReadGroup& Bus::read_group(int handle) const {
  if (handle < 0 || static_cast<std::size_t>(handle) >= read_groups_.size()) {
    throw std::out_of_range("unknown read group handle " + std::to_string(handle));
  }
  return *read_groups_[static_cast<std::size_t>(handle)];
}

Bus::WriteGroup& Bus::write_group(int handle) {
  if (handle < 0 || static_cast<std::size_t>(handle) >= write_groups_.size()) {
    throw std::out_of_range("unknown write group handle " + std::to_string(handle));
  }
  return *write_groups_[static_cast<std::size_t>(handle)];
}

bool Bus::group_uses_fast(int handle) const { return read_group(handle).use_fast; }

bool Bus::attempt_plain_read(ReadGroup& group, std::vector<int32_t>& out) {
  if (group.plain->txRxPacket() != COMM_SUCCESS) return false;

  out.resize(group.ids.size());
  for (std::size_t i = 0; i < group.ids.size(); ++i) {
    const uint8_t id = group.ids[i];
    if (!group.plain->isAvailable(id, group.address, group.length)) return false;
    const uint32_t raw = group.plain->getData(id, group.address, group.length);
    out[i] = group.is_signed ? sign_extend(raw, group.length) : static_cast<int32_t>(raw);
  }
  return true;
}

bool Bus::attempt_fast_read(ReadGroup& group, std::vector<int32_t>& out) {
  const std::size_t count = group.ids.size();
  const std::size_t block = static_cast<std::size_t>(group.length) + 4;  // err, id, data, crc

  if (packet_->fastSyncReadTx(port_, group.address, group.length, group.ids.data(),
                              static_cast<uint16_t>(count)) != COMM_SUCCESS) {
    return false;
  }

  uint8_t* rxpacket = group.rx_buffer.data();
  int comm_result = COMM_RX_FAIL;
  do {
    // `true` skips de-stuffing: devices do not byte-stuff a fast status packet.
    comm_result = packet_->rxPacket(port_, rxpacket, true);
  } while (comm_result == COMM_SUCCESS && rxpacket[PKT_ID] != BROADCAST);

  if (comm_result != COMM_SUCCESS) return false;
  if (rxpacket[PKT_INSTRUCTION] != STATUS_INSTRUCTION) return false;

  // GroupFastSyncRead trusts the packet layout blindly; verify it instead, so
  // that a motor which stayed silent fails the transfer rather than shifting
  // every later motor's value by one slot.
  const std::size_t length =
      static_cast<std::size_t>(rxpacket[PKT_LENGTH_L]) | (static_cast<std::size_t>(rxpacket[PKT_LENGTH_H]) << 8);
  if (length != 1 + count * block) return false;

  out.resize(count);
  std::size_t index = PKT_PARAMETER0;
  for (std::size_t i = 0; i < count; ++i) {
    if (rxpacket[index + 1] != group.ids[i]) return false;
    uint32_t raw = 0;
    for (uint16_t byte = 0; byte < group.length; ++byte) {
      raw |= static_cast<uint32_t>(rxpacket[index + 2 + byte]) << (8 * byte);
    }
    out[i] = group.is_signed ? sign_extend(raw, group.length) : static_cast<int32_t>(raw);
    index += block;
  }
  return true;
}

std::vector<int32_t> Bus::sync_read(int handle, int retries) {
  require_open();
  ReadGroup& group = read_group(handle);
  std::vector<int32_t> values;
  const int attempts = std::max(1, retries);

  if (group.use_fast) {
    for (int i = 0; i < attempts; ++i) {
      if (attempt_fast_read(group, values)) {
        group.fast_verified = true;
        return values;
      }
    }
    if (!group.fast_verified) {
      // The motors never answered a Fast Sync Read: most likely firmware
      // without 0x8A support.  Stop trying and stay on the plain path.
      group.use_fast = false;
      group.rx_buffer.clear();
      group.rx_buffer.shrink_to_fit();
    }
  }

  for (int i = 0; i < attempts; ++i) {
    if (attempt_plain_read(group, values)) return values;
  }

  throw CommError("Failed to sync read address " + std::to_string(group.address) + " on " +
                      port_name_ + " (ids: " + join_ids(group.ids) + ")",
                  COMM_RX_FAIL);
}

void Bus::sync_write(int handle, const std::vector<int32_t>& values, int retries) {
  require_open();
  WriteGroup& group = write_group(handle);
  if (values.size() != group.ids.size()) {
    throw std::invalid_argument("expected " + std::to_string(group.ids.size()) + " values, got " +
                                std::to_string(values.size()));
  }

  uint8_t bytes[4];
  for (std::size_t i = 0; i < group.ids.size(); ++i) {
    to_le_bytes(values[i], group.length, bytes);
    if (!group.group->changeParam(group.ids[i], bytes)) {
      throw std::runtime_error("GroupSyncWrite.changeParam failed for id " +
                               std::to_string(group.ids[i]));
    }
  }

  int comm_result = COMM_TX_FAIL;
  for (int i = 0, attempts = std::max(1, retries); i < attempts; ++i) {
    comm_result = group.group->txPacket();
    if (comm_result == COMM_SUCCESS) return;
  }

  throw CommError("Failed to sync write address " + std::to_string(group.address) + " on " +
                      port_name_ + " (ids: " + join_ids(group.ids) + ")",
                  comm_result);
}

int32_t Bus::read(uint8_t id, uint16_t address, uint16_t length, bool is_signed, int retries) {
  require_open();
  if (length != 1 && length != 2 && length != 4) {
    throw std::invalid_argument("length must be 1, 2 or 4");
  }

  uint8_t data[4] = {0, 0, 0, 0};
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;
  for (int i = 0, attempts = std::max(1, retries); i < attempts; ++i) {
    comm_result = packet_->readTxRx(port_, id, address, length, data, &error);
    if (comm_result == COMM_SUCCESS) {
      uint32_t raw = 0;
      for (uint16_t b = 0; b < length; ++b) raw |= static_cast<uint32_t>(data[b]) << (8 * b);
      return is_signed ? sign_extend(raw, length) : static_cast<int32_t>(raw);
    }
  }
  throw CommError("Failed to read address " + std::to_string(address) + " from id " +
                      std::to_string(id) + " on " + port_name_,
                  comm_result);
}

void Bus::write(uint8_t id, uint16_t address, uint16_t length, int32_t value, int retries) {
  require_open();
  if (length != 1 && length != 2 && length != 4) {
    throw std::invalid_argument("length must be 1, 2 or 4");
  }

  uint8_t data[4] = {0, 0, 0, 0};
  to_le_bytes(value, length, data);
  uint8_t error = 0;
  int comm_result = COMM_TX_FAIL;
  for (int i = 0, attempts = std::max(1, retries); i < attempts; ++i) {
    comm_result = packet_->writeTxRx(port_, id, address, length, data, &error);
    if (comm_result == COMM_SUCCESS) return;
  }
  throw CommError("Failed to write address " + std::to_string(address) + " to id " +
                      std::to_string(id) + " on " + port_name_,
                  comm_result);
}

uint16_t Bus::ping(uint8_t id, int retries) {
  require_open();
  uint16_t model_number = 0;
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;
  for (int i = 0, attempts = std::max(1, retries); i < attempts; ++i) {
    comm_result = packet_->ping(port_, id, &model_number, &error);
    if (comm_result == COMM_SUCCESS) return model_number;
  }
  throw CommError("No response from id " + std::to_string(id) + " on " + port_name_, comm_result);
}

std::vector<uint8_t> Bus::broadcast_ping() {
  require_open();
  std::vector<uint8_t> ids;
  const int comm_result = packet_->broadcastPing(port_, ids);
  if (comm_result != COMM_SUCCESS && ids.empty()) {
    throw CommError("Broadcast ping failed on " + port_name_, comm_result);
  }
  return ids;
}

namespace {

/// Runs `body` for every target on its own thread and rethrows the first
/// failure once all threads have been joined.
template <typename Body>
void run_on_each(std::size_t count, Body&& body) {
  if (count == 0) return;
  if (count == 1) {
    body(0);
    return;
  }

  std::vector<std::exception_ptr> errors(count);
  std::vector<std::thread> workers;
  workers.reserve(count - 1);
  for (std::size_t i = 1; i < count; ++i) {
    workers.emplace_back([&, i]() {
      try {
        body(i);
      } catch (...) {
        errors[i] = std::current_exception();
      }
    });
  }
  try {
    body(0);
  } catch (...) {
    errors[0] = std::current_exception();
  }
  for (std::thread& worker : workers) worker.join();

  for (const std::exception_ptr& error : errors) {
    if (error) std::rethrow_exception(error);
  }
}

}  // namespace

std::vector<std::vector<int32_t>> sync_read_parallel(
    const std::vector<std::pair<Bus*, int>>& targets, int retries) {
  std::vector<std::vector<int32_t>> results(targets.size());
  run_on_each(targets.size(), [&](std::size_t i) {
    Bus* bus = targets[i].first;
    std::lock_guard<std::mutex> guard(bus->mutex());
    results[i] = bus->sync_read(targets[i].second, retries);
  });
  return results;
}

void sync_write_parallel(
    const std::vector<std::tuple<Bus*, int, std::vector<int32_t>>>& targets, int retries) {
  run_on_each(targets.size(), [&](std::size_t i) {
    Bus* bus = std::get<0>(targets[i]);
    std::lock_guard<std::mutex> guard(bus->mutex());
    bus->sync_write(std::get<1>(targets[i]), std::get<2>(targets[i]), retries);
  });
}

}  // namespace robopy_dxl
