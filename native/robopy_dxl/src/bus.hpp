#pragma once

// A thin, allocation-free-in-the-hot-path wrapper around the C++ DynamixelSDK.
//
// Compared to driving `dynamixel_sdk` from Python this buys three things:
//
//   * Fast Sync Read (instruction 0x8A).  17 individual status packets, each
//     preceded by the motor's return delay, collapse into a single broadcast
//     status packet.  The pinned Python SDK (3.7.x) does not implement it at
//     all; only the C++ SDK does.
//   * Group objects are created once and reused.  The Python bus rebuilds a
//     GroupSyncRead/GroupSyncWrite, re-adds every id and re-serialises every
//     parameter on every single call.
//   * The GIL is dropped for the duration of the transfer, so two buses (a
//     leader and a follower arm are two independent USB devices) can be
//     serviced concurrently instead of one after the other.

#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace dynamixel {
class PortHandler;
class PacketHandler;
class GroupSyncRead;
class GroupSyncWrite;
}  // namespace dynamixel

namespace robopy_dxl {

/// Raised for any protocol level failure; maps to `robopy_dxl.DxlCommError`.
class CommError : public std::runtime_error {
 public:
  CommError(const std::string& message, int comm_result);

  int comm_result() const { return comm_result_; }

 private:
  int comm_result_;
};

class Bus {
 public:
  Bus(std::string port_name, int baudrate, float protocol_version);
  ~Bus();

  Bus(const Bus&) = delete;
  Bus& operator=(const Bus&) = delete;

  void open();
  void close();
  bool is_open() const { return is_open_; }

  /// Changes the baudrate, applying it immediately when the port is open.
  /// Registered groups stay valid, since they hold the port, not the speed.
  void set_baudrate(int baudrate);

  const std::string& port_name() const { return port_name_; }
  int baudrate() const { return baudrate_; }

  /// Registers a reusable sync-read group and returns its handle.
  ///
  /// `is_signed` selects whether raw values are sign extended from
  /// `length` bytes.  `prefer_fast` requests Fast Sync Read; it is silently
  /// downgraded to a plain Sync Read if the motors do not answer it.
  int make_read_group(uint16_t address, uint16_t length, const std::vector<uint8_t>& ids,
                      bool is_signed, bool prefer_fast);

  /// Registers a reusable sync-write group and returns its handle.
  int make_write_group(uint16_t address, uint16_t length, const std::vector<uint8_t>& ids);

  /// Reads a registered group.  Values follow the id order given at
  /// registration time.  Must be called with the GIL released.
  std::vector<int32_t> sync_read(int handle, int retries);

  /// Writes a registered group.  `values` must match the registered id count.
  void sync_write(int handle, const std::vector<int32_t>& values, int retries);

  /// True while the group is still using Fast Sync Read.
  bool group_uses_fast(int handle) const;

  int32_t read(uint8_t id, uint16_t address, uint16_t length, bool is_signed, int retries);
  void write(uint8_t id, uint16_t address, uint16_t length, int32_t value, int retries);

  /// Model number of the motor, or throws when it does not answer.
  uint16_t ping(uint8_t id, int retries);

  std::vector<uint8_t> broadcast_ping();

  std::mutex& mutex() { return mutex_; }

 private:
  struct ReadGroup {
    uint16_t address = 0;
    uint16_t length = 0;
    bool is_signed = false;
    bool use_fast = false;
    bool fast_verified = false;
    std::vector<uint8_t> ids;
    std::unique_ptr<dynamixel::GroupSyncRead> plain;
    /// Scratch buffer for the Fast Sync Read status packet, sized once.
    std::vector<uint8_t> rx_buffer;
  };

  struct WriteGroup {
    uint16_t address = 0;
    uint16_t length = 0;
    std::vector<uint8_t> ids;
    std::unique_ptr<dynamixel::GroupSyncWrite> group;
  };

  void require_open() const;
  ReadGroup& read_group(int handle);
  const ReadGroup& read_group(int handle) const;
  WriteGroup& write_group(int handle);

  /// One plain Sync Read attempt.  True when every id delivered data.
  bool attempt_plain_read(ReadGroup& group, std::vector<int32_t>& out);

  /// One Fast Sync Read attempt.  Unlike GroupFastSyncRead it checks that the
  /// devices answered in the requested order, so a missing motor is a failed
  /// transfer instead of a silently shifted result vector.
  bool attempt_fast_read(ReadGroup& group, std::vector<int32_t>& out);

  std::string port_name_;
  int baudrate_;
  float protocol_version_;
  bool is_open_ = false;

  dynamixel::PortHandler* port_ = nullptr;    // owned, freed in the destructor
  dynamixel::PacketHandler* packet_ = nullptr;  // owned by the SDK singleton

  std::vector<std::unique_ptr<ReadGroup>> read_groups_;
  std::vector<std::unique_ptr<WriteGroup>> write_groups_;

  std::mutex mutex_;
};

/// Reads several groups, one per bus, in parallel worker threads.
/// `targets` pairs a bus with one of its read-group handles.
std::vector<std::vector<int32_t>> sync_read_parallel(
    const std::vector<std::pair<Bus*, int>>& targets, int retries);

/// Writes several groups, one per bus, in parallel worker threads.
void sync_write_parallel(
    const std::vector<std::tuple<Bus*, int, std::vector<int32_t>>>& targets, int retries);

}  // namespace robopy_dxl
