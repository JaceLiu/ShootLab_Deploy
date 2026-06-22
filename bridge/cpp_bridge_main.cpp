#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <dds/dds.h>
#include "policy_bridge.h"

namespace bridge {

constexpr int kNumJoints = 29;
constexpr int kRemoteRawBytes = 24;
static const std::string kHgCmdTopic = "rt/lowcmd";
static const std::string kHgStateTopic = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? data_ : nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  mutable std::shared_mutex mutex_;
};

struct BridgeStateFrame {
  uint64_t timestamp_us = 0;
  uint64_t tick = 0;
  std::vector<float> q = std::vector<float>(kNumJoints, 0.0f);
  std::vector<float> dq = std::vector<float>(kNumJoints, 0.0f);
  std::vector<float> imu_quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> imu_gyro = {0.0f, 0.0f, 0.0f};
  std::vector<uint8_t> remote_raw = std::vector<uint8_t>(kRemoteRawBytes, 0);
};

struct BridgeCmdFrame {
  uint64_t timestamp_us = 0;
  uint64_t seq = 0;
  std::vector<float> q_des = std::vector<float>(kNumJoints, 0.0f);
  std::vector<float> kp = std::vector<float>(kNumJoints, 0.0f);
  std::vector<float> kd = std::vector<float>(kNumJoints, 0.0f);
  bool request_damping = false;
  bool exit_requested = false;
};

struct BridgeConfig {
  int domain_id = 0;
  std::string network_interface = "eth0";
  std::string lowstate_topic = "rt/lowstate";
  std::string lowcmd_topic = "rt/lowcmd";
  std::string bridge_state_topic = "rt/policy_bridge_state";
  std::string bridge_cmd_topic = "rt/policy_bridge_cmd";
  std::chrono::milliseconds loop_period{2};
  std::chrono::milliseconds cmd_timeout{100};
};

static std::string GetEnvOrDefault(const char* name, const std::string& default_value) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : default_value;
}

static int GetEnvOrDefaultInt(const char* name, int default_value) {
  const char* value = std::getenv(name);
  return value ? std::atoi(value) : default_value;
}

class UnitreeIoAdapter {
 public:
  bool Init(const BridgeConfig& config) {
    config_ = config;
    ChannelFactory::Instance()->Init(0, config.network_interface);

    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(config.lowcmd_topic));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(config.lowstate_topic));
    lowstate_subscriber_->InitChannel(
        std::bind(&UnitreeIoAdapter::LowStateHandler, this, std::placeholders::_1), 1);

    low_cmd_.mode_pr() = mode_pr_;
    low_cmd_.mode_machine() = mode_machine_;
    for (auto& motor : low_cmd_.motor_cmd()) {
      motor.mode() = 1;
      motor.q() = 0.0F;
      motor.dq() = 0.0F;
      motor.kp() = 0.0F;
      motor.kd() = 0.0F;
      motor.tau() = 0.0F;
    }

    return true;
  }

  bool ReadLowState(BridgeStateFrame& out_state) {
    const auto low_state = latest_lowstate_.GetData();
    if (!low_state) {
      return false;
    }

    out_state.timestamp_us = NowUs();
    out_state.tick = low_state->tick();
    for (int i = 0; i < kNumJoints; ++i) {
      out_state.q[i] = low_state->motor_state()[i].q();
      out_state.dq[i] = low_state->motor_state()[i].dq();
    }
    const auto& quat = low_state->imu_state().quaternion();
    for (size_t i = 0; i < out_state.imu_quat_wxyz.size(); ++i) {
      out_state.imu_quat_wxyz[i] = quat[i];
    }
    const auto& gyro = low_state->imu_state().gyroscope();
    for (size_t i = 0; i < out_state.imu_gyro.size(); ++i) {
      out_state.imu_gyro[i] = gyro[i];
    }
    std::memcpy(out_state.remote_raw.data(), low_state->wireless_remote().data(), kRemoteRawBytes);
    return true;
  }

  void SendPolicyCommand(const BridgeCmdFrame& cmd) {
    low_cmd_.mode_pr() = mode_pr_;
    low_cmd_.mode_machine() = mode_machine_;
    for (int i = 0; i < kNumJoints; ++i) {
      auto& motor = low_cmd_.motor_cmd().at(i);
      motor.mode() = 1;
      motor.q() = cmd.q_des.at(i);
      motor.dq() = 0.0F;
      motor.kp() = cmd.kp.at(i);
      motor.kd() = cmd.kd.at(i);
      motor.tau() = 0.0F;
    }
    low_cmd_.crc() = Crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_), (sizeof(low_cmd_) >> 2) - 1);
    lowcmd_publisher_->Write(low_cmd_);
  }

  void SendDampingCommand() {
    low_cmd_.mode_pr() = mode_pr_;
    low_cmd_.mode_machine() = mode_machine_;
    for (int i = 0; i < kNumJoints; ++i) {
      auto& motor = low_cmd_.motor_cmd().at(i);
      motor.mode() = 1;
      motor.q() = 0.0F;
      motor.dq() = 0.0F;
      motor.kp() = 0.0F;
      motor.kd() = 8.0F;
      motor.tau() = 0.0F;
    }
    low_cmd_.crc() = Crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_), (sizeof(low_cmd_) >> 2) - 1);
    lowcmd_publisher_->Write(low_cmd_);
  }

 private:
  static uint64_t NowUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  static uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1U << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (crc & 0x80000000) {
          crc <<= 1;
          crc ^= polynomial;
        } else {
          crc <<= 1;
        }
        if (data & xbit) {
          crc ^= polynomial;
        }
        xbit >>= 1;
      }
    }
    return crc;
  }

  void LowStateHandler(const void* message) {
    const auto low_state = *(const LowState_*)message;
    latest_lowstate_.SetData(low_state);
    mode_machine_ = low_state.mode_machine();
  }

  BridgeConfig config_;
  uint8_t mode_pr_ = 0;
  uint8_t mode_machine_ = 0;
  LowCmd_ low_cmd_;
  DataBuffer<LowState_> latest_lowstate_;
  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
};

class DdsBridgeTransport {
 public:
  bool Init(const BridgeConfig& config) {
    config_ = config;

    participant_ = dds_create_participant(config.domain_id, nullptr, nullptr);
    if (participant_ < 0) {
      std::cerr << "dds_create_participant failed: " << participant_ << "\n";
      return false;
    }

    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

    state_topic_ = dds_create_topic(
        participant_, &mjlab_msg_dds__BridgeState__desc, config.bridge_state_topic.c_str(), qos, nullptr);
    cmd_topic_ = dds_create_topic(
        participant_, &mjlab_msg_dds__BridgeCmd__desc, config.bridge_cmd_topic.c_str(), qos, nullptr);
    state_writer_ = dds_create_writer(participant_, state_topic_, qos, nullptr);
    cmd_reader_ = dds_create_reader(participant_, cmd_topic_, qos, nullptr);
    dds_delete_qos(qos);

    if (state_topic_ < 0 || cmd_topic_ < 0 || state_writer_ < 0 || cmd_reader_ < 0) {
      std::cerr << "Failed to create bridge DDS entities.\n";
      return false;
    }
    return true;
  }

  void PublishState(const BridgeStateFrame& state) {
    mjlab_msg_dds__BridgeState_ msg{};
    msg.timestamp_us = state.timestamp_us;
    msg.tick = state.tick;
    AssignFloatSeq(msg.q, state.q);
    AssignFloatSeq(msg.dq, state.dq);
    AssignFloatSeq(msg.imu_quat_wxyz, state.imu_quat_wxyz);
    AssignFloatSeq(msg.imu_gyro, state.imu_gyro);
    AssignOctetSeq(msg.remote_raw, state.remote_raw);
    dds_write(state_writer_, &msg);
  }

  bool TakeLatestCommand(BridgeCmdFrame& cmd) {
    void* samples[1];
    dds_sample_info_t infos[1];
    mjlab_msg_dds__BridgeCmd_* sample = mjlab_msg_dds__BridgeCmd___alloc();
    samples[0] = sample;

    const dds_return_t rc = dds_take(cmd_reader_, samples, infos, 1, 1);
    if (rc <= 0) {
      mjlab_msg_dds__BridgeCmd__free(sample, DDS_FREE_ALL);
      return false;
    }

    bool has_valid = infos[0].valid_data;
    if (has_valid) {
      auto* msg = static_cast<mjlab_msg_dds__BridgeCmd_*>(samples[0]);
      cmd.timestamp_us = msg->timestamp_us;
      cmd.seq = msg->seq;
      CopyFloatSeq(msg->q_des, cmd.q_des);
      CopyFloatSeq(msg->kp, cmd.kp);
      CopyFloatSeq(msg->kd, cmd.kd);
      cmd.request_damping = msg->request_damping != 0;
      cmd.exit_requested = msg->exit_requested != 0;
    }

    mjlab_msg_dds__BridgeCmd__free(sample, DDS_FREE_ALL);
    return has_valid;
  }

  ~DdsBridgeTransport() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

 private:
  static void AssignFloatSeq(dds_sequence_float& seq, const std::vector<float>& values) {
    seq._length = static_cast<uint32_t>(values.size());
    seq._maximum = seq._length;
    seq._release = false;
    seq._buffer = const_cast<float*>(values.data());
  }

  static void AssignOctetSeq(dds_sequence_octet& seq, const std::vector<uint8_t>& values) {
    seq._length = static_cast<uint32_t>(values.size());
    seq._maximum = seq._length;
    seq._release = false;
    seq._buffer = const_cast<uint8_t*>(values.data());
  }

  static void CopyFloatSeq(const dds_sequence_float& seq, std::vector<float>& out) {
    const size_t n = std::min(out.size(), static_cast<size_t>(seq._length));
    for (size_t i = 0; i < n; ++i) {
      out[i] = seq._buffer[i];
    }
  }

  BridgeConfig config_;
  dds_entity_t participant_{-1};
  dds_entity_t state_topic_{-1};
  dds_entity_t cmd_topic_{-1};
  dds_entity_t state_writer_{-1};
  dds_entity_t cmd_reader_{-1};
};

class Sdk2BridgeApp {
 public:
  explicit Sdk2BridgeApp(BridgeConfig config) : config_(std::move(config)) {}

  int Run() {
    if (!io_.Init(config_)) {
      std::cerr << "Failed to initialize Unitree IO adapter.\n";
      return 1;
    }
    if (!transport_.Init(config_)) {
      std::cerr << "Failed to initialize DDS bridge transport.\n";
      return 1;
    }

    const auto loop_period = config_.loop_period;
    auto last_cmd_time = Clock::now();
    BridgeCmdFrame latest_cmd;

    running_.store(true);
    while (running_.load()) {
      const auto loop_start = Clock::now();

      BridgeStateFrame state;
      if (io_.ReadLowState(state)) {
        transport_.PublishState(state);
      }

      BridgeCmdFrame cmd_candidate;
      if (transport_.TakeLatestCommand(cmd_candidate)) {
        latest_cmd = std::move(cmd_candidate);
        last_cmd_time = Clock::now();
      }

      const auto cmd_age = std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now() - last_cmd_time);
      if (latest_cmd.exit_requested || latest_cmd.request_damping ||
          cmd_age > config_.cmd_timeout) {
        io_.SendDampingCommand();
      } else {
        io_.SendPolicyCommand(latest_cmd);
      }

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now() - loop_start);
      if (elapsed < loop_period) {
        std::this_thread::sleep_for(loop_period - elapsed);
      }
    }

    io_.SendDampingCommand();
    return 0;
  }

 private:
  using Clock = std::chrono::steady_clock;

  BridgeConfig config_;
  UnitreeIoAdapter io_;
  DdsBridgeTransport transport_;
  std::atomic<bool> running_{false};
};

}  // namespace bridge

int main() {
  std::cout
      << "SDK2 bridge scaffold created. LowState/LowCmd wiring is active; "
         "BridgeState/BridgeCmd DDS transport still requires generated C++ types.\n";
  bridge::BridgeConfig config;
  config.domain_id = bridge::GetEnvOrDefaultInt("BRIDGE_DOMAIN_ID", config.domain_id);
  config.network_interface = bridge::GetEnvOrDefault("BRIDGE_NETWORK_INTERFACE", config.network_interface);
  config.lowstate_topic = bridge::GetEnvOrDefault("BRIDGE_LOWSTATE_TOPIC", config.lowstate_topic);
  config.lowcmd_topic = bridge::GetEnvOrDefault("BRIDGE_LOWCMD_TOPIC", config.lowcmd_topic);
  config.bridge_state_topic = bridge::GetEnvOrDefault("BRIDGE_STATE_TOPIC", config.bridge_state_topic);
  config.bridge_cmd_topic = bridge::GetEnvOrDefault("BRIDGE_CMD_TOPIC", config.bridge_cmd_topic);
  config.loop_period = std::chrono::milliseconds(
      bridge::GetEnvOrDefaultInt("BRIDGE_LOOP_PERIOD_MS", static_cast<int>(config.loop_period.count())));
  config.cmd_timeout = std::chrono::milliseconds(
      bridge::GetEnvOrDefaultInt("BRIDGE_CMD_TIMEOUT_MS", static_cast<int>(config.cmd_timeout.count())));
  bridge::Sdk2BridgeApp app(config);
  return app.Run();
}
