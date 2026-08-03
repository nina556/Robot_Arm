#include <ecat/task.hpp>
#include <qiuniu/init.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kJointCount = 7;
constexpr std::int64_t kCycleTimeNs = 1000000;
constexpr double kPi = 3.14159265358979323846;

// 每根轴只保存读取反馈所需的 TxPDO 指针。
struct AxisFeedback {
  std::uint16_t slave_pos = 0;
  volatile std::uint16_t *control_word = nullptr;
  volatile std::int8_t *mode = nullptr;
  volatile std::int32_t *target_position = nullptr;
  volatile std::uint32_t *profile_velocity = nullptr;
  volatile std::uint32_t *profile_acceleration = nullptr;
  volatile std::uint32_t *profile_deceleration = nullptr;
  volatile const std::int32_t *position = nullptr;
  volatile const std::int32_t *velocity = nullptr;
  volatile const std::uint16_t *status_word = nullptr;
  volatile const std::uint16_t *error_code = nullptr;
};

template <typename T>
void require_size(const std::vector<T> &values, const char *name) {
  if (values.size() != kJointCount) {
    throw std::runtime_error(std::string(name) + " 必须包含 7 个元素");
  }
}

}  // namespace

class MasterEncoderPublisher final : public rclcpp::Node {
public:
  MasterEncoderPublisher()
      : Node("master_encoder_publisher") {
    master_id_ = declare_parameter<int>("master_id", 2);
    eni_path_ = declare_parameter<std::string>(
        "eni_path",
        "/home/niic/workspace/CSP_direct_drive/src/light_test/config/NIIC_ENI_M2.xml");
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "master2");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"Right_Joint1", "Right_Joint2", "Right_Joint3", "Right_Joint4",
         "Right_Joint5", "Right_Joint6", "Right_Joint7"});
    zero_offsets_ = declare_parameter<std::vector<std::int64_t>>(
        "zero_offsets", std::vector<std::int64_t>(kJointCount, 0));
    directions_ = declare_parameter<std::vector<std::int64_t>>(
        "directions", std::vector<std::int64_t>(kJointCount, 1));
    counts_per_rev_ = declare_parameter<std::vector<double>>(
        "encoder_counts_per_rev",
        std::vector<double>(kJointCount, 8388608.0));
    gear_ratios_ = declare_parameter<std::vector<double>>(
        "gear_ratios", std::vector<double>(kJointCount, 1.0));
    const double publish_rate =
        declare_parameter<double>("publish_rate_hz", 100.0);

    require_size(joint_names_, "joint_names");
    require_size(zero_offsets_, "zero_offsets");
    require_size(directions_, "directions");
    require_size(counts_per_rev_, "encoder_counts_per_rev");
    require_size(gear_ratios_, "gear_ratios");
    if (publish_rate <= 0.0) {
      throw std::runtime_error("publish_rate_hz 必须大于 0");
    }

    const std::string joint_topic = "/" + topic_prefix_ + "/joint_states";
    const std::string raw_topic = "/" + topic_prefix_ + "/encoder_counts";
    const std::string inject_topic = "/" + topic_prefix_ + "/inject_offsets";

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        joint_topic, 10);
    raw_count_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        raw_topic, 10);

    // 话题直注: Web UI 发布 Int64MultiArray 到 /<prefix>/inject_offsets, 即时生效
    inject_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        inject_topic, 10,
        [this](std_msgs::msg::Int64MultiArray::ConstSharedPtr msg) {
          if (msg->data.size() != kJointCount) {
            RCLCPP_ERROR(get_logger(), "inject_offsets 需要 %zu 个值，收到 %zu",
                         kJointCount, msg->data.size());
            return;
          }
          for (std::size_t i = 0; i < kJointCount; ++i) {
            zero_offsets_[i] = static_cast<std::int64_t>(msg->data[i]);
          }
          RCLCPP_INFO(get_logger(), "zero_offsets 实时注入: [%ld %ld %ld %ld %ld %ld %ld]",
                      zero_offsets_[0], zero_offsets_[1], zero_offsets_[2],
                      zero_offsets_[3], zero_offsets_[4], zero_offsets_[5], zero_offsets_[6]);
        });

    // 动态参数回调: ros2 param set 实时更新零点, 无需重启
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (const auto &p : params) {
            if (p.get_name() == "zero_offsets") {
              zero_offsets_ = p.as_integer_array();
              require_size(zero_offsets_, "zero_offsets");
              RCLCPP_INFO(get_logger(), "zero_offsets 实时更新: [%ld %ld %ld %ld %ld %ld %ld]",
                          zero_offsets_[0], zero_offsets_[1], zero_offsets_[2],
                          zero_offsets_[3], zero_offsets_[4], zero_offsets_[5], zero_offsets_[6]);
            }
            if (p.get_name() == "directions") {
              directions_ = p.as_integer_array();
              require_size(directions_, "directions");
            }
          }
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          return result;
        });

    task_ = std::make_unique<ecat::task>(master_id_);
    task_->priority(80);
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(2, &cpus);
    task_->cpu_affinity(&cpus, sizeof(cpus));
    task_->record(false);
    std::int64_t cycle_time = kCycleTimeNs;
    task_->load_eni(eni_path_.c_str(), cycle_time);

    task_->set_config_callback([this] { register_all_axes(); });
    task_->set_send_callback([this] {
      read_all_axes();
      keep_all_axes_disabled();
    });
    task_->start();

    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate));

    RCLCPP_INFO(get_logger(),
                "Master %d 编码器发布器已启动，等待 7 轴反馈", master_id_);
  }

  ~MasterEncoderPublisher() override {
    if (task_) {
      task_->break_();
      task_->wait();
      task_->release();
    }
  }

  void publish_once() {
    publish();
  }

  std::chrono::nanoseconds publish_period() const {
    return publish_period_;
  }

private:
  void register_all_axes() {
    std::size_t axis_id = 0;
    for (std::uint16_t slave = 0;
         slave < task_->slave_count() && axis_id < kJointCount; ++slave) {
      if (task_->profile_no(slave) != 402) {
        continue;
      }

      auto &axis = axes_[axis_id];
      axis.slave_pos = slave;
      const bool position_ok = task_->try_register_pdo_entry(
          axis.position, slave, {0x6064, 0});
      const bool output_ok =
          task_->try_register_pdo_entry(
              axis.control_word, slave, {0x6040, 0}) &&
          task_->try_register_pdo_entry(axis.mode, slave, {0x6060, 0}) &&
          task_->try_register_pdo_entry(
              axis.target_position, slave, {0x607a, 0});
      task_->try_register_pdo_entry(
          axis.profile_velocity, slave, {0x6081, 0});
      task_->try_register_pdo_entry(
          axis.profile_acceleration, slave, {0x6083, 0});
      task_->try_register_pdo_entry(
          axis.profile_deceleration, slave, {0x6084, 0});
      task_->try_register_pdo_entry(axis.velocity, slave, {0x606c, 0});
      task_->try_register_pdo_entry(axis.status_word, slave, {0x6041, 0});
      task_->try_register_pdo_entry(axis.error_code, slave, {0x603f, 0});

      RCLCPP_INFO(get_logger(),
                  "轴 %zu -> slave_pos=%u，输入PDO=%d，输出PDO=%d",
                  axis_id, slave, position_ok, output_ok);
      if (position_ok && output_ok) {
        ++axis_id;
      }
    }
    axis_count_.store(axis_id, std::memory_order_release);
    if (axis_id != kJointCount) {
      RCLCPP_ERROR(get_logger(), "只找到 %zu/7 根有效轴", axis_id);
    }
  }

  void read_all_axes() {
    const auto count = axis_count_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
      const auto &axis = axes_[i];
      positions_[i].store(
          axis.position ? *axis.position : 0, std::memory_order_relaxed);
      velocities_[i].store(
          axis.velocity ? *axis.velocity : 0, std::memory_order_relaxed);
      status_words_[i].store(
          axis.status_word ? *axis.status_word : 0,
          std::memory_order_relaxed);
      error_codes_[i].store(
          axis.error_code ? *axis.error_code : 0,
          std::memory_order_relaxed);
    }
    samples_.fetch_add(1, std::memory_order_release);
  }

  void keep_all_axes_disabled() {
    const auto count = axis_count_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
      auto &axis = axes_[i];
      if (!axis.control_word || !axis.target_position || !axis.position) {
        continue;
      }

      // 0x0006 只进入 Ready To Switch On，不会使能电机。
      *axis.control_word = 0x0006;
      if (axis.mode) {
        *axis.mode = 8;
      }
      *axis.target_position = *axis.position;
      if (axis.profile_velocity) {
        *axis.profile_velocity = 1000;
      }
      if (axis.profile_acceleration) {
        *axis.profile_acceleration = 1000;
      }
      if (axis.profile_deceleration) {
        *axis.profile_deceleration = 1000;
      }
    }
  }

  double count_to_rad(std::size_t i, std::int32_t count) const {
    const double denominator = counts_per_rev_[i] * gear_ratios_[i];
    if (denominator <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(directions_[i]) *
           (static_cast<double>(count) -
            static_cast<double>(zero_offsets_[i])) *
           (2.0 * kPi) / denominator;
  }

  double velocity_to_rad_s(std::size_t i, std::int32_t velocity) const {
    const double denominator = counts_per_rev_[i] * gear_ratios_[i];
    if (denominator <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(directions_[i]) *
           static_cast<double>(velocity) * (2.0 * kPi) / denominator;
  }

  void publish() {
    const auto count = axis_count_.load(std::memory_order_acquire);
    if (count != kJointCount) {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = joint_names_;
    joint_state.position.resize(kJointCount);
    joint_state.velocity.resize(kJointCount);

    std_msgs::msg::Int64MultiArray raw_counts;
    raw_counts.data.resize(kJointCount);

    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto &axis = axes_[i];
      const auto position =
          axis.position ? *axis.position : positions_[i].load();
      const auto velocity =
          axis.velocity ? *axis.velocity : velocities_[i].load();
      joint_state.position[i] = count_to_rad(i, position);
      joint_state.velocity[i] = velocity_to_rad_s(i, velocity);
      raw_counts.data[i] = position;

      const auto error = error_codes_[i].load(std::memory_order_relaxed);
      if (error != 0) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "轴 %zu 驱动器错误码：0x%04x，状态字：0x%04x",
            i, error, status_words_[i].load(std::memory_order_relaxed));
      }
    }

    joint_state_pub_->publish(joint_state);
    raw_count_pub_->publish(raw_counts);
  }

  std::unique_ptr<ecat::task> task_;
  std::array<AxisFeedback, kJointCount> axes_{};
  std::array<std::atomic<std::int32_t>, kJointCount> positions_{};
  std::array<std::atomic<std::int32_t>, kJointCount> velocities_{};
  std::array<std::atomic<std::uint16_t>, kJointCount> status_words_{};
  std::array<std::atomic<std::uint16_t>, kJointCount> error_codes_{};
  std::atomic<std::size_t> axis_count_{0};
  std::atomic<std::uint64_t> samples_{0};

  int master_id_ = 2;
  std::string eni_path_;
  std::string topic_prefix_;
  std::vector<std::string> joint_names_;
  std::vector<std::int64_t> zero_offsets_;
  std::vector<std::int64_t> directions_;
  std::vector<double> counts_per_rev_;
  std::vector<double> gear_ratios_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr raw_count_pub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr inject_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::chrono::nanoseconds publish_period_{std::chrono::milliseconds(10)};
};

int main(int argc, char **argv) {
  qiuniu_init();
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MasterEncoderPublisher>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->publish_once();
    std::this_thread::sleep_for(node->publish_period());
  }
  rclcpp::shutdown();
  return 0;
}
