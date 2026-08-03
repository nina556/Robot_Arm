#include <ecat/task.hpp>
#include <qiuniu/init.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

constexpr std::size_t kJointCount = 7;
constexpr int kMasterId = 2;
constexpr std::int64_t kCycleTimeNs = 4000000;
constexpr std::uint32_t kFeedbackSettleCycles = 2000;
constexpr double kPi = 3.14159265358979323846;
constexpr const char *kEniPath =
    ENI_FILE_PATH_MACRO;

struct Axis {
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
    throw std::runtime_error(std::string(name) + " must contain 7 values");
  }
}

bool operation_enabled(std::uint16_t status) {
  return (status & 0x006f) == 0x0027;
}

double duration_seconds(const builtin_interfaces::msg::Duration &duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

}  // namespace

class Master2MoveItBridge final : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle =
      rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  Master2MoveItBridge() : Node("master2_moveit_bridge") {
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
    limit_enabled_ = declare_parameter<std::vector<bool>>(
        "limit_enabled", std::vector<bool>(kJointCount, false));
    raw_limit_a_ = declare_parameter<std::vector<std::int64_t>>(
        "raw_limit_a", std::vector<std::int64_t>(kJointCount, 0));
    raw_limit_b_ = declare_parameter<std::vector<std::int64_t>>(
        "raw_limit_b", std::vector<std::int64_t>(kJointCount, 0));
    const auto configured_max_step =
        declare_parameter<std::int64_t>("max_step_per_cycle", 100);
    nominal_step_per_cycle_ = configured_max_step;
    max_step_per_cycle_.store(configured_max_step);
    following_error_counts_ =
        declare_parameter<std::int64_t>("following_error_counts", 250000);
    goal_tolerance_counts_ =
        declare_parameter<std::int64_t>("goal_tolerance_counts", 5000);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 5.0);
    const double publish_rate =
        declare_parameter<double>("publish_rate_hz", 100.0);

    require_size(joint_names_, "joint_names");
    require_size(zero_offsets_, "zero_offsets");
    require_size(directions_, "directions");
    require_size(counts_per_rev_, "encoder_counts_per_rev");
    require_size(gear_ratios_, "gear_ratios");
    require_size(limit_enabled_, "limit_enabled");
    require_size(raw_limit_a_, "raw_limit_a");
    require_size(raw_limit_b_, "raw_limit_b");
    if (configured_max_step <= 0 || configured_max_step > 600 ||
        following_error_counts_ <= 0 ||
        goal_tolerance_counts_ <= 0 || goal_timeout_sec_ <= 0.0 ||
        publish_rate <= 0.0) {
      throw std::runtime_error("safety and timing parameters must be positive");
    }

    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (directions_[i] != -1 && directions_[i] != 1) {
        throw std::runtime_error("directions values must be -1 or 1");
      }
      if (counts_per_rev_[i] <= 0.0 || gear_ratios_[i] <= 0.0) {
        throw std::runtime_error("encoder scale values must be positive");
      }
      raw_min_[i] = std::min(raw_limit_a_[i], raw_limit_b_[i]);
      raw_max_[i] = std::max(raw_limit_a_[i], raw_limit_b_[i]);
      if (limit_enabled_[i] && raw_min_[i] == raw_max_[i]) {
        throw std::runtime_error("enabled raw limits must span a range");
      }
      name_to_index_[joint_names_[i]] = i;
    }

    parameter_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters) {
            if (parameter.get_name() != "max_step_per_cycle") {
              continue;
            }
            const auto value = parameter.as_int();
            if (value < 100 || value > 600) {
              result.successful = false;
              result.reason =
                  "max_step_per_cycle must be between 100 and 600";
              return result;
            }
            max_step_per_cycle_.store(value, std::memory_order_release);
            RCLCPP_WARN(
                get_logger(),
                "Dynamic speed changed: %ld counts/cycle (10 ms cycle, %.0f%%)",
                value,
                100.0 * static_cast<double>(value) /
                    static_cast<double>(nominal_step_per_cycle_));
          }
          return result;
        });

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/master2/joint_states", 10);
    raw_count_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        "/master2/encoder_counts", 10);
    estop_state_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/master2/emergency_stop_state", rclcpp::QoS(1).transient_local());
    enabled_state_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/master2/motors_enabled", rclcpp::QoS(1).transient_local());

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/master2/emergency_stop", rclcpp::QoS(10),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (msg->data) {
            trip_estop("emergency stop topic");
          }
        });
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/master2/joint_trajectory", 10,
        [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
          std::string error;
          if (!controller_enabled_.load()) {
            RCLCPP_ERROR(get_logger(), "trajectory rejected: motors are disabled");
            return;
          }
          if (!validate_trajectory(*msg, error)) {
            RCLCPP_ERROR(get_logger(), "trajectory rejected: %s", error.c_str());
            return;
          }
          const auto generation = execution_generation_.fetch_add(1) + 1;
          std::thread(&Master2MoveItBridge::execute_topic_trajectory, this,
                      *msg, generation).detach();
        });

    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "/master2/enable_motors",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (!request->data) {
            execution_generation_.fetch_add(1);
            controller_enabled_.store(false);
            target_valid_.store(false);
            response->success = true;
            response->message = "motors disabled";
            return;
          }
          std::string error;
          if (estop_latched_.load()) {
            error = "emergency stop is latched";
          } else if (!feedback_ready_.load()) {
            error = "EtherCAT feedback is not ready";
          } else if (!positions_safe(error)) {
          } else {
            for (std::size_t i = 0; i < kJointCount; ++i) {
              targets_[i].store(actual_positions_[i].load());
              commanded_[i].store(actual_positions_[i].load());
            }
            target_valid_.store(true);
            controller_enabled_.store(true);
            response->success = true;
            response->message =
                "all axes enabled; continuous joints use nearest-turn mapping";
            return;
          }
          response->success = false;
          response->message = error;
        });
    reset_estop_service_ = create_service<std_srvs::srv::Trigger>(
        "/master2/reset_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::string error;
          if (!feedback_ready_.load()) {
            error = "EtherCAT feedback is not ready";
          } else if (!positions_safe(error)) {
          } else {
            bool drive_error = false;
            for (const auto &value : error_codes_) {
              drive_error = drive_error || value.load() != 0;
            }
            if (drive_error) {
              error = "a drive still reports an error";
            }
          }
          if (!error.empty()) {
            response->success = false;
            response->message = error;
            return;
          }
          estop_reason_code_.store(0);
          estop_latched_.store(false);
          response->success = true;
          response->message = "emergency stop reset; motors remain disabled";
        });

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "/arm_controller/follow_joint_trajectory",
        std::bind(&Master2MoveItBridge::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&Master2MoveItBridge::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&Master2MoveItBridge::handle_accepted, this,
                  std::placeholders::_1));

    task_ = std::make_unique<ecat::task>(kMasterId);
    task_->priority(80);
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(2, &cpus);
    task_->cpu_affinity(&cpus, sizeof(cpus));
    task_->record(false);
    std::int64_t cycle_time = kCycleTimeNs;
    task_->load_eni(kEniPath, cycle_time);
    task_->set_config_callback([this] { register_all_axes(); });
    task_->set_cycle_callback([this] { update_all_axes(); });
    task_->start();

    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate)),
        std::bind(&Master2MoveItBridge::publish_state, this));

    RCLCPP_WARN(get_logger(),
                "Controller started DISABLED. J3 and J5 are continuous "
                "joints without software position limits.");
  }

  ~Master2MoveItBridge() override {
    trip_estop("controller shutdown");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (task_) {
      task_->break_();
      task_->wait();
      task_->release();
    }
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
      const bool ok =
          task_->try_register_pdo_entry(axis.control_word, slave, {0x6040, 0}) &&
          task_->try_register_pdo_entry(axis.mode, slave, {0x6060, 0}) &&
          task_->try_register_pdo_entry(
              axis.target_position, slave, {0x607a, 0}) &&
          task_->try_register_pdo_entry(axis.position, slave, {0x6064, 0}) &&
          task_->try_register_pdo_entry(axis.status_word, slave, {0x6041, 0});
      task_->try_register_pdo_entry(
          axis.profile_velocity, slave, {0x6081, 0});
      task_->try_register_pdo_entry(
          axis.profile_acceleration, slave, {0x6083, 0});
      task_->try_register_pdo_entry(
          axis.profile_deceleration, slave, {0x6084, 0});
      task_->try_register_pdo_entry(axis.velocity, slave, {0x606c, 0});
      task_->try_register_pdo_entry(axis.error_code, slave, {0x603f, 0});
      RCLCPP_INFO(get_logger(), "axis %zu slave=%u PDO ready=%d",
                  axis_id + 1, slave, ok);
      if (ok) {
        ++axis_id;
      }
    }
    axis_count_.store(axis_id);
    feedback_ready_.store(false);
    feedback_settle_cycles_ = 0;
    if (axis_id != kJointCount) {
      trip_estop("not all seven axes were found");
    }
  }

  void update_all_axes() {
    const auto count = axis_count_.load(std::memory_order_acquire);
    if (count != kJointCount) {
      return;
    }

    // EtherCAT PDO values can be zero/stale for a short period after a master
    // restart. Collect complete feedback first and keep every drive disabled;
    // safety limits become active only after the stream has settled.
    for (std::size_t i = 0; i < count; ++i) {
      auto &axis = axes_[i];
      if (!axis.control_word || !axis.target_position || !axis.position ||
          !axis.status_word) {
        feedback_settle_cycles_ = 0;
        feedback_ready_.store(false, std::memory_order_relaxed);
        return;
      }
      const auto actual = *axis.position;
      actual_positions_[i].store(actual, std::memory_order_relaxed);
      actual_velocities_[i].store(
          axis.velocity ? *axis.velocity : 0, std::memory_order_relaxed);
      status_words_[i].store(*axis.status_word, std::memory_order_relaxed);
      error_codes_[i].store(
          axis.error_code ? *axis.error_code : 0, std::memory_order_relaxed);
    }
    if (feedback_settle_cycles_ < kFeedbackSettleCycles) {
      ++feedback_settle_cycles_;
      for (std::size_t i = 0; i < count; ++i) {
        auto &axis = axes_[i];
        const auto actual = actual_positions_[i].load(std::memory_order_relaxed);
        *axis.target_position = actual;
        *axis.control_word = 0x0006;
        commanded_[i].store(actual, std::memory_order_relaxed);
        targets_[i].store(actual, std::memory_order_relaxed);
      }
      return;
    }
    feedback_ready_.store(true, std::memory_order_release);

    for (std::size_t i = 0; i < count; ++i) {
      auto &axis = axes_[i];
      if (!axis.control_word || !axis.target_position || !axis.position ||
          !axis.status_word) {
        continue;
      }
      const auto actual = *axis.position;
      const auto status = *axis.status_word;
      const auto error = axis.error_code ? *axis.error_code : 0;
      actual_positions_[i].store(actual, std::memory_order_relaxed);
      actual_velocities_[i].store(
          axis.velocity ? *axis.velocity : 0, std::memory_order_relaxed);
      status_words_[i].store(status, std::memory_order_relaxed);
      error_codes_[i].store(error, std::memory_order_relaxed);

      if (error != 0) {
        trip_estop_rt(100 + static_cast<int>(i));
      }
      if (limit_enabled_[i] &&
          (actual < raw_min_[i] || actual > raw_max_[i])) {
        trip_estop_rt(200 + static_cast<int>(i));
      }

      const bool enabled = operation_enabled(status);
      operation_enabled_[i].store(enabled, std::memory_order_relaxed);
      if (estop_latched_.load(std::memory_order_relaxed)) {
        *axis.target_position = actual;
        *axis.control_word = enabled ? 0x000b : 0x0006;
        continue;
      }
      if (!controller_enabled_.load(std::memory_order_relaxed)) {
        *axis.target_position = actual;
        commanded_[i].store(actual, std::memory_order_relaxed);
        *axis.control_word = enabled ? 0x0007 : 0x0006;
        continue;
      }

      if (status & 0x0008) {
        *axis.control_word = 0x0080;
      } else if ((status & 0x004f) == 0x0040) {
        *axis.control_word = 0x0006;
      } else if ((status & 0x006f) == 0x0021) {
        *axis.control_word = 0x0007;
      } else if ((status & 0x006f) == 0x0023) {
        if (axis.mode) {
          *axis.mode = 8;
        }
        commanded_[i].store(actual, std::memory_order_relaxed);
        *axis.target_position = actual;
        *axis.control_word = 0x000f;
      } else if (!enabled) {
        *axis.control_word = 0x0006;
      } else {
        *axis.control_word = 0x000f;
        const auto desired = target_valid_.load(std::memory_order_relaxed)
                                 ? targets_[i].load(std::memory_order_relaxed)
                                 : actual;
        if (limit_enabled_[i] &&
            (desired < raw_min_[i] || desired > raw_max_[i])) {
          trip_estop_rt(300 + static_cast<int>(i));
          *axis.target_position = actual;
          continue;
        }
        const auto current = commanded_[i].load(std::memory_order_relaxed);
        const std::int64_t difference =
            static_cast<std::int64_t>(desired) - current;
        const auto max_step =
            max_step_per_cycle_.load(std::memory_order_relaxed);
        const auto step = std::clamp<std::int64_t>(
            difference, -max_step, max_step);
        const auto next =
            static_cast<std::int32_t>(static_cast<std::int64_t>(current) + step);
        commanded_[i].store(next, std::memory_order_relaxed);
        *axis.target_position = next;
        if (std::llabs(static_cast<std::int64_t>(actual) - next) >
            following_error_counts_) {
          trip_estop_rt(400 + static_cast<int>(i));
        }
      }
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

  void trip_estop_rt(int reason_code) {
    bool expected = false;
    if (!estop_latched_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
      return;
    }
    estop_reason_code_.store(reason_code, std::memory_order_relaxed);
    controller_enabled_.store(false, std::memory_order_relaxed);
    target_valid_.store(false, std::memory_order_relaxed);
    execution_generation_.fetch_add(1, std::memory_order_relaxed);
  }

  void trip_estop(const std::string &reason) {
    {
      std::lock_guard<std::mutex> lock(estop_reason_mutex_);
      estop_reason_ = reason;
    }
    trip_estop_rt(1);
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP: %s", reason.c_str());
  }

  bool positions_safe(std::string &error) const {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!limit_enabled_[i]) {
        continue;
      }
      const auto actual = actual_positions_[i].load();
      if (actual < raw_min_[i] || actual > raw_max_[i]) {
        error = joint_names_[i] + " is outside its raw encoder limits";
        return false;
      }
    }
    return true;
  }

  std::int32_t rad_to_count(
      std::size_t i, double rad,
      std::optional<std::int32_t> reference = std::nullopt) const {
    double raw =
        static_cast<double>(zero_offsets_[i]) +
        static_cast<double>(directions_[i]) * rad * counts_per_rev_[i] *
            gear_ratios_[i] / (2.0 * kPi);
    if (!limit_enabled_[i] && reference.has_value()) {
      const double counts_per_joint_turn =
          counts_per_rev_[i] * gear_ratios_[i];
      raw += std::round(
                 (static_cast<double>(*reference) - raw) /
                 counts_per_joint_turn) *
             counts_per_joint_turn;
    }
    if (!std::isfinite(raw) ||
        raw < std::numeric_limits<std::int32_t>::min() ||
        raw > std::numeric_limits<std::int32_t>::max()) {
      throw std::range_error("joint target is outside int32 encoder range");
    }
    return static_cast<std::int32_t>(std::llround(raw));
  }

  double count_to_rad(std::size_t i, std::int32_t count) const {
    const double angle =
        static_cast<double>(directions_[i]) *
        (static_cast<double>(count) -
         static_cast<double>(zero_offsets_[i])) *
        (2.0 * kPi) / (counts_per_rev_[i] * gear_ratios_[i]);
    // MoveIt represents continuous joints in the principal [-pi, pi] range.
    // Raw encoder turns remain intact internally; only the published/planning
    // angle is normalized.
    return limit_enabled_[i] ? angle : std::remainder(angle, 2.0 * kPi);
  }

  bool validate_trajectory(
      const trajectory_msgs::msg::JointTrajectory &trajectory,
      std::string &error) const {
    if (trajectory.points.empty()) {
      error = "trajectory has no points";
      return false;
    }
    if (trajectory.joint_names.empty()) {
      error = "trajectory has no joint names";
      return false;
    }
    std::vector<std::size_t> indices;
    indices.reserve(trajectory.joint_names.size());
    for (const auto &name : trajectory.joint_names) {
      const auto it = name_to_index_.find(name);
      if (it == name_to_index_.end()) {
        error = "unknown joint: " + name;
        return false;
      }
      if (std::find(indices.begin(), indices.end(), it->second) !=
          indices.end()) {
        error = "duplicate joint: " + name;
        return false;
      }
      indices.push_back(it->second);
    }

    double previous_time = -1.0;
    std::vector<std::int32_t> previous_counts(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      previous_counts[i] = actual_positions_[i].load();
    }
    for (std::size_t point_index = 0;
         point_index < trajectory.points.size(); ++point_index) {
      const auto &point = trajectory.points[point_index];
      if (point.positions.size() != indices.size()) {
        error = "point position count does not match joint_names";
        return false;
      }
      const double point_time = duration_seconds(point.time_from_start);
      if (!std::isfinite(point_time) || point_time <= previous_time) {
        error = "time_from_start must strictly increase";
        return false;
      }
      const bool zero_time_start = point_index == 0 && point_time <= 1e-9;
      const double segment_time =
          point_time - std::max(0.0, previous_time);
      for (std::size_t j = 0; j < indices.size(); ++j) {
        const auto axis = indices[j];
        if (!std::isfinite(point.positions[j])) {
          error = "trajectory contains a non-finite position";
          return false;
        }
        const auto count =
            rad_to_count(axis, point.positions[j], previous_counts[axis]);
        if (limit_enabled_[axis] &&
            (count < raw_min_[axis] || count > raw_max_[axis])) {
          error = joint_names_[axis] + " target exceeds raw encoder limits";
          return false;
        }
        if (!zero_time_start) {
          const double required_rate =
              std::abs(static_cast<double>(count - previous_counts[axis])) /
              std::max(segment_time, 1e-6);
          const double available_rate =
              static_cast<double>(nominal_step_per_cycle_) * 1000.0;
          if (required_rate > available_rate) {
            error = joint_names_[axis] +
                    " trajectory is faster than the configured count slew rate";
            return false;
          }
        }
        previous_counts[axis] = count;
      }
      previous_time = point_time;
    }
    return true;
  }

  std::array<double, kJointCount> sample_trajectory(
      const trajectory_msgs::msg::JointTrajectory &trajectory,
      const std::array<double, kJointCount> &initial, double elapsed) const {
    std::array<double, kJointCount> result = initial;
    std::size_t next = 0;
    while (next < trajectory.points.size() &&
           duration_seconds(trajectory.points[next].time_from_start) <
               elapsed) {
      ++next;
    }
    if (next >= trajectory.points.size()) {
      next = trajectory.points.size() - 1;
    }
    const auto &end = trajectory.points[next];
    const double end_time = duration_seconds(end.time_from_start);
    const trajectory_msgs::msg::JointTrajectoryPoint *start_point = nullptr;
    double start_time = 0.0;
    if (next > 0) {
      start_point = &trajectory.points[next - 1];
      start_time = duration_seconds(start_point->time_from_start);
    }
    const double alpha = std::clamp(
        (elapsed - start_time) / std::max(end_time - start_time, 1e-9),
        0.0, 1.0);
    for (std::size_t j = 0; j < trajectory.joint_names.size(); ++j) {
      const auto axis = name_to_index_.at(trajectory.joint_names[j]);
      const double start = start_point ? start_point->positions[j] : initial[axis];
      double delta = end.positions[j] - start;
      if (!limit_enabled_[axis]) {
        delta = std::remainder(delta, 2.0 * kPi);
      }
      result[axis] = start + delta * alpha;
    }
    return result;
  }

  void set_targets_from_radians(
      const std::array<double, kJointCount> &positions) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto reference = target_valid_.load()
                                 ? targets_[i].load()
                                 : actual_positions_[i].load();
      targets_[i].store(rad_to_count(i, positions[i], reference));
    }
    target_valid_.store(true);
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
    if (!controller_enabled_.load() || estop_latched_.load()) {
      RCLCPP_ERROR(get_logger(),
                   "MoveIt goal rejected: controller disabled or E-stop active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::string error;
    try {
      if (!validate_trajectory(goal->trajectory, error)) {
        RCLCPP_ERROR(get_logger(), "MoveIt goal rejected: %s", error.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "MoveIt goal rejected: %s", e.what());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>) {
    execution_generation_.fetch_add(1);
    hold_current_position();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto generation = execution_generation_.fetch_add(1) + 1;
    std::thread(&Master2MoveItBridge::execute_action, this, goal_handle,
                generation).detach();
  }

  std::array<double, kJointCount> actual_radians() const {
    std::array<double, kJointCount> values{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
      values[i] = count_to_rad(i, actual_positions_[i].load());
    }
    return values;
  }

  void hold_current_position() {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      targets_[i].store(actual_positions_[i].load());
    }
    target_valid_.store(true);
  }

  bool execute_trajectory(
      const trajectory_msgs::msg::JointTrajectory &trajectory,
      std::uint64_t generation,
      const std::function<bool(const std::array<double, kJointCount> &)> &
          feedback_callback,
      std::string &error) {
    const auto initial = actual_radians();
    const double duration =
        duration_seconds(trajectory.points.back().time_from_start);
    auto previous_tick = std::chrono::steady_clock::now();
    double trajectory_time = 0.0;
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok()) {
      if (generation != execution_generation_.load()) {
        error = "trajectory canceled or replaced";
        hold_current_position();
        return false;
      }
      if (estop_latched_.load() || !controller_enabled_.load()) {
        error = "controller disabled by emergency stop";
        return false;
      }
      const auto now_tick = std::chrono::steady_clock::now();
      const double wall_delta =
          std::chrono::duration<double>(now_tick - previous_tick).count();
      previous_tick = now_tick;
      const double speed_scale =
          static_cast<double>(
              max_step_per_cycle_.load(std::memory_order_relaxed)) /
          static_cast<double>(nominal_step_per_cycle_);
      trajectory_time += wall_delta * speed_scale;
      const auto desired =
          sample_trajectory(
              trajectory, initial, std::min(trajectory_time, duration));
      set_targets_from_radians(desired);
      if (!feedback_callback(desired)) {
        error = "trajectory canceled";
        hold_current_position();
        return false;
      }
      if (trajectory_time >= duration) {
        break;
      }
      rate.sleep();
    }

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(goal_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool reached = true;
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (std::llabs(
                static_cast<std::int64_t>(actual_positions_[i].load()) -
                targets_[i].load()) > goal_tolerance_counts_) {
          reached = false;
        }
      }
      if (reached) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    error = "goal timeout; emergency stop latched";
    trip_estop(error);
    return false;
  }

  void execute_action(
      const std::shared_ptr<GoalHandle> goal_handle,
      std::uint64_t generation) {
    std::string error;
    auto feedback_callback =
        [this, &goal_handle](const std::array<double, kJointCount> &desired) {
          if (goal_handle->is_canceling()) {
            return false;
          }
          auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
          feedback->header.stamp = now();
          feedback->joint_names = joint_names_;
          feedback->desired.positions.assign(desired.begin(), desired.end());
          const auto actual = actual_radians();
          feedback->actual.positions.assign(actual.begin(), actual.end());
          feedback->error.positions.resize(kJointCount);
          for (std::size_t i = 0; i < kJointCount; ++i) {
            feedback->error.positions[i] = desired[i] - actual[i];
          }
          goal_handle->publish_feedback(feedback);
          return true;
        };

    const bool ok = execute_trajectory(
        goal_handle->get_goal()->trajectory, generation, feedback_callback,
        error);
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    if (ok) {
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "trajectory completed";
      goal_handle->succeed(result);
    } else if (goal_handle->is_canceling()) {
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = error;
      goal_handle->canceled(result);
    } else {
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = error;
      goal_handle->abort(result);
    }
  }

  void execute_topic_trajectory(
      trajectory_msgs::msg::JointTrajectory trajectory,
      std::uint64_t generation) {
    std::string error;
    const bool ok = execute_trajectory(
        trajectory, generation,
        [](const std::array<double, kJointCount> &) { return true; }, error);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "topic trajectory failed: %s", error.c_str());
    }
  }

  void publish_state() {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = joint_names_;
    joint_state.position.resize(kJointCount);
    joint_state.velocity.resize(kJointCount);
    std_msgs::msg::Int64MultiArray raw;
    raw.data.resize(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto position = actual_positions_[i].load();
      joint_state.position[i] = count_to_rad(i, position);
      joint_state.velocity[i] =
          static_cast<double>(directions_[i]) *
          static_cast<double>(actual_velocities_[i].load()) *
          (2.0 * kPi) / (counts_per_rev_[i] * gear_ratios_[i]);
      raw.data[i] = position;
    }
    joint_state_pub_->publish(joint_state);
    raw_count_pub_->publish(raw);

    std_msgs::msg::Bool estop;
    estop.data = estop_latched_.load();
    estop_state_pub_->publish(estop);
    std_msgs::msg::Bool enabled;
    enabled.data = controller_enabled_.load();
    enabled_state_pub_->publish(enabled);

    const int reason = estop_reason_code_.exchange(0);
    if (reason >= 100) {
      RCLCPP_ERROR(get_logger(), "E-stop safety reason code: %d", reason);
    }
  }

  std::unique_ptr<ecat::task> task_;
  std::array<Axis, kJointCount> axes_{};
  std::array<std::atomic<std::int32_t>, kJointCount> actual_positions_{};
  std::array<std::atomic<std::int32_t>, kJointCount> actual_velocities_{};
  std::array<std::atomic<std::int32_t>, kJointCount> targets_{};
  std::array<std::atomic<std::int32_t>, kJointCount> commanded_{};
  std::array<std::atomic<std::uint16_t>, kJointCount> status_words_{};
  std::array<std::atomic<std::uint16_t>, kJointCount> error_codes_{};
  std::array<std::atomic<bool>, kJointCount> operation_enabled_{};
  std::array<std::int64_t, kJointCount> raw_min_{};
  std::array<std::int64_t, kJointCount> raw_max_{};
  std::atomic<std::size_t> axis_count_{0};
  std::uint32_t feedback_settle_cycles_{0};
  std::atomic<bool> feedback_ready_{false};
  std::atomic<bool> controller_enabled_{false};
  std::atomic<bool> target_valid_{false};
  std::atomic<bool> estop_latched_{false};
  std::atomic<int> estop_reason_code_{0};
  std::atomic<std::uint64_t> execution_generation_{0};
  std::mutex estop_reason_mutex_;
  std::string estop_reason_;

  std::vector<std::string> joint_names_;
  std::vector<std::int64_t> zero_offsets_;
  std::vector<std::int64_t> directions_;
  std::vector<double> counts_per_rev_;
  std::vector<double> gear_ratios_;
  std::vector<bool> limit_enabled_;
  std::vector<std::int64_t> raw_limit_a_;
  std::vector<std::int64_t> raw_limit_b_;
  std::unordered_map<std::string, std::size_t> name_to_index_;
  std::atomic<std::int64_t> max_step_per_cycle_{100};
  std::int64_t nominal_step_per_cycle_{100};
  std::int64_t following_error_counts_;
  std::int64_t goal_tolerance_counts_;
  double goal_timeout_sec_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr raw_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_service_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  qiuniu_init();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Master2MoveItBridge>());
  rclcpp::shutdown();
  return 0;
}
