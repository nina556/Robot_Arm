#include <ecat/task.hpp>
#include <qiuniu/init.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/mman.h>

namespace {

// 固定控制 EtherCAT Master 2，只测试扫描到的第 0 个 CiA402 电机。
constexpr int kMasterId = 2;
constexpr std::int64_t kCycleTimeNs = 1000000;
constexpr double kPi = 3.14159265358979323846;

// 当前编码器参数：单圈 2^23 计数、无额外减速比、正方向。
constexpr double kCountsPerRev = 8388608.0;
constexpr double kGearRatio = 1.0;
constexpr int kDirection = 1;

// 每个 1 ms 周期最多改变 100 个计数，避免目标位置瞬间跳变。
constexpr std::int32_t kMaxStepPerCycle = 100;
constexpr const char *kDefaultEni =
    "/home/niic/Light_test/src/light_test/config/NIIC_ENI_M2.xml";
constexpr const char *kZeroFile = "/home/niic/Light_test/zero_count.txt";

struct Axis {
  // RxPDO：主站写入驱动器。
  std::uint16_t slave_pos = 0;
  volatile std::uint16_t *control_word = nullptr;
  volatile std::int8_t *mode = nullptr;
  volatile std::int32_t *target_position = nullptr;

  // TxPDO：驱动器反馈给主站。
  volatile const std::uint16_t *status_word = nullptr;
  volatile const std::int8_t *mode_display = nullptr;
  volatile const std::int32_t *actual_position = nullptr;
  volatile const std::uint16_t *error_code = nullptr;
};

// ROS 主线程与 EtherCAT 实时周期之间只通过原子变量交换命令和状态。
std::atomic<bool> g_running{true};
std::atomic<bool> g_axis_ready{false};
std::atomic<bool> g_enable_requested{false};
std::atomic<bool> g_operation_enabled{false};
std::atomic<bool> g_target_valid{false};
std::atomic<std::int32_t> g_actual{0};
std::atomic<std::int32_t> g_target{0};
std::atomic<std::int32_t> g_commanded{0};
std::atomic<std::int32_t> g_zero{0};
std::atomic<bool> g_zero_valid{false};
std::atomic<std::uint16_t> g_status{0};
std::atomic<std::uint16_t> g_error{0};
std::atomic<std::int8_t> g_mode_display{0};
ecat::task *g_task = nullptr;

void stop_handler(int) {
  g_running.store(false);
  g_enable_requested.store(false);
  if (g_task) {
    g_task->break_();
  }
}

bool load_zero() {
  std::ifstream input(kZeroFile);
  std::int32_t value = 0;
  if (!(input >> value)) {
    return false;
  }
  g_zero.store(value);
  g_zero_valid.store(true);
  return true;
}

bool save_zero(std::int32_t value) {
  std::ofstream output(kZeroFile, std::ios::trunc);
  if (!output) {
    return false;
  }
  output << value << '\n';
  return static_cast<bool>(output);
}

std::int32_t rad_to_count(double rad) {
  // 软件零点对应 0 rad，方向参数同时作用于正反向换算。
  const double raw = static_cast<double>(g_zero.load()) +
                     kDirection * rad * kCountsPerRev * kGearRatio /
                         (2.0 * kPi);
  const double clipped =
      std::clamp(raw,
                 static_cast<double>(std::numeric_limits<std::int32_t>::min()),
                 static_cast<double>(std::numeric_limits<std::int32_t>::max()));
  return static_cast<std::int32_t>(std::llround(clipped));
}

double count_to_rad(std::int32_t count) {
  return kDirection * static_cast<double>(count - g_zero.load()) *
         (2.0 * kPi) / (kCountsPerRev * kGearRatio);
}

bool operation_enabled(std::uint16_t status) {
  // CiA402 状态字掩码：0x0027 表示 Operation Enabled。
  return (status & 0x006f) == 0x0027;
}

void update_control(Axis &axis) {
  if (!axis.control_word || !axis.status_word || !axis.actual_position ||
      !axis.target_position) {
    return;
  }

  const std::uint16_t status = *axis.status_word;
  const std::int32_t actual = *axis.actual_position;
  g_status.store(status);
  g_actual.store(actual);
  g_error.store(axis.error_code ? *axis.error_code : 0);
  g_mode_display.store(axis.mode_display ? *axis.mode_display : 0);

  const bool enabled = operation_enabled(status);
  g_operation_enabled.store(enabled);

  // 未请求使能时持续把目标位置对齐实际位置，防止下次使能时跳动。
  if (!g_enable_requested.load()) {
    *axis.control_word = enabled ? 0x0007 : 0x0006;
    g_target.store(actual);
    g_commanded.store(actual);
    *axis.target_position = actual;
    return;
  }

  // CiA402 使能顺序：故障复位 -> Shutdown -> Switch On -> Enable Operation。
  if (status & 0x0008) {
    *axis.control_word = 0x0080;
    return;
  }
  if ((status & 0x004f) == 0x0040) {
    *axis.control_word = 0x0006;
    return;
  }
  if ((status & 0x006f) == 0x0021) {
    *axis.control_word = 0x0007;
    return;
  }
  if ((status & 0x006f) == 0x0023) {
    if (axis.mode) {
      *axis.mode = 8;
    }
    g_commanded.store(actual);
    *axis.target_position = actual;
    *axis.control_word = 0x000f;
    return;
  }
  if (!enabled) {
    *axis.control_word = 0x0006;
    return;
  }

  *axis.control_word = 0x000f;
  if (!g_target_valid.load()) {
    g_target.store(actual);
    g_commanded.store(actual);
    *axis.target_position = actual;
    return;
  }

  const std::int32_t desired = g_target.load();
  const std::int32_t current = g_commanded.load();
  const std::int64_t difference =
      static_cast<std::int64_t>(desired) - static_cast<std::int64_t>(current);
  const std::int64_t step =
      std::clamp<std::int64_t>(difference, -kMaxStepPerCycle,
                               kMaxStepPerCycle);
  // 角度命令和原始计数命令最终都在这里缓慢逼近目标。
  const auto next = static_cast<std::int32_t>(
      static_cast<std::int64_t>(current) + step);
  g_commanded.store(next);
  *axis.target_position = next;
}

void print_status() {
  const auto actual = g_actual.load();
  std::cout << "PDO就绪=" << g_axis_ready.load()
            << " 请求使能=" << g_enable_requested.load()
            << " 已使能=" << g_operation_enabled.load()
            << " 状态字=0x" << std::hex << g_status.load()
            << " 错误码=0x" << g_error.load() << std::dec
            << " 模式=" << static_cast<int>(g_mode_display.load())
            << " 实际计数=" << actual
            << " 目标计数=" << g_target.load()
            << " 零点计数=";
  if (g_zero_valid.load()) {
    std::cout << g_zero.load()
              << " 实际角度(rad)=" << count_to_rad(actual);
  } else {
    std::cout << "未设置";
  }
  std::cout << '\n';
}

void print_help() {
  std::cout
      << "可用命令：\n"
      << "  status          查看驱动器和编码器状态\n"
      << "  enable          使能第 0 轴并保持当前位置\n"
      << "  disable         取消使能第 0 轴\n"
      << "  zero            将当前编码器值保存为软件零点\n"
      << "  rad <数值>      发送关节角度，单位为弧度\n"
      << "  count <数值>    发送绝对原始编码器目标值\n"
      << "  quit            取消使能并退出\n";
}

}  // namespace

int main(int argc, char **argv) {
  const std::string eni = argc > 1 ? argv[1] : kDefaultEni;

  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  qiuniu_init();
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::perror("mlockall");
  }

  load_zero();
  Axis axis;
  ecat::task task(kMasterId);
  g_task = &task;

  task.priority(80);
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(2, &cpus);
  task.cpu_affinity(&cpus, sizeof(cpus));
  task.record(false);
  std::int64_t cycle_time_ns = kCycleTimeNs;
  task.load_eni(eni, cycle_time_ns);

  task.set_config_callback([&] {
    // 按从站顺序查找，只注册第一个 CiA402 电机。
    bool found = false;
    for (std::uint16_t slave = 0; slave < task.slave_count(); ++slave) {
      if (task.profile_no(slave) != 402) {
        continue;
      }
      axis.slave_pos = slave;
      const bool ok =
          task.try_register_pdo_entry(axis.control_word, slave, {0x6040, 0}) &&
          task.try_register_pdo_entry(axis.status_word, slave, {0x6041, 0}) &&
          task.try_register_pdo_entry(axis.mode, slave, {0x6060, 0}) &&
          task.try_register_pdo_entry(axis.target_position, slave, {0x607a, 0}) &&
          task.try_register_pdo_entry(axis.actual_position, slave, {0x6064, 0});
      task.try_register_pdo_entry(axis.mode_display, slave, {0x6061, 0});
      task.try_register_pdo_entry(axis.error_code, slave, {0x603f, 0});
      g_axis_ready.store(ok);
      found = true;
      std::cout << "第 0 轴使用 slave_pos=" << slave
                << "，PDO就绪=" << ok << '\n';
      break;
    }
    if (!found) {
      std::cerr << "Master 2 上未找到 CiA402 电机\n";
    }
  });

  task.set_cycle_callback([&] {
    if (g_axis_ready.load()) {
      update_control(axis);
    }
  });

  task.start();
  print_help();

  std::string line;
  while (g_running.load() && std::cout << "> " && std::getline(std::cin, line)) {
    std::istringstream input(line);
    std::string command;
    input >> command;

    if (command == "status") {
      print_status();
    } else if (command == "enable") {
      g_target_valid.store(false);
      g_enable_requested.store(true);
      std::cout << "已请求使能，将保持当前位置\n";
    } else if (command == "disable") {
      g_enable_requested.store(false);
      g_target_valid.store(false);
      std::cout << "已请求取消使能\n";
    } else if (command == "zero") {
      const auto zero = g_actual.load();
      if (save_zero(zero)) {
        g_zero.store(zero);
        g_zero_valid.store(true);
        std::cout << "已保存软件零点，编码器值=" << zero << '\n';
      } else {
        std::cerr << "无法写入零点文件：" << kZeroFile << '\n';
      }
    } else if (command == "rad") {
      double rad = 0.0;
      if (!(input >> rad) || !std::isfinite(rad)) {
        std::cerr << "用法：rad <有限的弧度值>\n";
      } else if (!g_zero_valid.load()) {
        std::cerr << "请先执行 zero 设置软件零点\n";
      } else if (!g_operation_enabled.load()) {
        std::cerr << "请先使能电机\n";
      } else {
        const auto count = rad_to_count(rad);
        g_target.store(count);
        g_target_valid.store(true);
        std::cout << "目标角度(rad)=" << rad
                  << "，目标计数=" << count << '\n';
      }
    } else if (command == "count") {
      std::int64_t value = 0;
      if (!(input >> value) ||
          value < std::numeric_limits<std::int32_t>::min() ||
          value > std::numeric_limits<std::int32_t>::max()) {
        std::cerr << "用法：count <int32范围内的绝对编码器值>\n";
      } else if (!g_operation_enabled.load()) {
        std::cerr << "请先使能电机\n";
      } else {
        g_target.store(static_cast<std::int32_t>(value));
        g_target_valid.store(true);
        std::cout << "目标计数=" << value << '\n';
      }
    } else if (command == "help") {
      print_help();
    } else if (command == "quit" || command == "exit") {
      break;
    } else if (!command.empty()) {
      std::cerr << "未知命令，请输入 help 查看帮助\n";
    }
  }

  g_enable_requested.store(false);
  g_target_valid.store(false);
  ecat::RTTools::usleep(100000);
  task.break_();
  task.wait();
  task.release();
  g_task = nullptr;
  return 0;
}
