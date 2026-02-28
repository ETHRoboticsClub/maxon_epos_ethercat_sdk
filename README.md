# Maxon EtherCAT SDK

## Overview

This is a C++ library providing a high-level interface for controlling [Maxon](https://www.maxonmc.com/) motor drivers of the [EPOS line](https://www.maxongroup.com/maxon/view/content/epos-detailsite) over EtherCAT (using the [CANopen over EtherCAT CoE](https://www.ethercat.org/en/technology.html#1.9.1) protocol). It is modified by Linghao Zhang based on [elmo_ethercat_sdk](https://github.com/leggedrobotics/elmo_ethercat_sdk) by Jonas Junger.

The lower level EtherCAT communication is handled by the [soem_interface](https://github.com/leggedrobotics/soem_interface) library.

The `maxon_epos_ethercat_sdk` is developed on Ubuntu 20.04 LTS with [ROS Noetic](https://wiki.ros.org/noetic).

The source code is released under the BSD-3-Clause license.
A copy of the license is available in the [LICENSE](LICENSE) file.

**Authors:** Linghao Zhang, Jonas Junger, Lennart Nachtigall

**Maintainer:** Linghao Zhang, lingzhang@ethz.ch

**Contributors:** Fabio Dubois, Markus Staeuble, Martin Wermelinger

## Installation

### Dependencies

#### Catkin Packages

|        Repo         |                          url                          |   License    |               Content               |
| :-----------------: | :---------------------------------------------------: | :----------: | :---------------------------------: |
|   soem_interface    | https://github.com/leggedrobotics/soem_interface.git  |    GPLv3     | Low-level EtherCAT functionalities  |
| ethercat_sdk_master | https://github.com/leggedrobotics/ethercat_sdk_master | BSD 3-Clause | High-level EtherCAT functionalities |
|   message_logger    | https://github.com/leggedrobotics/message_logger.git  | BSD 3-Clause |         simple log streams          |

#### System Dependencies (tested on Ubuntu 20.04 LTS)

- [ROS Noetic](https://wiki.ros.org/noetic)
- catkin
- yaml-cpp

> Likely to work with [ROS Meolodic](https://wiki.ros.org/melodic) and Ubuntu 18.04 LTS

### Building from Source

To build the library from source, clone the latest version from this repository and from the dependencies into your catkin workspace and compile the package using

```bash
cd catkin_workspace/src
git clone https://github.com/leggedrobotics/soem_interface.git
git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
git clone https://github.com/leggedrobotics/message_logger.git
git clone https://github.com/leggedrobotics/maxon_epos_ethercat_sdk.git
cd ../
catkin build maxon_epos_ethercat_sdk
```

## Example

See [ethercat_device_configurator](https://github.com/leggedrobotics/ethercat_device_conROS 2 Maxon EtherCAT bring-up node (10–20 Maxons)
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <csignal>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>

// ---- Adjust these includes/namespaces to your ROS2 EtherCAT stack ----
// If your ROS2 package name differs, update this include accordingly.
#include <ethercat_ros_configurator/EthercatDeviceConfigurator.hpp>

// Maxon slave type (adjust include if your SDK path differs)
#include <maxon_epos_ethercat_sdk/Maxon.hpp>

class MaxonEthercatBringupNode : public rclcpp::Node {
public:
  explicit MaxonEthercatBringupNode(const std::string& setup_yaml_path)
  : rclcpp::Node("maxon_ethercat_bringup"),
    setup_yaml_path_(setup_yaml_path)
  {
    // Parameters (ROS2 style)
    nthreads_ = this->declare_parameter<int>("nthreads", 4);
    expected_min_ = this->declare_parameter<int>("expected_min_maxons", 10);
    expected_max_ = this->declare_parameter<int>("expected_max_maxons", 20);

    // You can choose your RT prio; keep it moderate to avoid starving kernel processes.
    rt_priority_ = this->declare_parameter<int>("rt_priority", 48);

    RCLCPP_INFO(this->get_logger(), "Bringup node created. setup.yaml: %s", setup_yaml_path_.c_str());
  }

  bool init()
  {
    // Create configurator from YAML (adjust namespace/class to your ROS2 version)
    configurator_ = std::make_shared<EthercatRos::EthercatDeviceConfigurator>(setup_yaml_path_.c_str());

    // Start masters (SDO config etc.)
    for (auto& master : configurator_->getMasters()) {
      if (!master->startup()) {
        RCLCPP_ERROR(this->get_logger(), "Master startup failed.");
        return false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "All masters started (SAFE_OP/ready for PDO).");

    // Extract Maxon slaves ONLY (if your API differs, see note below)
    // If your configurator doesn't provide getSlavesOfType, you can filter by name/type another way.
    maxons_ = configurator_->getSlavesOfType<maxon::Maxon>(
        EthercatRos::EthercatDeviceConfigurator::EthercatSlaveType::Maxon);

    RCLCPP_INFO(this->get_logger(), "Found %zu Maxon actuators in setup.yaml.", maxons_.size());

    if (static_cast<int>(maxons_.size()) < expected_min_ || static_cast<int>(maxons_.size()) > expected_max_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Expected %d–%d Maxons, but found %zu. Check setup.yaml.",
                   expected_min_, expected_max_, maxons_.size());
      return false;
    }

    // Start each slave's internal worker thread (same style as your ROS1 code)
    for (const auto& slave : configurator_->getSlaves()) {
      slave->startWorkerThread();
    }

    // Start EtherCAT PDO update loop thread
    worker_thread_ = std::thread([this]() { this->pdoWorkerLoop(); });

    // Optional: wait a moment for the PDO loop to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Bring Maxons to OperationEnabled (blocking)
    for (auto& m : maxons_) {
      m->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, true);
    }
    RCLCPP_INFO(this->get_logger(), "Requested OperationEnabled for all Maxons.");

    // Optional: a timer just to show node is alive
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        size_t op = 0;
        for (auto& m : maxons_) {
          if (m->getReading().getDriveState() == maxon::DriveState::OperationEnabled) op++;
        }
        RCLCPP_INFO(this->get_logger(), "Status: OP enabled %zu/%zu", op, maxons_.size());
      }
    );

    return true;
  }

  void shutdownBringup()
  {
    // Make this idempotent
    bool expected = false;
    if (!shutdown_called_.compare_exchange_strong(expected, true)) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Shutdown started.");

    // 1) PreShutdown while PDO loop still running
    if (configurator_) {
      for (const auto& master : configurator_->getMasters()) {
        master->preShutdown();
      }
      RCLCPP_INFO(this->get_logger(), "preShutdown() called on masters.");
    }

    // 2) Stop PDO loop and join worker thread
    abrt_.store(true);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "PDO worker thread joined.");

    // 3) Stop each slave worker thread cleanly
    if (configurator_) {
      for (const auto& slave : configurator_->getSlaves()) {
        slave->abort();
        slave->joinWorkerThread();
      }
      RCLCPP_INFO(this->get_logger(), "Slave worker threads stopped.");
    }

    // 4) Shutdown masters (no EtherCAT comm possible afterwards)
    if (configurator_) {
      for (const auto& master : configurator_->getMasters()) {
        master->shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "Masters shutdown complete.");
    }

    RCLCPP_INFO(this->get_logger(), "Shutdown finished.");
  }

  ~MaxonEthercatBringupNode() override
  {
    // Ensure we cleanup even if main forgets
    shutdownBringup();
  }

private:
  void pdoWorkerLoop()
  {
    // Set RT priority for each master (like your ROS1 worker)
    bool rt_ok = true;
    for (const auto& master : configurator_->getMasters()) {
      rt_ok &= master->setRealtimePriority(rt_priority_);
    }

    if (rt_ok) {
      RCLCPP_INFO(this->get_logger(), "RT priority set to %d for all masters.", rt_priority_);
    } else {
      RCLCPP_WARN(this->get_logger(), "RT priority could not be set (check privileges).");
    }

    // PDO loop
    while (!abrt_.load()) {
      for (const auto& master : configurator_->getMasters()) {
        master->update(ecat_master::UpdateMode::StandaloneEnforceRate);
      }
    }
  }

private:
  std::string setup_yaml_path_;

  // ROS2 params
  int nthreads_{4};
  int expected_min_{10};
  int expected_max_{20};
  int rt_priority_{48};

  // EtherCAT + slaves
  EthercatRos::EthercatDeviceConfigurator::SharedPtr configurator_;
  std::vector<std::shared_ptr<maxon::Maxon>> maxons_;

  // Threads + shutdown flags
  std::thread worker_thread_;
  std::atomic<bool> abrt_{false};
  std::atomic<bool> shutdown_called_{false};

  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: maxon_ethercat_bringup path/to/setup.yaml\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const std::string setup_path = argv[1];
  auto node = std::make_shared<MaxonEthercatBringupNode>(setup_path);

  if (!node->init()) {
    std::cerr << "Bringup init failed.\n";
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Use MultiThreadedExecutor similar to ROS1 AsyncSpinner(nthreads)
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), static_cast<size_t>(node->get_parameter("nthreads").as_int())
  );
  exec.add_node(node);

  // Spin until Ctrl+C; ROS2 will call rclcpp::shutdown()
  exec.spin();

  // After spin ends, do clean EtherCAT shutdown
  node->shutdownBringup();

  rclcpp::shutdown();
  return 0;
}
Two important notes (so it works in your ROS2 stack)
1) Your configurator API might differ in ROS2
In ROS1 you used:
EthercatRos::EthercatDeviceConfigurator
getMasters(), getSlaves(), slave->startWorkerThread(), etc.
If your ROS2 version has different namespaces/types (for example EthercatDeviceConfigurator without EthercatRos::), you only need to change:
the include
the configurator type name
the “get Maxons” call (if getSlavesOfType is named differently)
Everything else (threading + shutdown pattern) stays the same.
2) Don’t do heavy work inside a POSIX signal handler
In ROS2, Ctrl+C triggers rclcpp::shutdown() and ends spin(). That’s why this design shuts EtherCAT down after spin() returns (safe and deterministic).
If you paste the ROS2 EthercatDeviceConfigurator header path + namespace you actually have (just the first few lines from your ROS2 package), I’ll adjust the includes/types so this compiles exactly in your workspace.figurator) for an minimal working example.

## Usage

See [Usage.md](doc/Usage.md).

## Contributing

See [Contributing.md](Contributing.md)
