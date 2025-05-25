#ifndef FXIMU_DRIVER_FXIMU_NODE_HPP_
#define FXIMU_DRIVER_FXIMU_NODE_HPP_

#define STATUS_OK 0x00
#define STATUS_SEQ_REPEAT 0x01
#define STATUS_SEQ_JUMP 0x02
#define STATUS_CHECKSUM_ERROR 0x03
#define STATUS_SKIP_SECOND 0x04
#define STATUS_SKIP_NANOS 0x5
#define STATUS_OUT_SYNC 0x06

#include "fximu_driver/fximu_driver.hpp"
#include "fximu_driver/adaptive_filter.h"

#include <memory>
#include <string>
#include <vector>
#include <ctime>
#include <iostream>
#include <chrono>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

#define USE_HIGH_PRECISION_CLOCK 1

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;

namespace drivers
{
  namespace fximu_driver
  {
    class FximuNode final : public lc::LifecycleNode {

      public:

        explicit FximuNode(const rclcpp::NodeOptions & options);

        FximuNode(const rclcpp::NodeOptions & options, const IoContext & ctx);
        ~FximuNode() override;

        LNI::CallbackReturn on_configure(const lc::State & state) override;
        LNI::CallbackReturn on_activate(const lc::State & state) override;
        LNI::CallbackReturn on_deactivate(const lc::State & state) override;
        LNI::CallbackReturn on_cleanup(const lc::State & state) override;
        LNI::CallbackReturn on_shutdown(const lc::State & state) override;

        // void subscriber_callback(const UInt8MultiArray::SharedPtr msg);

        void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);

        auto get_time(){
          #ifdef USE_MONOTONIC_CLOCK
            return std::chrono::steady_clock::now();
          #elif USE_HIGH_PRECISION_CLOCK
            return std::chrono::high_resolution_clock::now();
          #else
            return std::chrono::system_clock::now();
          #endif
        }

      private:

        std::string imu_frame_id;
        std::string mag_frame_id;

        std::unique_ptr<IoContext> m_owned_ctx{};
        std::string m_device_name{};
        std::unique_ptr<SerialPortConfig> m_device_config;
        std::unique_ptr<FximuDriver> m_serial_driver;

        lc::LifecyclePublisher<Imu>::SharedPtr imu_publisher;    
        lc::LifecyclePublisher<MagneticField>::SharedPtr mag_publisher;     

        void reset_driver();
        void get_serial_parameters();
        void declare_parameters();
        void send_parameters();
        bool handle_sys_status(uint8_t current_status);
        void init_sync();
        void mcu_sync(bool send_async);

        int8_t read_state = -1;               // serial read state

        uint8_t sys_status = 0;               // current system status
        uint8_t prev_sys_status = 0;          // previous system status

        uint8_t packet_seq = 0;               // current packet sequence
        uint8_t prev_packet_seq = 0;          // previous packet sequence

        uint32_t packet_count = 0;            // packet counter

        bool enable_magneto = false;          // enable magnetometer
        bool publish_magneto = false;         // publish magnetometer data

        AdaptiveFilter* filter_timing;
        AdaptiveFilter* filter_rtc_offset;

    };
  }
}

#endif