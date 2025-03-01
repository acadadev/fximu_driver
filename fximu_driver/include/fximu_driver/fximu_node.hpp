#ifndef FXIMU_DRIVER__FXIMU_NODE_HPP_
#define FXIMU_DRIVER__FXIMU_NODE_HPP_

#include "fximu_driver/fximu_driver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <ctime>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>


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
        ~FximuNode();

        LNI::CallbackReturn on_configure(const lc::State & state) override;
        LNI::CallbackReturn on_activate(const lc::State & state) override;
        LNI::CallbackReturn on_deactivate(const lc::State & state) override;
        LNI::CallbackReturn on_cleanup(const lc::State & state) override;
        LNI::CallbackReturn on_shutdown(const lc::State & state) override;
        //void subscriber_callback(const UInt8MultiArray::SharedPtr msg);
        void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);

      private:

        std::string imu_frame_id;
        std::string mag_frame_id;

        std::unique_ptr<IoContext> m_owned_ctx{};
        std::string m_device_name{};
        std::unique_ptr<SerialPortConfig> m_device_config;
        std::unique_ptr<FximuDriver> m_serial_driver;

        lc::LifecyclePublisher<Imu>::SharedPtr imu_publisher;    
        lc::LifecyclePublisher<MagneticField>::SharedPtr mag_publisher;     

        void get_serial_parameters(void);
        void get_device_parameters(void);
        void send_parameters(void);
        bool handle_sys_status(uint8_t current_status);

        void send_sync_packet(bool immediate);

        double avg_rtc;
        double avg_nanos;

        bool timer_second_sent = false;
        uint32_t timer_prev_seconds = 0;

        uint8_t sys_status;
        uint8_t prev_sys_status;

        rclcpp::TimerBase::SharedPtr timer_sync;
        //rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;       

    };
  }
}

#endif