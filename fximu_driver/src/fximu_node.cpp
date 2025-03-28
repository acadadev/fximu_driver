#include "fximu_driver/util.h"
#include "fximu_driver/parameters.h"
#include "fximu_driver/fximu_node.hpp"

// TODO: 
#include <stdlib.h>

#define DECPOINTS 5
// TODO: magac above

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;


// TODO: right now sync packet is being sent each N seconds with a timer thread.
// TODO: instead, after reception of imu packet, we can piggyback a sync packet, 
// TODO: guaranteeing no collision. 
// TODO: we can use a semaphore inn hardware part.

// TODO: ENU or NED? option to select/ right now its wrong.
// TODO: also remove imu is parket drong gravity.

namespace drivers
{
  namespace fximu_driver
  {

    FximuNode::FximuNode(const rclcpp::NodeOptions & options)
    : lc::LifecycleNode("fximu_node", options), m_owned_ctx{new IoContext(2)}, m_serial_driver{new FximuDriver(*m_owned_ctx)}
    {
      get_serial_parameters();
    }

    FximuNode::FximuNode(const rclcpp::NodeOptions & options, const IoContext & ctx)
    : lc::LifecycleNode("fximu_node", options), m_serial_driver{new FximuDriver(ctx)}
    {
      get_serial_parameters();   
    }

    FximuNode::~FximuNode() {
      if (m_owned_ctx) {
        m_owned_ctx->waitForExit();
      }
    }

    void FximuNode::send_sync_packet(bool init_second) {

      std::vector<uint8_t> sync_packet;
      
      auto time = std::chrono::high_resolution_clock::now();
      uint32_t host_seconds;
      uint32_t host_nanos;

      if(init_second) {

        uint32_t previous_nanos = 0;
        while(true) {
            time = std::chrono::high_resolution_clock::now();
            host_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
            host_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch() - std::chrono::seconds(host_seconds)).count();
            if(host_nanos < previous_nanos) { break; }
            previous_nanos = host_nanos;
        }   

        // add propagation delay
        time += std::chrono::microseconds(0);  

        host_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();

        u32_to_ui8 u; 
        u.u32 = host_seconds;
        sync_packet.push_back('>');
        sync_packet.push_back(u.ui8[0]);
        sync_packet.push_back(u.ui8[1]);
        sync_packet.push_back(u.ui8[2]);
        sync_packet.push_back(u.ui8[3]);  
        sync_packet.push_back(PACKET_POSTFIX);      


      } else {

        // add propagation delay
        time += std::chrono::microseconds(0);
        
        host_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        host_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch() - std::chrono::seconds(host_seconds)).count();

        u32_to_ui8 u;
        u.u32 = host_nanos;

        sync_packet.push_back('<');
        sync_packet.push_back(u.ui8[0]);
        sync_packet.push_back(u.ui8[1]);
        sync_packet.push_back(u.ui8[2]);
        sync_packet.push_back((((uint8_t) host_seconds % 4) << 6) | u.ui8[3]);   
        sync_packet.push_back(PACKET_POSTFIX);     

      }

      // send packet
      m_serial_driver->port()->send(sync_packet); 
    }

    LNI::CallbackReturn FximuNode::on_configure(const lc::State & state) {

      (void)state;

      // create serial connection
      try {
        m_serial_driver->init_port(m_device_name, *m_device_config);
        if (!m_serial_driver->port()->is_open()) {
          m_serial_driver->port()->open();
        }
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "error creating serial port: %s - %s", m_device_name.c_str(), ex.what());
        return LNI::CallbackReturn::FAILURE;
      }   

      // get device parameters with default from yaml file
      get_device_parameters(); 

      // send device parameters
      send_parameters();

      // TODO: request sys status, see if need a reset

      // send sync packet, block until beginning of second
      send_sync_packet(true);   

      // create publisher
      imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS{100});
      mag_publisher = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", rclcpp::QoS{100});

      // create subscriber
      // auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
      // auto callback = std::bind(&FximuNode::subscriber_callback, this, std::placeholders::_1);
      // m_subscriber = this->create_subscription<UInt8MultiArray>("serial_write", qos, callback);

      // start receiving data
      m_serial_driver->port()->async_receive(
        std::bind(&FximuNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
      );   

      timer_sync = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [&]() {
            const auto time = std::chrono::high_resolution_clock::now();
            // const uint32_t timer_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
            send_sync_packet(false);

        }, 0);  
        
      timer_sync->reset();

      RCLCPP_INFO(get_logger(), "FXIMU successfully configured.");

      return LNI::CallbackReturn::SUCCESS;
    }   

    LNI::CallbackReturn FximuNode::on_activate(const lc::State & state) {
      (void)state;
      imu_publisher->on_activate();
      mag_publisher->on_activate();
      timer_sync->reset();
      RCLCPP_INFO(get_logger(), "FXIMU activated.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_deactivate(const lc::State & state) {
      (void)state;
      imu_publisher->on_deactivate();
      mag_publisher->on_deactivate();
      timer_sync->cancel();
      RCLCPP_INFO(get_logger(), "FXIMU deactivated.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_cleanup(const lc::State & state) {
      (void)state;
      m_serial_driver->port()->close();
      imu_publisher.reset();
      mag_publisher.reset();
      timer_sync->cancel();
      // m_subscriber.reset();
      RCLCPP_INFO(get_logger(), "FXIMU cleaned up.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_shutdown(const lc::State & state) {
      (void)state;
      RCLCPP_INFO(get_logger(), "FXIMU shutting down.");
      return LNI::CallbackReturn::SUCCESS;
    }

    void FximuNode::get_device_parameters(void) {

      try {
        
        // uint8 section
        this->declare_parameter<uint8_t>("gyroODR", 6);
        this->declare_parameter<uint8_t>("gyroFSR", 0);
        this->declare_parameter<uint8_t>("accelODR", 6);
        this->declare_parameter<uint8_t>("accelFSR", 0);
        this->declare_parameter<uint8_t>("notchFilterBW", 2);
        this->declare_parameter<uint8_t>("notchFilterDIR", 7);
        this->declare_parameter<uint8_t>("antiAliasFilterBW", 21);       
        this->declare_parameter<uint8_t>("outputDiv", 4);
        this->declare_parameter<uint8_t>("gyroUIFilterOrder", 2);
        this->declare_parameter<uint8_t>("gyroUIFilterIndex", 1);
        this->declare_parameter<uint8_t>("accelUIFilterOrder", 2);
        this->declare_parameter<uint8_t>("accelUIFilterIndex", 1);
        this->declare_parameter<uint8_t>("steadyLimit", 3);

        // BMM350_ODR_400HZ                            UINT8_C(0x2)
        // BMM350_ODR_200HZ                            UINT8_C(0x3)
        // BMM350_ODR_100HZ                            UINT8_C(0x4)
        // BMM350_ODR_50HZ                             UINT8_C(0x5)
        // BMM350_ODR_25HZ                             UINT8_C(0x6)
        // BMM350_ODR_12_5HZ                           UINT8_C(0x7)
        // BMM350_ODR_6_25HZ                           UINT8_C(0x8)
        // BMM350_ODR_3_125HZ                          UINT8_C(0x9)
        // BMM350_ODR_1_5625HZ                         UINT8_C(0xA)

        // default is 400 HZ
        this->declare_parameter<uint8_t>("magOdr", 0x2); 

        // BMM350_AVG_NO_AVG                           UINT8_C(0x0)
        // BMM350_AVG_2                                UINT8_C(0x1)
        // BMM350_AVG_4                                UINT8_C(0x2)
        // BMM350_AVG_8                                UINT8_C(0x3)    

        // default is no averating
        this->declare_parameter<uint8_t>("magAvg", 0x0); 

        // uint16 section
        this->declare_parameter<uint16_t>("timer0Sec", 4);
        this->declare_parameter<uint16_t>("timer1Sec", 5);

        // int16 section
        this->declare_parameter<int16_t>("offsetGyroX", 0);
        this->declare_parameter<int16_t>("offsetGyroY", 0);
        this->declare_parameter<int16_t>("offsetGyroZ", 0);
        this->declare_parameter<int16_t>("offsetAccelX", 0);
        this->declare_parameter<int16_t>("offsetAccelY", 0);
        this->declare_parameter<int16_t>("offsetAccelZ", 0);        

        // float section
        this->declare_parameter<float>("kDeltaAccelerationThreshold", 10.0);
        this->declare_parameter<float>("kDeltaAngularThreshold", 4.0);
        this->declare_parameter<float>("kAngularThreshold", 1.0);

        this->declare_parameter<float>("notchFilterFHZ", 1.449);

        this->declare_parameter<float>("filterGainAccel", 0.02);  // filter gain for acceletometer and gyro
        this->declare_parameter<float>("filterGainMag", 0.001);   // filter gain for magnetometer data

        this->declare_parameter<float>("biasAlpha", 0.01);        // for auto bias calculation filtering
        this->declare_parameter<float>("gainAlpha", 0.95);        // adaptive gain calculation
        this->declare_parameter<float>("stableAlpha", 0.05);      // initial steady condition
        this->declare_parameter<float>("magAlpha", 0.02);         // averaged mag data for display

        this->declare_parameter<float>("magBiasX", 0.0);
        this->declare_parameter<float>("magBiasY", 0.0);
        this->declare_parameter<float>("magBiasZ", 0.0);

        this->declare_parameter<float>("magSoftA1", 0.0);
        this->declare_parameter<float>("magSoftA2", 0.0);
        this->declare_parameter<float>("magSoftA3", 0.0);
         
        this->declare_parameter<float>("magSoftB1", 0.0);
        this->declare_parameter<float>("magSoftB2", 0.0);
        this->declare_parameter<float>("magSoftB3", 0.0);

        this->declare_parameter<float>("magSoftC1", 0.0);
        this->declare_parameter<float>("magSoftC2", 0.0);
        this->declare_parameter<float>("magSoftC3", 0.0);            

        // boolean section
        this->declare_parameter<bool>("enableNotchFilter", true);
        this->declare_parameter<bool>("enableAAFilter", true);
        this->declare_parameter<bool>("enableUIFilter", true);

        this->declare_parameter<bool>("enableSerial", false);
        this->declare_parameter<bool>("enableInitialCalibration", false);
        this->declare_parameter<bool>("enableAdaptiveBias", true);
        this->declare_parameter<bool>("enableAdaptiveGain", true);

        this->declare_parameter<bool>("enableMagnetometer", true);
        this->declare_parameter<bool>("useMagnetometerData", false);
        this->declare_parameter<bool>("pubMagnetometerData", true);

        // string section, these do not go to device
        imu_frame_id = this->declare_parameter<std::string>("imu_frame_id", "imu_link");
        mag_frame_id = this->declare_parameter<std::string>("mag_frame_id", "mag_link");

      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "parameter exception %s", ex.what());
        throw ex;
      }
    }

    void FximuNode::get_serial_parameters() {

      uint32_t baud_rate{};
      auto fc = FlowControl::NONE;
      auto pt = Parity::NONE;
      auto sb = StopBits::ONE;

      try {
        m_device_name = declare_parameter<std::string>("device_name", "");
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "the device_name is invalid");
        throw ex;
      }

      try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "the baud_rate is invalid");
        throw ex;
      }

      try {

        const auto fc_string = declare_parameter<std::string>("flow_control", "");
        if (fc_string == "none") {
          fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
          fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
          fc = FlowControl::SOFTWARE;
        } else {
          throw std::invalid_argument{"flow_controlnone, software, or hardware]"};
        }

      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "the flow_control is invalid");
        throw ex;
      }

      try {

        const auto pt_string = declare_parameter<std::string>("parity", "");
        if (pt_string == "none") {
          pt = Parity::NONE;
        } else if (pt_string == "odd") {
          pt = Parity::ODD;
        } else if (pt_string == "even") {
          pt = Parity::EVEN;
        } else {
          throw std::invalid_argument{"parity[none, odd, or even]"};
        }
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "the parity is invalid");
        throw ex;
      }

      try {

        const auto sb_string = declare_parameter<std::string>("stop_bits", "");
        if (sb_string == "1" || sb_string == "1.0") {
          sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
          sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
          sb = StopBits::TWO;
        } else {
          throw std::invalid_argument{"stop_bits[1, 1.5, or 2]"};
        }
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "the stop_bits is invalid");
        throw ex;
      }

      m_device_config = std::make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
    }

    void FximuNode::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred) { 

      const auto received_time = std::chrono::high_resolution_clock::now();
      const uint32_t host_seconds = std::chrono::duration_cast<std::chrono::seconds>(received_time.time_since_epoch()).count();
      const uint32_t host_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(received_time.time_since_epoch() - std::chrono::seconds(host_seconds)).count();

      if(
        (bytes_transferred == 64) &
        (buffer[0] == PACKET_PREFIX) &                                  // prefix check
        (buffer[IMU_DATA_SIZE - 1] == PACKET_POSTFIX)                   // postfix check
      ) {   

        // get crc from received packet
        uint8_t crc8 = buffer[IMU_DATA_SIZE - 2];                        // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), IMU_DATA_SIZE - 3);


        if(crc8 != c_crc8) {

          RCLCPP_ERROR(this->get_logger(), "imu crc8:%d != c_crc8:%d", crc8, c_crc8);

        } else {  

          auto imu_data = sensor_msgs::msg::Imu(); 
          auto mag_data = sensor_msgs::msg::MagneticField();

          imu_data.orientation.w = R4(buffer, 1);                         // q0
          imu_data.orientation.x = R4(buffer, 1 + 4);                     // q1
          imu_data.orientation.y = R4(buffer, 1 + 8);                     // q2
          imu_data.orientation.z = R4(buffer, 1 + 12);                    // q3

          imu_data.linear_acceleration.x = R4(buffer, 17);
          imu_data.linear_acceleration.y = R4(buffer, 17 + 4);
          imu_data.linear_acceleration.z = R4(buffer, 17 + 8);

          imu_data.angular_velocity.x = R4(buffer, 29);
          imu_data.angular_velocity.y = R4(buffer, 29 + 4);
          imu_data.angular_velocity.z = R4(buffer, 29 + 8);

          mag_data.magnetic_field.x = R4(buffer, 41);
          mag_data.magnetic_field.y = R4(buffer, 41 + 4);
          mag_data.magnetic_field.z = R4(buffer, 41 + 8);

          float mag_vector = sqrt(
              (mag_data.magnetic_field.x * mag_data.magnetic_field.x) +
              (mag_data.magnetic_field.y * mag_data.magnetic_field.y) +
              (mag_data.magnetic_field.z * mag_data.magnetic_field.z)
          );

          /*
          RCLCPP_INFO(get_logger(), "%.6f uT",
            mag_vector
          );
          */


          // TIMING values coming from DEVICE

          // byte containing posix sec and rtc subseconds
          uint32_t device_posix_sub = U4(buffer, 53);

          // the lsb 17 bits are posix second of day
          uint32_t device_second_of_day = device_posix_sub & 0x0001FFFF;

          // the msb 15 bits are rtc ticks
          uint16_t device_rtc_ticks = (uint16_t) ((device_posix_sub >> 17) & 0x7FFF);

          // sys ticks, 0 to 80000000
          uint32_t device_sys_ticks = U4(buffer, 57);

          // calculate device nanos
          uint32_t device_sys_ticks_nanos = (uint32_t) (device_sys_ticks * 12.5);              

          // TIMING values coming from HOST

          // integerate device posix seconds
          uint32_t host_second_of_day = host_seconds % 86400;

           // calculate received ns in ticks
          uint32_t host_rtc_ticks = (uint32_t) (host_nanos / 30517.578125);  

          int32_t current_rtc_delta;
          int32_t current_delta_debug;

          int8_t second_difference = host_second_of_day - device_second_of_day;
          bool publish_packet = true;
          uint32_t stamp_seconds;

          if(second_difference == 0) {
              // @ same second, must normally produce negative due to delay.
              current_rtc_delta = device_rtc_ticks - host_rtc_ticks;
              current_delta_debug = device_sys_ticks_nanos - host_nanos;    
              stamp_seconds = host_seconds;    
          } else if((second_difference > 0) & (second_difference < 2)) {
              // rtc lagging seconds
              current_rtc_delta = -((32768 - device_rtc_ticks) + host_rtc_ticks);  
              current_delta_debug = -cyclic_distance_ns(host_nanos, device_sys_ticks_nanos);  
              stamp_seconds = host_seconds - 1;
          } else if((second_difference < 0) & (second_difference > -2)) {
              // rtc leading seconds
              current_rtc_delta = ((32768 - host_rtc_ticks) + device_rtc_ticks);
              current_delta_debug = cyclic_distance_ns(host_nanos, device_sys_ticks_nanos);
              stamp_seconds = host_seconds + 1;
          } else {
            publish_packet = false;
          }

          avg_rtc = (avg_rtc * 0.9) + (current_rtc_delta * 0.1);
          avg_nanos = (avg_nanos * 0.99) + ((current_delta_debug / 1000000000.0) * 0.01);

          // notice: we print SYS_REPORT here
          if(buffer[61]!=0) {
            RCLCPP_ERROR(this->get_logger(), "SYS_REPORT: %d", buffer[61]);
          }

          // dont publish if timestamp difference is greater than 2 seconds
          if(publish_packet) {

              // TODO: keep these data packets outside loop

              //imu_data.header.stamp = rclcpp::Clock().now();

              rclcpp::Time stamp(static_cast<uint64_t>((stamp_seconds * 1e9) + device_sys_ticks_nanos));

              // TODO: later broadcast tf from imu_link to mag_link

              //imu_data.header.stamp = rclcpp::Time(stamp_seconds) + rclcpp::Duration(0, device_sys_ticks_nanos);
              imu_data.header.stamp = stamp;
              imu_data.header.frame_id = imu_frame_id;

              mag_data.header.stamp = imu_data.header.stamp;
              mag_data.header.frame_id = mag_frame_id;

              imu_publisher->publish(imu_data);
              mag_publisher->publish(mag_data);

              /*
              // for calibration
              RCLCPP_INFO(get_logger(), "%.6f %.6f %.6f %.6f %.6f %.6f %.6f",
                imu_data.orientation.w,
                imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                mag_data.magnetic_field.x,
                mag_data.magnetic_field.y,
                mag_data.magnetic_field.z
              );
              */

          }

        }
      } else if(
        (bytes_transferred == 64) &
        (buffer[0] == DIAG_PREFIX) &                                // prefix check
        (buffer[IMU_DATA_SIZE - 1] == PACKET_POSTFIX)               // postfix check
      ) { 

        // get crc from received packet
        uint8_t crc8 = buffer[IMU_DATA_SIZE - 2];                 // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), IMU_DATA_SIZE - 3);

        if(crc8 != c_crc8) {

          RCLCPP_ERROR(this->get_logger(), "diag crc8:%d != c_crc8:%d", crc8, c_crc8);

        } else { 

          float ax_bias = R4(buffer, 1);                         // ax_bias
          float ay_bias = R4(buffer, 1 + 4);                     // ay_bias
          float az_bias = R4(buffer, 1 + 8);                     // az_bias
          float wx_bias = R4(buffer, 1 + 12);                    // wx_bias
          float wy_bias = R4(buffer, 1 + 16);                    // wy_bias
          float wz_bias = R4(buffer, 1 + 20);                    // wz_bias

          float mag_temp = R4(buffer, 25);
          
          uint32_t posix_time = U4(buffer, 30);

          uint8_t received_second = buffer[34];
          uint32_t received_nano = U4(buffer, 35);

          uint8_t mcu_second = buffer[39];
          uint32_t mcu_nano = U4(buffer, 40);

          int32_t MCU_PULL = I4(buffer, 44);
          int32_t MCU_OFFSET = I4(buffer, 48);

          int16_t RTC_PULL = I2(buffer, 52);
          int16_t RTC_OFFSET = I2(buffer, 54);
          int16_t DT_PERIOD_PULL = I2(buffer, 56);

          // notice: this is setting sys_status from diag packet
          sys_status = buffer[61];
          handle_sys_status(sys_status);

          RCLCPP_INFO(get_logger(), "wx: %.6f, wy: %.6f, wz: %.6f, temp: %.2fC",
            wx_bias,
            wy_bias,
            wz_bias,
            mag_temp
          );

          /*
          RCLCPP_INFO(get_logger(), "posix: %u, recs: %u, recns: %u, mcus: %u, mcuns: %u",
            posix_time,
            received_second,
            received_nano, 
            mcu_second,
            mcu_nano
          );       

          RCLCPP_INFO(get_logger(), "mcup: %d, mcuo: %d, rtcp: %d, rtco: %d, dtpp: %d",
            MCU_PULL,
            MCU_OFFSET, 
            RTC_PULL,
            RTC_OFFSET,
            DT_PERIOD_PULL 
          );

          RCLCPP_INFO(get_logger(), "sys_status: %d",
            sys_status
          );
          */



        }

      } else if(
        (bytes_transferred == 64) &
        (buffer[0] == CALIBRATION_PREFIX) &                         // prefix check
        (buffer[IMU_DATA_SIZE - 1] == PACKET_POSTFIX)               // postfix check
      ) { 

        // get crc from received packet
        uint8_t crc8 = buffer[IMU_DATA_SIZE - 2];                 // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), IMU_DATA_SIZE - 3);

        if(crc8 != c_crc8) {

          RCLCPP_ERROR(this->get_logger(), "calib crc8:%d != c_crc8:%d", crc8, c_crc8);

        } else { 

          float ax = R4(buffer, 1);                     // instant accel x, in newtons
          float ay = R4(buffer, 1 + 4);                 // instant accel y, in newtons
          float az = R4(buffer, 1 + 8);                 // instant accel z, in newtons

          float wx = R4(buffer, 13);                    // instant gyro x, in degrees
          float wy = R4(buffer, 13 + 4);                // instant gyro y, in degrees
          float wz = R4(buffer, 13 + 8);                // instant gyro z, in degrees

          float mx = R4(buffer, 25);                    // magnetic x
          float my = R4(buffer, 25 + 4);                // magnetic y
          float mz = R4(buffer, 25 + 8);                // magnetic z
          
          /*
          RCLCPP_INFO(get_logger(), "GYRO %f,%f,%f",
            wx,
            wy,
            wz
          );

          
          RCLCPP_INFO(get_logger(), "ACCEL %f,%f,%f",
            ax,
            ay,
            az
          );  
          */
          /*
          RCLCPP_INFO(get_logger(), "MAGF %f,%f,%f",
            mx,
            my,
            mz
          );         */   

        }

      } else if(
        (bytes_transferred == 64) &
        (buffer[0] == CALIBRATION_RAW_PREFIX) &                         // prefix check
        (buffer[IMU_DATA_SIZE - 1] == PACKET_POSTFIX)               // postfix check
      ) { 

        // get crc from received packet
        uint8_t crc8 = buffer[IMU_DATA_SIZE - 2];                 // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), IMU_DATA_SIZE - 3);

        if(crc8 != c_crc8) {

          RCLCPP_ERROR(this->get_logger(), "raw crc8:%d != c_crc8:%d", crc8, c_crc8);

        } else { 

          int16_t raw_ax = I2(buffer, 1);                         // raw ax
          int16_t raw_ay = I2(buffer, 1 + 2);                     // raw ay
          int16_t raw_az = I2(buffer, 1 + 4);                     // raw az
          int16_t raw_wx = I2(buffer, 1 + 6);                     // raw wx
          int16_t raw_wy = I2(buffer, 1 + 8);                     // raw wy
          int16_t raw_wz = I2(buffer, 1 + 10);                    // raw wz

          int32_t raw_mx = I4(buffer, 13); 
          int32_t raw_my = I4(buffer, 13 + 4);
          int32_t raw_mz = I4(buffer, 13 + 8);
          int32_t raw_mt = I4(buffer, 13 + 12);

          float f_mx = R4(buffer, 29);
          float f_my = R4(buffer, 29 + 4);
          float f_mz = R4(buffer, 29 + 8);
          float f_mt = R4(buffer, 29 + 12);

          RCLCPP_INFO(get_logger(), "MAGR %f,%f,%f",
            f_mx,
            f_my,
            f_mz
          );    

          RCLCPP_INFO(get_logger(), "MAGF %d,%d,%d",
            raw_mx,
            raw_my,
            raw_mz
          );
          
        }        

      }

    } // end receive callback
    
    void FximuNode::send_parameters(void) {

        // parameter packet: $, type, subtype, index, reserved(1), payload(4), reserved(1), checksum, \n

        uint8_t tx_param[PARAM_PACKET_SIZE] = {0};

        // handle uint8
        memset(tx_param, 0, PARAM_PACKET_SIZE);
        tx_param[0] = PACKET_PREFIX;
        tx_param[1] = PACKET_TYPE_PARAM;
        tx_param[2] = PARAM_TYPE_UINT8;
        tx_param[11] = '\n';

        uint8_t temp_uint8;
        for(uint8_t i=0; i < SIZE_PARAMS_UINT8; i++) {
          tx_param[3] = i;
          temp_uint8 = get_parameter(names_uint8[i]).as_int();
          tx_param[5] = temp_uint8;
          tx_param[10] = crc8ccitt(tx_param, 10);
          std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
          m_serial_driver->port()->send(tx_vector); 
          RCLCPP_INFO(get_logger(), "%s: %u", names_uint8[i], temp_uint8);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        
        // handle uint16
        memset(tx_param, 0, PARAM_PACKET_SIZE);
        tx_param[0] = PACKET_PREFIX;
        tx_param[1] = PACKET_TYPE_PARAM;
        tx_param[2] = PARAM_TYPE_UINT16;
        tx_param[11] = '\n';

        uint16_t temp_uint16;
        for(uint8_t i=0; i < SIZE_PARAMS_UINT16; i++) {
          tx_param[3] = i;
          temp_uint16 = get_parameter(names_uint16[i]).as_int();
          tx_param[5] = temp_uint16 & 0xFF;
          tx_param[6] = temp_uint16 >> 8;
          tx_param[10] = crc8ccitt(tx_param, 10);
          std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
          m_serial_driver->port()->send(tx_vector); 
          RCLCPP_INFO(get_logger(), "%s: %u", names_uint16[i], temp_uint16);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        
        // handle int16
        memset(tx_param, 0, PARAM_PACKET_SIZE);
        tx_param[0] = PACKET_PREFIX;
        tx_param[1] = PACKET_TYPE_PARAM;
        tx_param[2] = PARAM_TYPE_INT16;
        tx_param[11] = '\n';      

        int16_t temp_int16;
        for(uint8_t i=0; i < SIZE_PARAMS_INT16; i++) {
          tx_param[3] = i;
          temp_int16 = get_parameter(names_int16[i]).as_int();
          tx_param[5] = temp_int16 & 0xFF;
          tx_param[6] = temp_int16 >> 8;
          tx_param[10] = crc8ccitt(tx_param, 10);
          std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
          m_serial_driver->port()->send(tx_vector); 
          RCLCPP_INFO(get_logger(), "%s: %d", names_int16[i], temp_int16);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle floats
        memset(tx_param, 0, PARAM_PACKET_SIZE);
        tx_param[0] = PACKET_PREFIX;
        tx_param[1] = PACKET_TYPE_PARAM;
        tx_param[2] = PARAM_TYPE_FLOAT;
        tx_param[11] = '\n';      

        
        f32_to_ui8 u;
        for(uint8_t i=0; i < SIZE_PARAMS_FLOAT; i++) {
          tx_param[3] = i;
          u.f32 = (float) get_parameter(names_float[i]).as_double();
          tx_param[5] = u.ui8[0];
          tx_param[6] = u.ui8[1];
          tx_param[7] = u.ui8[2];
          tx_param[8] = u.ui8[3];
          tx_param[10] = crc8ccitt(tx_param, 10);
          std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
          m_serial_driver->port()->send(tx_vector); 
          RCLCPP_INFO(get_logger(), "%s: %f", names_float[i], u.f32);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle bools
        memset(tx_param, 0, PARAM_PACKET_SIZE);
        tx_param[0] = PACKET_PREFIX;
        tx_param[1] = PACKET_TYPE_PARAM;
        tx_param[2] = PARAM_TYPE_BOOL;
        tx_param[11] = '\n';      

        bool temp_bool;
        for(uint8_t i=0; i < SIZE_PARAMS_BOOL; i++) {
          tx_param[3] = i;
          temp_bool = get_parameter(names_bool[i]).as_bool();
          if(temp_bool) { tx_param[5] = 0x1; } else { tx_param[5] = 0x0; }
          tx_param[10] = crc8ccitt(tx_param, 10);
          std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
          m_serial_driver->port()->send(tx_vector); 
          RCLCPP_INFO(get_logger(), "%s: %u", names_bool[i], temp_bool);
          rclcpp::sleep_for(std::chrono::milliseconds(10));          
        }
    }

    bool FximuNode::handle_sys_status(uint8_t current_status) {

        // sysctl packet: $, type, code, reserved(6), checksum, \n

        if(current_status != prev_sys_status) {

            RCLCPP_INFO(this->get_logger(), "sys_status has changed to %d", sys_status);

            if((current_status & 0b01000000) == 64) {

                // send reset if bit6 is set, usb connection will crash, so lifecycle node has to respawn
                // for old node to shutdown properly takes ~20 seconds

                RCLCPP_INFO(this->get_logger(), "sending reset command to IMU device");
                RCLCPP_INFO(this->get_logger(), "restarting lifecycle node. will take 20 seconds");

                uint8_t tx_param[PARAM_PACKET_SIZE] = {0};
                memset(tx_param, 0, PARAM_PACKET_SIZE);

                tx_param[0] = PACKET_PREFIX;
                tx_param[1] = PACKET_TYPE_CTL;
                tx_param[2] = SYSCTL_RESET;
                tx_param[10] = crc8ccitt(tx_param, 10);

                tx_param[11] = '\n';

                std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
                m_serial_driver->port()->send(tx_vector); 
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                
                
            } else if((current_status & 0b00100000) == 32) {

                // send soft reset if bit5 is set
                RCLCPP_INFO(this->get_logger(), "sending soft reset command to IMU device");

                uint8_t tx_param[PARAM_PACKET_SIZE] = {0};
                memset(tx_param, 0, PARAM_PACKET_SIZE);

                tx_param[0] = PACKET_PREFIX;
                tx_param[1] = PACKET_TYPE_CTL;
                tx_param[2] = SYSCTL_SOFTRESET;
                tx_param[10] = crc8ccitt(tx_param, 10);

                tx_param[11] = '\n';

                std::vector<uint8_t> tx_vector(tx_param, tx_param + PARAM_PACKET_SIZE);
                m_serial_driver->port()->send(tx_vector); 
                rclcpp::sleep_for(std::chrono::milliseconds(100));

            } 

            prev_sys_status = current_status; // set previous_status
            return true;
        } else {
          return false;
        }

    }

  } // CLASS END

}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::fximu_driver::FximuNode)