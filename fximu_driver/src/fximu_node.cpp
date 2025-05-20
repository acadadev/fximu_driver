#include "fximu_driver/util.h"
#include "fximu_driver/parameters.h"
#include "fximu_driver/fximu_node.hpp"

#include <cstdlib>

#define DEC_POINTS 5

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

using namespace std::chrono;
using timestamp = std::pair<seconds, nanoseconds>;

// TODO: magnetometer auto calibration above
// TODO: ENU or NED? option to select/ right now its wrong.
// TODO: also remove imu is packet wrong gravity.

namespace drivers
{
  namespace fximu_driver
  {

    FximuNode::FximuNode(const rclcpp::NodeOptions & options)
    : lc::LifecycleNode("fximu_node", options), m_owned_ctx{new IoContext(2)}, m_serial_driver{new FximuDriver(*m_owned_ctx)}
    {
      get_serial_parameters();
      filter_timing = new AdaptiveFilter();
    }

    FximuNode::FximuNode(const rclcpp::NodeOptions & options, const IoContext & ctx)
    : lc::LifecycleNode("fximu_node", options), m_serial_driver{new FximuDriver(ctx)}
    {
      get_serial_parameters();
      filter_timing = new AdaptiveFilter();
    }

    FximuNode::~FximuNode() {
      if (m_owned_ctx) {
        m_owned_ctx->waitForExit();
      }
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

      // TODO: notice this was removed and important request sys status, see if need a reset

      // send sync packet, block until beginning of second
      send_init_sync();

      // create publisher
      imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS{100});
      mag_publisher = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", rclcpp::QoS{100});

      // start receiving data
      m_serial_driver->port()->async_receive(
        std::bind(&FximuNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
      );   

      RCLCPP_INFO(get_logger(), "FXIMU successfully configured.");

      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_activate(const lc::State & state) {
      (void)state;
      imu_publisher->on_activate();
      mag_publisher->on_activate();
      RCLCPP_INFO(get_logger(), "FXIMU activated.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_deactivate(const lc::State & state) {
      (void)state;
      imu_publisher->on_deactivate();
      mag_publisher->on_deactivate();
      RCLCPP_INFO(get_logger(), "FXIMU deactivated.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_cleanup(const lc::State & state) {
      (void)state;
      m_serial_driver->port()->close();
      imu_publisher.reset();
      mag_publisher.reset();
      RCLCPP_INFO(get_logger(), "FXIMU cleaned up.");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_shutdown(const lc::State & state) {
      (void)state;
      RCLCPP_INFO(get_logger(), "FXIMU shutting down.");
      return LNI::CallbackReturn::SUCCESS;
    }

    void FximuNode::get_serial_parameters() {

      uint32_t baud_rate{};
      auto fc = FlowControl::NONE;
      auto pt = Parity::NONE;
      auto sb = StopBits::ONE;

      try {
        m_device_name = declare_parameter<std::string>("device_name", "/dev/fximu");
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "invalid device_name");
        throw ex;
      }

      try {
        baud_rate = declare_parameter<int>("baud_rate", 921600);
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "invalid baud_rate");
        throw ex;
      }

      try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "none");
        if (fc_string == "none") {
          fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
          fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
          fc = FlowControl::SOFTWARE;
        } else {
          fc = FlowControl::NONE;
        }
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "invalid flow_control");
        throw ex;
      }

      try {
        const auto pt_string = declare_parameter<std::string>("parity", "none");
        if (pt_string == "none") {
          pt = Parity::NONE;
        } else if (pt_string == "odd") {
          pt = Parity::ODD;
        } else if (pt_string == "even") {
          pt = Parity::EVEN;
        } else {
          pt = Parity::NONE;
        }
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "invalid parity");
        throw ex;
      }

      try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "1");
        if (sb_string == "1" || sb_string == "1.0") {
          sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
          sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
          sb = StopBits::TWO;
        } else {
          sb = StopBits::ONE;
        }
      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "invalid stop bits");
        throw ex;
      }
      m_device_config = std::make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
    }

    void FximuNode::get_device_parameters() {

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

        // default is no averaging
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

        this->declare_parameter<float>("filterGainAccel", 0.02);  // filter gain for accelerometer and gyro
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

    void FximuNode::send_parameters()
    {

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

    void FximuNode::send_mcu_sync(uint32_t seconds, uint32_t nanos)
    {

      auto time = get_time();  // time marker
      time += std::chrono::microseconds(0);             // add a propagation delay
      std::vector<uint8_t> mcu_sync_packet;                   // mcu sync packet, 12 bytes
      mcu_sync_packet.push_back('+');                       // mcu sync packet id
      u32_to_ui8 u;

      u.u32 = seconds;
      mcu_sync_packet.push_back(u.ui8[0]);
      mcu_sync_packet.push_back(u.ui8[1]);
      mcu_sync_packet.push_back(u.ui8[2]);
      mcu_sync_packet.push_back(u.ui8[3]);

      u.u32 = nanos;
      mcu_sync_packet.push_back(u.ui8[0]);
      mcu_sync_packet.push_back(u.ui8[1]);
      mcu_sync_packet.push_back(u.ui8[2]);
      mcu_sync_packet.push_back(u.ui8[3]);

      mcu_sync_packet.push_back(0x0);                       // spare byte
      mcu_sync_packet.push_back(0x0);                       // spare byte

      mcu_sync_packet.push_back(PACKET_POSTFIX);  // 12-bytes total

      m_serial_driver->port()->send(mcu_sync_packet);

    }

    void FximuNode::send_init_sync()
    {

      std::vector<uint8_t> sync_packet;
      auto time = get_time();
      uint32_t host_seconds;
      uint32_t previous_nanos = 0;

      // TODO: trigger this 1ms before second, allowing transmission time, yet, putting the future value inside packet.
      // TODO: it just needs to be sent a little bit earlier.
      while(true) {
        time = get_time();
        host_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        uint32_t host_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
          time.time_since_epoch() - std::chrono::seconds(host_seconds)).count();
        if(host_nanos < previous_nanos) { break; }
        previous_nanos = host_nanos;
      }

      u32_to_ui8 u;
      u.u32 = host_seconds;
      sync_packet.push_back('>');
      sync_packet.push_back(u.ui8[0]);
      sync_packet.push_back(u.ui8[1]);
      sync_packet.push_back(u.ui8[2]);
      sync_packet.push_back(u.ui8[3]);
      sync_packet.push_back(PACKET_POSTFIX);

      // send packet
      m_serial_driver->port()->send(sync_packet);
    }

    void FximuNode::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
    {

      const auto received_time = get_time();

      // single epoch calculation with direct uint32_t conversion
      const auto since_epoch = received_time.time_since_epoch();
      const uint32_t received_sec = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::seconds>(since_epoch).count()
      );

      // nanoseconds with guaranteed range [0, 999,999,999]
      const uint32_t received_ns = static_cast<uint32_t>(
          (since_epoch % std::chrono::seconds(1)).count()
      );

      // final timestamp structure
      timestamp received_timestamp{
        std::chrono::seconds{received_sec % 86400},
        std::chrono::nanoseconds{received_ns}
      };

      if (
        (bytes_transferred == 64) &
        (buffer[0] == PACKET_PREFIX) &                                  // prefix check
        (buffer[IMU_DATA_SIZE - 1] == PACKET_POSTFIX)                   // postfix check
      ) {   

        // get crc from received packet
        uint8_t crc8 = buffer[IMU_DATA_SIZE - 2];                        // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), IMU_DATA_SIZE - 3);

        if (crc8 != c_crc8) {

          RCLCPP_ERROR(this->get_logger(), "imu crc8:%d != c_crc8:%d", crc8, c_crc8);

        } else {

          // as soon as the crc is verified, send mcu sync packet
          // TODO: send_mcu_sync(received_sec, received_ns);

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

          // TIMING values coming from DEVICE
          uint32_t device_rtc_timestamp = U4(buffer, 53);

          // the lsb 17 bits are posix second of day
          uint32_t device_rtc_second_of_day = device_rtc_timestamp & 0x0001FFFF;

          // the msb 15 bits are rtc ticks
          uint16_t device_rtc_ticks = (uint16_t) ((device_rtc_timestamp >> 17) & 0x7FFF);

          // calculate nanos
          uint32_t device_rtc_nanos = device_rtc_ticks * 30517.578125;

          // device timestamp structure
          timestamp device_timestamp{
            std::chrono::seconds{device_rtc_second_of_day},
            std::chrono::nanoseconds{device_rtc_nanos}
          };

          const auto received_point = received_timestamp.first + received_timestamp.second;
          const auto device_point = device_timestamp.first + device_timestamp.second;

          const int32_t nanos_diff = (received_point - device_point).count();

          filter_timing->update(nanos_diff);
          bool norm = filter_timing->normal(nanos_diff);

          // monotonicity check
          timestamp prev_device_timestamp{seconds{prev_device_posix_time}, nanoseconds{prev_device_nanos}};
          if (device_timestamp.first > prev_device_timestamp.first) {
            read_state = STATUS_OK;        // definitively later, since device seconds larger than prev_device_tmiestamp
          } else if (device_timestamp.first < prev_device_timestamp.first) {
            read_state = STATUS_SKIP_SECOND; // definitive skip, since seconds smaller than prev_device_timestamp
          } else {
            if (device_timestamp.second >= prev_device_timestamp.second) {
              read_state = STATUS_OK; // TODO: what about >=, cant be equal due to latency
            } else {
              read_state = STATUS_SKIP_NANOS;
            }
          }

          prev_device_posix_time = device_rtc_second_of_day;
          prev_device_nanos = device_rtc_nanos;

          // TODO: when there is timeskip do not publish packet

          // TODO: remove for debugging, right nor RTC time.
          //if (norm==true && read_state==STATUS_OK)
          //{
            RCLCPP_INFO(this->get_logger(), "read_state: %d, recv: %tu, dev: %tu, norm: %d, nanos_diff: %d, avg: %f", read_state, received_point.count(), device_point.count(), norm, nanos_diff, filter_timing->getAverage());
          //}


          bool publish_packet = true;

          /*
          if (abs(nanos_diff) > 1'000'000'000) {  // >1 second threshold
            read_state = STATUS_OUT_SYNC;
            publish_packet = false;
          } else {
            read_state = STATUS_OK;
          }*/

          if(publish_packet) {

              // imu_data.header.stamp = rclcpp::Clock().now();
              // imu_data.header.stamp = rclcpp::Time(device_posix_time) + rclcpp::Duration(0, device_nanos);
              //rclcpp::Time stamp(static_cast<uint64_t>((device_posix_time * 1e9) + device_nanos));

              //imu_data.header.stamp = stamp;

              // TODO: fix later
              imu_data.header.stamp = rclcpp::Clock().now();

              imu_data.header.frame_id = imu_frame_id;
              mag_data.header.stamp = imu_data.header.stamp;
              mag_data.header.frame_id = mag_frame_id;

              imu_publisher->publish(imu_data);
              mag_publisher->publish(mag_data);

          } else {
            RCLCPP_INFO(this->get_logger(), "Packet not published, read_state: %d", read_state);
          }

        }
      }
    }



  } // CLASS END

}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::fximu_driver::FximuNode)