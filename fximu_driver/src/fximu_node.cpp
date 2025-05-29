#include "fximu_driver/util.h"
#include "fximu_driver/parameters.h"
#include "fximu_driver/fximu_node.hpp"

#define DEC_POINTS 5

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

using namespace std::chrono;
using timestamp = std::pair<seconds, nanoseconds>;

// TODO: ENU or NED? option to select/ right now its wrong.
// TODO: AUDIT: is gravity removed, is there a boolean for it? is it removed wrong from the packet.

namespace drivers
{
  namespace fximu_driver
  {

    FximuNode::FximuNode(const rclcpp::NodeOptions & options)
    : lc::LifecycleNode("fximu_node", options), m_owned_ctx{new IoContext(2)}, m_serial_driver{new FximuDriver(*m_owned_ctx)}
    {
      get_serial_parameters();                   // get serial parameters for connection
      declare_parameters();                      // get device parameters with default from yaml file
      filter_timing = new AdaptiveFilter();      // timing filter for imu packet delay
	  filter_rtt = new AdaptiveFilterRTT();      // ntp round trip time filter

      init_packet.assign(6, 0);                  // initial sync packet
      sync_packet.assign(64, 0);                 // ntp sync packet
      param_packet.assign(64, 0);                // parameter packet
      imu_packet.assign(64, 0);                  // imu_packet

    }

    FximuNode::FximuNode(const rclcpp::NodeOptions & options, const IoContext & ctx)
    : lc::LifecycleNode("fximu_node", options), m_serial_driver{new FximuDriver(ctx)}
    {
      get_serial_parameters();                   // get serial parameters for connection
      declare_parameters();                      // get device parameters with default from yaml file
      filter_timing = new AdaptiveFilter();      // timing filter for imu packet delay
	  filter_rtt = new AdaptiveFilterRTT();      // ntp round trip time filter

      init_packet.assign(6, 0);                  // initial sync packet
      sync_packet.assign(64, 0);                 // ntp sync packet
      param_packet.assign(64, 0);                // parameter packet
      imu_packet.assign(64, 0);                  // imu_packet

    }

    FximuNode::~FximuNode() {
      if (m_owned_ctx) {
        m_owned_ctx->waitForExit();
      }
    }

    LNI::CallbackReturn FximuNode::on_configure(const lc::State & state) {

      (void)state;

      init_packet[0] = INIT_PREFIX;                  // initial sync packet
      sync_packet[0] = SYNC_PREFIX;                  // sync packet
      param_packet[0] = PARAMETER_PREFIX;            // parameter packet
      imu_packet[0] = DATA_PREFIX;                   // imu packet

      init_packet[SYNC_PACKET_SIZE - 1] = PACKET_POSTFIX;
      sync_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;
      param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;
      imu_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

      // create publishers, must be done after declaring parameters
      imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS{100});
      if(enable_magneto && publish_magneto) {
        mag_publisher = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", rclcpp::QoS{100});
      }

      try {																				// create serial connection
        m_serial_driver->init_port(m_device_name, *m_device_config);
        if (!m_serial_driver->port()->is_open()) {
          m_serial_driver->port()->open();
        }
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "error creating serial port: %s - %s", m_device_name.c_str(), ex.what());
        return LNI::CallbackReturn::FAILURE;
      }

      // must start receiving data before sending parameters
      m_serial_driver->port()->async_receive(
        std::bind(&FximuNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
      );

      send_parameters();																// send device parameters
      rclcpp::sleep_for(std::chrono::milliseconds(3000));                               // 3 second pause
      init_sync();																	    // send sync packet, block until beginning of second
                                                                                        // init_sync also enables sending of imu packets

      RCLCPP_INFO(get_logger(), "FXIMU lifecycle successfully configured");

      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_activate(const lc::State & state) {
      (void)state;
      imu_publisher->on_activate();
      if(enable_magneto && publish_magneto) {
        mag_publisher->on_activate();
      }
      RCLCPP_INFO(get_logger(), "FXIMU lifecycle activated");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_deactivate(const lc::State & state) {
      (void)state;
      imu_publisher->on_deactivate();
      if(enable_magneto && publish_magneto) {
        mag_publisher->on_deactivate();
      }
      RCLCPP_INFO(get_logger(), "FXIMU lifecycle deactivated");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_cleanup(const lc::State & state) {
      (void)state;
      m_serial_driver->port()->close();
      imu_publisher.reset();
      if(enable_magneto && publish_magneto) {
        mag_publisher.reset();
      }
      RCLCPP_INFO(get_logger(), "FXIMU lifecycle cleaned up");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_shutdown(const lc::State & state) {
      (void)state;
      RCLCPP_INFO(get_logger(), "FXIMU lifecycle shutting down");
      return LNI::CallbackReturn::SUCCESS;
    }

    void FximuNode::reset_driver() {

      auto current_state = get_current_state();

      // deactivate if enabled
      if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        on_deactivate(current_state);
      }

      // cleanup
      on_cleanup(current_state);

      // configure
      if (on_configure(current_state) != LNI::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(get_logger(), "FXIMU lifecycle Reconfiguration failed!");
        return;
      }

      // activate
      on_activate(current_state);

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

      // TODO: use filter warmed up flag, instead of first_trigger for 63 64

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

    void FximuNode::declare_parameters() {

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

        // magnetometer enabled and whether we are publishing the data
        enable_magneto = this->get_parameter("enableMagnetometer").as_bool();
        publish_magneto = this->get_parameter("pubMagnetometerData").as_bool();

      } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "parameter exception %s", ex.what());
        throw ex;
      }
    }

    void FximuNode::send_parameters()
    {

        // parameter packet: $, type, subtype, index, reserved(1), payload(4), reserved(1), checksum, \n

        // handle uint8
        param_packet.assign(USB_PACKET_SIZE, 0);
        param_packet[0] = PARAMETER_PREFIX;
        param_packet[1] = PACKET_TYPE_PARAM;
        param_packet[2] = PARAM_TYPE_UINT8;

        uint8_t temp_uint8;
        for(uint8_t i=0; i < SIZE_PARAMS_UINT8; i++) {
          param_packet[3] = i;
          temp_uint8 = get_parameter(names_uint8[i]).as_int();
          param_packet[5] = temp_uint8;
          param_packet[62] = crc8(param_packet, 10);
          m_serial_driver->port()->send(param_packet);
          RCLCPP_DEBUG(this->get_logger(), "%s: %u", names_uint8[i], temp_uint8);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle uint16
        param_packet.assign(USB_PACKET_SIZE, 0);
        param_packet[0] = PARAMETER_PREFIX;
        param_packet[1] = PACKET_TYPE_PARAM;
        param_packet[2] = PARAM_TYPE_UINT16;
        param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

        uint16_t temp_uint16;
        for(uint8_t i=0; i < SIZE_PARAMS_UINT16; i++) {
          param_packet[3] = i;
          temp_uint16 = get_parameter(names_uint16[i]).as_int();
          param_packet[5] = temp_uint16 & 0xFF;
          param_packet[6] = temp_uint16 >> 8;
          param_packet[62] = crc8(param_packet, 10);
          m_serial_driver->port()->send(param_packet);
          RCLCPP_DEBUG(this->get_logger(), "%s: %u", names_uint16[i], temp_uint16);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle int16
        param_packet.assign(USB_PACKET_SIZE, 0);
        param_packet[0] = PARAMETER_PREFIX;
        param_packet[1] = PACKET_TYPE_PARAM;
        param_packet[2] = PARAM_TYPE_INT16;
        param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

        int16_t temp_int16;
        for(uint8_t i=0; i < SIZE_PARAMS_INT16; i++) {
          param_packet[3] = i;
          temp_int16 = get_parameter(names_int16[i]).as_int();
          param_packet[5] = temp_int16 & 0xFF;
          param_packet[6] = temp_int16 >> 8;
          param_packet[62] = crc8(param_packet, 10);
          m_serial_driver->port()->send(param_packet);
          RCLCPP_DEBUG(this->get_logger(), "%s: %d", names_int16[i], temp_int16);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle floats
        param_packet.assign(USB_PACKET_SIZE, 0);
        param_packet[0] = PARAMETER_PREFIX;
        param_packet[1] = PACKET_TYPE_PARAM;
        param_packet[2] = PARAM_TYPE_FLOAT;
        param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

        f32_to_ui8 u;
        for(uint8_t i=0; i < SIZE_PARAMS_FLOAT; i++) {
          param_packet[3] = i;
          u.f32 = (float) get_parameter(names_float[i]).as_double();
          param_packet[5] = u.ui8[0];
          param_packet[6] = u.ui8[1];
          param_packet[7] = u.ui8[2];
          param_packet[8] = u.ui8[3];
          param_packet[62] = crc8(param_packet, 10);
          m_serial_driver->port()->send(param_packet);
          RCLCPP_DEBUG(this->get_logger(), "%s: %f", names_float[i], u.f32);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        // handle bools
        param_packet.assign(USB_PACKET_SIZE, 0);
        param_packet[0] = PARAMETER_PREFIX;
        param_packet[1] = PACKET_TYPE_PARAM;
        param_packet[2] = PARAM_TYPE_BOOL;
        param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

        bool temp_bool;
        for(uint8_t i=0; i < SIZE_PARAMS_BOOL; i++) {
          param_packet[3] = i;
          temp_bool = get_parameter(names_bool[i]).as_bool();
          if(temp_bool) { param_packet[5] = 0x1; } else { param_packet[5] = 0x0; }
          param_packet[62] = crc8(param_packet, 10);
          m_serial_driver->port()->send(param_packet);
          RCLCPP_DEBUG(this->get_logger(), "%s: %u", names_bool[i], temp_bool);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // notice: only to be called from receive_callback for imu_packet
    bool FximuNode::handle_sys_status(uint8_t current_sys_status) {

        // see if sys status changed
        if(current_sys_status != prev_sys_status) {

          prev_sys_status = current_sys_status;        // placed top, in case of reset or soft reset.

          // Bit 7: eeprom init error
          if ((current_sys_status >> 7) & 1) { // Check if bit 7 is set
            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 7 (0x80): EEPROM Initialization Error");
          }

          // Bit 6: hard restart required
          if ((current_sys_status >> 6) & 1) { // Check if bit 6 is set

            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 6 (0x40): Hard Restart Required");

            // send reset if bit6 is set, usb connection will crash, so lifecycle node has to respawn
            param_packet.assign(USB_PACKET_SIZE, 0);
            param_packet[0] = PARAMETER_PREFIX;
            param_packet[1] = PACKET_TYPE_SYSCTL;
            param_packet[2] = SYSCTL_RESET;
            param_packet[62] = crc8(param_packet, 10);
            param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

            m_serial_driver->port()->send(param_packet);
            rclcpp::sleep_for(std::chrono::milliseconds(3000));  // sleep for 3s for serial port to ready

            this->reset_driver();                                // restart driver
            return true;
          }

          // Bit 5: soft restart required
          if ((current_sys_status >> 5) & 1) { // Check if bit 5 is set

            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 5 (0x20): Sensor Restart Required");

            // send soft reset if bit5 is set
            param_packet.assign(USB_PACKET_SIZE, 0);
            param_packet[0] = PARAMETER_PREFIX;
            param_packet[1] = PACKET_TYPE_SYSCTL;
            param_packet[2] = SYSCTL_SOFTRESET;
            param_packet[62] = crc8(param_packet, 10);
            param_packet[USB_PACKET_SIZE - 1] = PACKET_POSTFIX;

            m_serial_driver->port()->send(param_packet);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            return true;
          }

          // Bit 4: eeprom write error
          if ((current_sys_status >> 4) & 1) { // Check if bit 4 is set
             // RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 4 (0x10): EEPROM Write Error");
			 RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 4 (0x10): Timer2 Triggered");
          }

          // Bit 3: ui filter configuration error
          if ((current_sys_status >> 3) & 1) { // Check if bit 3 is set
            // TODO: RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 3 (0x08): UI Filter Configuration Error");
			RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 3 (0x08): g_rtc_sub_mark fix");
          }

          // Bit 2: magnetic overflow
          if ((current_sys_status >> 2) & 1) { // Check if bit 2 is set
            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 2 (0x04): Magnetic Overflow");
          }

          // Bit 1: sensor restarted
          if ((current_sys_status >> 1) & 1) { // Check if bit 1 is set
            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 1 (0x02): Sensor Restart Complete");
          }

          // Bit 0: initial calibration failed due to non-steady state
          if (current_sys_status & 1) { // Check if bit 0 is set (same as (sys_status >> 0) & 1)
            RCLCPP_INFO(this->get_logger(), "SYS_STATUS Bit 0 (0x01): Initial Calibration Failed (Non-Steady State)");
          }
        }
        return false;
    }

    void FximuNode::init_sync()
    {

      auto sync_time = get_time();
      uint32_t host_seconds;
      uint32_t previous_nanos = 0;

      while(true) {
        sync_time = get_time();
        host_seconds = std::chrono::duration_cast<std::chrono::seconds>(sync_time.time_since_epoch()).count();
        uint32_t host_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(sync_time.time_since_epoch() - std::chrono::seconds(host_seconds)).count();
        if(host_nanos < previous_nanos) { break; }
        previous_nanos = host_nanos;
      }

      u32_to_ui8 u;
      u.u32 = host_seconds;
      init_packet[1] = u.ui8[0];
      init_packet[2] = u.ui8[1];
      init_packet[3] = u.ui8[2];
      init_packet[4] = u.ui8[3];

      m_serial_driver->port()->send(init_packet);				                    // always send sync mode

	  RCLCPP_INFO(this->get_logger(), "FXIMU SYNC seconds %u", host_seconds);
    }

    void FximuNode::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
    {

      if (
        (bytes_transferred == 64) &													// packet size check
        (buffer[0] == DATA_PREFIX) &                                  			    // prefix check
        (buffer[USB_PACKET_SIZE - 1] == PACKET_POSTFIX)                   			// postfix check
      ) {
        const auto received_marker = get_time();									// get packet received time
        const auto since_epoch = received_marker.time_since_epoch();				// single epoch calculation with direct uint32_t conversion

        const uint32_t received_marker_sec = static_cast<uint32_t>(					// received second
            std::chrono::duration_cast<std::chrono::seconds>(since_epoch).count()
        );
        const uint32_t received_marker_ns = static_cast<uint32_t>(					// received nanoseconds with guaranteed range [0, 999,999,999]
            (since_epoch % std::chrono::seconds(1)).count()
        );

        timestamp received_timestamp {												// received  timestamp structure
          std::chrono::seconds{received_marker_sec},
          std::chrono::nanoseconds{received_marker_ns}
        };

        // get crc from received packet
        uint8_t crc8 = buffer[USB_PACKET_SIZE - 2];                        			// crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), USB_PACKET_SIZE - 3);				// calculated crc

        if (crc8 != c_crc8) {
          RCLCPP_ERROR(this->get_logger(), "IMU CRC8:%d not equal c_CRC8:%d", crc8, c_crc8);  // crc error
        } else {

          sys_status = buffer[61];

          if(read_state == -1) {
            RCLCPP_INFO(this->get_logger(), "FXIMU init read_state = -1");
            prev_packet_seq = buffer[60];								  // record incoming packet sequence number as previous
            read_state = 0;                                               // set read state to normal
            handle_sys_status(sys_status);                                // handle sys_status even in first packet
            return;                                                       // return if first read
          } else {
            packet_seq = buffer[60];								      // incoming packet sequence number
            uint8_t expected_seq = prev_packet_seq + 1;                   // calculate expected sequence number
            if(packet_seq != expected_seq) {
              RCLCPP_ERROR(this->get_logger(), "prev_packet_seq: %d, expected_seq: %d, skip: packet_seq: %d", prev_packet_seq, expected_seq, packet_seq);
              if(read_state == 0) { read_state = 1; } else if(read_state == 1) { read_state = -1; return; }   // reset state if skips for a second time
            } else {
              read_state = 0;
            }
            prev_packet_seq = packet_seq;								  // record sequence number as previous
            handle_sys_status(sys_status);                                // handle sys_status
          }

          // notice: we can have read_state 0 or 1 at this point

          // compose imu packet
          auto imu_data = sensor_msgs::msg::Imu(); 
          auto mag_data = sensor_msgs::msg::MagneticField();

          imu_data.orientation.w = R4(buffer, 1);                         // q0
          imu_data.orientation.x = R4(buffer, 1 + 4);                     // q1
          imu_data.orientation.y = R4(buffer, 1 + 8);                     // q2
          imu_data.orientation.z = R4(buffer, 1 + 12);                    // q3

          imu_data.linear_acceleration.x = R4(buffer, 17);				  // ax
          imu_data.linear_acceleration.y = R4(buffer, 17 + 4);            // ay
          imu_data.linear_acceleration.z = R4(buffer, 17 + 8);			  // az

          imu_data.angular_velocity.x = R4(buffer, 29);					  // wx
          imu_data.angular_velocity.y = R4(buffer, 29 + 4);				  // wy
          imu_data.angular_velocity.z = R4(buffer, 29 + 8);				  // wz

          mag_data.magnetic_field.x = R4(buffer, 41);					  // mx
          mag_data.magnetic_field.y = R4(buffer, 41 + 4);				  // my
          mag_data.magnetic_field.z = R4(buffer, 41 + 8);			      // mz

          // TODO: make const
          uint32_t device_rtc_seconds = U4(buffer, 53);					  // RTC seconds
          uint16_t device_rtc_ticks = U2(buffer, 57);			          // RTC ticks
          int8_t rtc_offset = (int8_t) buffer[59];                        // RTC sync offset

          // cap rtc_offset, it might be larger than these values.
		  if(rtc_offset == 127) { rtc_offset = 125; } else if(rtc_offset == 126) { rtc_offset = -125; }

          // calculate nanos
          uint32_t device_rtc_nanos = device_rtc_ticks * 30517.578125;

          // device timestamp structure
          timestamp device_timestamp {std::chrono::seconds{device_rtc_seconds}, std::chrono::nanoseconds{device_rtc_nanos}};

          // calculate nanos diff
          const auto received_point = received_timestamp.first + received_timestamp.second;
          const auto device_point = device_timestamp.first + device_timestamp.second;
          const int32_t nanos_diff = (received_point - device_point).count();

          // TODO: driver corrects RTC with NTP like syncronization
          //       offset is calculated and tracked.
          //       imu packet should be corrected with this offset.
          //       this offset updated after each sync.
          //       after each sync

		  if(abs(nanos_diff) > 900000000) {

		  	RCLCPP_ERROR(this->get_logger(), "threshold exceeded - nanos_diff %d rtc %d.%d host %d.%d",
				nanos_diff,
				device_rtc_seconds,
				device_rtc_ticks,
				received_marker_sec,
				received_marker_ns);

            // TODO: connection state
            // TODO: reconnect_feature() on tm4c - it must recalculate cached vars, and reset certain values.
			// TODO: skip packet
            // TODO: if second time, reset driver
			// this->reset_driver();

		  } else {

            // TODO: dont need filter_timing, any more, but test before erasing it.
            //       test that rtt is in par with delay. as a matter of fact, we can
            //       investigate why rtt is expected/2 on x86.
            //       observe what on x86 and rpi, the rtt, and packet delay.
            //       packet delay must be half of rtt
			filter_timing->update(nanos_diff);

          	// stamp for imu packet
          	// rclcpp::Time stamp = rclcpp::Clock().now();
          	// rclcpp::Time stamp = rclcpp::Time(device_rtc_seconds) + rclcpp::Duration(0, device_rtc_nanos);
          	rclcpp::Time stamp(static_cast<uint64_t>((device_rtc_seconds * 1e9) + device_rtc_nanos));
          	imu_data.header.stamp = stamp;
          	imu_data.header.frame_id = imu_frame_id;

          	// process magneto only if enabled and publishing
          	if(enable_magneto && publish_magneto) {
             	mag_data.header.stamp = imu_data.header.stamp; // mag timestamp
             	mag_data.header.frame_id = mag_frame_id;       // mag frameid
             	imu_publisher->publish(imu_data);              // publish imu data
             	mag_publisher->publish(mag_data);              // publish mag data
          	} else {
             	imu_publisher->publish(imu_data);              // publish imu data only
          	}
			/*
		  	RCLCPP_INFO(this->get_logger(), "nanos_diff %d rtc %d.%d host %d.%d",
				nanos_diff,
				device_rtc_seconds,
				device_rtc_ticks,
				received_marker_sec,
				received_marker_ns);
            */
		  }

          /*
		  double period_mean = filter_timing->getAverage();
          RCLCPP_INFO(this->get_logger(), "avg %f std_dev %f rtc_offset %d",
                      period_mean,
                      filter_timing->getStdDev(),
			    	  rtc_offset);
          */

          // TODO:  when prev_device_rtc_tics is 0. should skip first run, but it already does
          //        however even when it skips first run, it does not skip by noting prev_device_rtc_ticks
          // mid-second interrupt, according to sensor rtc
          if((prev_device_rtc_ticks < 16384) && (device_rtc_ticks >= 16384)) {

              if(device_rtc_seconds % 4 == 0) {                        // each 3 seconds

                u32_to_ui8 u;                                          // sync request procedure starts here
                i32_to_ui8 i;

                // precalculated offset to gain time
                // i.i32 = (int32_t) filter_rtt->getOffset();
                i.i32 = phi;
                sync_packet[9] = i.ui8[0];
                sync_packet[10] = i.ui8[1];
                sync_packet[11] = i.ui8[2];
                sync_packet[12] = i.ui8[3];

                auto t1_time = get_time();  					   // time marker
                t1_time += std::chrono::microseconds(0);           // add a propagation delay

                t1_seconds = std::chrono::duration_cast<std::chrono::seconds>(t1_time.time_since_epoch()).count();
                t1_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(t1_time.time_since_epoch() - std::chrono::seconds(t1_seconds)).count();

                u.u32 = t1_seconds;
                sync_packet[1] = u.ui8[0];
                sync_packet[2] = u.ui8[1];
                sync_packet[3] = u.ui8[2];
                sync_packet[4] = u.ui8[3];

                u.u32 = t1_nanos;
                sync_packet[5] = u.ui8[0];
                sync_packet[6] = u.ui8[1];
                sync_packet[7] = u.ui8[2];
                sync_packet[8] = u.ui8[3];

                m_serial_driver->port()->send(sync_packet);

              }
          }
          prev_device_rtc_ticks = device_rtc_ticks;
        }

      } else if(
        (bytes_transferred == 64) &
        (buffer[0] == DIAG_PREFIX) &                                // prefix check
        (buffer[USB_PACKET_SIZE - 1] == PACKET_POSTFIX)               // postfix check
	  ) {

        const auto received_marker = get_time();									// get packet received time
        const auto since_epoch = received_marker.time_since_epoch();				// single epoch calculation with direct uint32_t conversion

        const uint32_t received_marker_sec = static_cast<uint32_t>(					// received second
            std::chrono::duration_cast<std::chrono::seconds>(since_epoch).count()
        );
        const uint32_t received_marker_ns = static_cast<uint32_t>(					// received nanoseconds with guaranteed range [0, 999,999,999]
            (since_epoch % std::chrono::seconds(1)).count()
        );

        // get crc from received packet
        uint8_t crc8 = buffer[USB_PACKET_SIZE - 2];                   // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), USB_PACKET_SIZE - 3);

        if(crc8 != c_crc8) {
          RCLCPP_ERROR(this->get_logger(), "DIAG CRC8:%d not equal c_CRC8:%d", crc8, c_crc8);
        } else {

           float ax_bias = R4(buffer, 1);                           // ax_bias
           float ay_bias = R4(buffer, 5);                           // ay_bias
           float az_bias = R4(buffer, 9);                           // az_bias

           float wx_bias = R4(buffer, 13);                          // wx_bias
           float wy_bias = R4(buffer, 17);                          // wy_bias
           float wz_bias = R4(buffer, 21);                          // wz_bias

           float mag_temp = R4(buffer, 45);
		   float sensor_temp = R4(buffer, 49);

		   uint32_t t2_seconds = U4(buffer, 25);		// T2 seconds
		   uint32_t t2_nanos = U4(buffer, 29);			// T2 nanos

		   uint32_t t3_seconds = U4(buffer, 33);		// T3 seconds
		   uint32_t t3_nanos = U4(buffer, 37);			// T3 nanos

           int32_t external_offset = I4(buffer, 41);

	       timestamp t1 {std::chrono::seconds(t1_seconds), std::chrono::nanoseconds(t1_nanos)};
           timestamp t2 {std::chrono::seconds{t2_seconds}, std::chrono::nanoseconds{t2_nanos}};
		   timestamp t3 {std::chrono::seconds{t3_seconds}, std::chrono::nanoseconds{t3_nanos}};
           timestamp t4 {std::chrono::seconds{received_marker_sec}, std::chrono::nanoseconds{received_marker_ns}};

           const auto t1_point = t1.first + t1.second;
           const auto t2_point = t2.first + t2.second;
           const auto t3_point = t3.first + t3.second;
           const auto t4_point = t4.first + t4.second;

           sigma = (t4_point - t1_point).count() - (t3_point - t2_point).count();
	       phi = ((t2_point - t1_point).count() + (t3_point - t4_point).count()) / 2;

           filter_rtt->update(sigma, phi);

		   // δ = (T4 - T1) - (T3 - T2)
           // θ = [(T2 - T1) + (T3 - T4)] / 2

           //RCLCPP_INFO(this->get_logger(), "t1 %lld t2 %lld t3 %lld t4 %lld", t1_point.count(), t2_point.count(), t3_point.count(), t4_point.count());

           RCLCPP_INFO(this->get_logger(), "RTT %f OFFSET %f external_offset %d", filter_rtt->getRTT(), filter_rtt->getOffset(), external_offset);

           // RCLCPP_INFO(this->get_logger(), "ax: %.4f, ay: %.4f, az: %.4f, wx: %.4f, wy: %.4f, wz: %.4f, mt: %.1fC, st: %.1fC", ax_bias, ay_bias, az_bias, wx_bias, wy_bias, wz_bias, mag_temp, sensor_temp);
		   // RCLCPP_INFO(this->get_logger(), "device_rtc_seconds: %u, device_rtc_sub_seconds: %u, device_rtc_nanos: %u", device_rtc_seconds, device_rtc_sub_seconds, device_rtc_nanos);

	     }
      }
	} // end receive_callback
  } // namespace fximu_driver
} // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::fximu_driver::FximuNode)