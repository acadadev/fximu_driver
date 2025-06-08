#include "fximu_driver/util.h"
#include "fximu_driver/parameters.h"
#include "fximu_driver/fximu_node.hpp"

#define DEC_POINTS 5

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

using namespace std::chrono;
using timestamp = std::pair<seconds, nanoseconds>;

// TODO: FEATURES (can only be implemented once system is stable)
//        - use of sensor clock
//        - ENU or NED create option and study
//        - AUDIT: is gravity removed, is there a boolean for it? is it removed wrong from the packet.


// TODO: need a different more recent filter for offset. or not even a filter. the recent
//        - we increment delay with offset
//        - if offset  is negative, we decrement  from it
//        - but what if the delay is negative?
//        - std dev of corrected and raw delays are the same
// TODO: no need to filter phi, but maybe outlier detection

// TODO: limit rtc offset to maybe 64?

// TODO: TEST: observe initial status logs after restart and cold restart to figure out stale data problems
// TODO: offset filter must be regional i.e. taking the last n measurements.

// TODO: the reason of spikes is certainly, t41 flickering on rpi5. i dont know what, but it might cause an
// TODO: instant uprise in the adaptive filter value

// TODO: outlier detection could be dangerous, report it.
// TODO: plot offset unfiltered.

// TODO: mess with asio. could it be sync_receive

// TODO: iocontext was 2, and hardware control protocol is changed

namespace drivers
{
  namespace fximu_driver
  {

    FximuNode::FximuNode(const rclcpp::NodeOptions & options) // TODO: was 2
    : lc::LifecycleNode("fximu_node", options), m_owned_ctx{new IoContext(1)}, m_serial_driver{new FximuDriver(*m_owned_ctx)}
    {
      get_serial_parameters();                                        			// get serial parameters for connection
      declare_parameters();                                           			// get device parameters with default from yaml file
	  filter_rtt = new AdaptiveFilter(0.0, 0.25, 0.01, 128);          			// ntp round trip time filter
      filter_offset = new AdaptiveFilterOutlier(0.0, 0.25, 0.025, 32, 4.0);		// ntp offset time filter
      filter_delay = new AdaptiveFilterPeriod();                      			// packet delay time filter
	  filter_delay_raw = new AdaptiveFilterPeriod();                  			// packet raw delay time filter
      init_packet.assign(6, 0);                  								// initial sync packet
      sync_packet.assign(64, 0);                 								// ntp sync packet
      param_packet.assign(64, 0);                								// parameter packet
      imu_packet.assign(64, 0);                  								// imu_packet
	  // TODO: move these to configure maybe? so not double type
    }

    FximuNode::FximuNode(const rclcpp::NodeOptions & options, const IoContext & ctx)
    : lc::LifecycleNode("fximu_node", options), m_serial_driver{new FximuDriver(ctx)}
    {
      get_serial_parameters();                                        			// get serial parameters for connection
      declare_parameters();                                           			// get device parameters with default from yaml file
	  filter_rtt = new AdaptiveFilter(0.0, 0.25, 0.01, 128);          			// ntp round trip time filter
      filter_offset = new AdaptiveFilterOutlier(0.0, 0.25, 0.025, 32, 4.0);		// ntp offset time filter
      filter_delay = new AdaptiveFilterPeriod();                      			// packet delay time filter
	  filter_delay_raw = new AdaptiveFilterPeriod();                  			// packet raw delay time filter
      init_packet.assign(6, 0);                  								// initial sync packet
      sync_packet.assign(64, 0);                 								// ntp sync packet
      param_packet.assign(64, 0);                								// parameter packet
      imu_packet.assign(64, 0);                  								// imu_packet
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



      send_parameters();																// send device parameters
      rclcpp::sleep_for(std::chrono::milliseconds(100));                                // 0.1 second pause
      init_sync();																	    // send sync packet, block until beginning of second
                                                                                        // init_sync also enables sending of imu packets

      // start receiving packets
      m_serial_driver->port()->async_receive(
        std::bind(&FximuNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
      ); // TODO: remember.


      timer_sync = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [&]() {
            const auto time = std::chrono::high_resolution_clock::now();
            const uint32_t host_seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
            send_sync_request(host_seconds);

/*
    std::vector<uint8_t> reply(64);
    asio::error_code ec;
    size_t bytes_read = asio::read(
        *m_serial_driver->port(),
        asio::buffer(reply),
        asio::transfer_exactly(64),
        ec
    );

    // (4) Handle reply
    if (!ec && bytes_read == 64) {
        const auto t4 = get_time();  // Timestamp immediately after read completes
        process_sync_reply(reply, t4);
    } else {
        RCLCPP_ERROR("Sync reply timeout/corrupt");
    }
*/


        }, 0);
      timer_sync->reset();



      RCLCPP_INFO(get_logger(), "FXIMU lifecycle successfully configured");

      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_activate(const lc::State & state) {
      (void)state;
      imu_publisher->on_activate();
      timer_sync->reset();
      if(enable_magneto && publish_magneto) {
        mag_publisher->on_activate();
      }
      RCLCPP_INFO(get_logger(), "FXIMU lifecycle activated");
      return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn FximuNode::on_deactivate(const lc::State & state) {
      (void)state;
      imu_publisher->on_deactivate();
	  timer_sync->cancel();
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
      timer_sync->cancel();
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

    // notice: only to be called from receive_callback for imu_packet
    bool FximuNode::handle_sys_status(uint8_t current_sys_status, uint8_t sys_code) {

        // see if sys status changed
        if(current_sys_status != prev_sys_status) {

          		prev_sys_status = current_sys_status;        // placed top, in case of reset or soft reset.

          		// Bit 7: eeprom init error
          		if ((current_sys_status >> 7) & 1) { // Check if bit 7 is set
            		RCLCPP_WARN(this->get_logger(), "SYS_STATUS Bit 7 (0x80): EEPROM Initialization Error");
          		}

          		// Bit 6: hard restart required
          		if ((current_sys_status >> 6) & 1) { // Check if bit 6 is set
            		RCLCPP_WARN(this->get_logger(), "SYS_STATUS Bit 6 (0x40): Hard Restart Required");
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
            		RCLCPP_WARN(this->get_logger(), "SYS_STATUS Bit 5 (0x20): Sensor Restart Required");
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
            		 RCLCPP_WARN(this->get_logger(), "SYS_STATUS Bit 4 (0x10): EEPROM Write Error");
         		 }

          		// Bit 3: magneto init error
         		 if ((current_sys_status >> 3) & 1) { // Check if bit 3 is set
            		 RCLCPP_WARN(this->get_logger(), "SYS_STATUS Bit 3 (0x08): Magnetometer Init Error");
         		 }

        		 // First 3 bits indicate meaning of sys_code
		  		 uint8_t code_indicator = current_sys_status & 0x07;

		  		 if(code_indicator == 0x01) {

            		switch(sys_code) {
						case 0x01:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x01): Sensor Restart Complete");
							break;
						case 0x02:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x02): Initial calibration failed due non-steady state threshold");
							break;
						case 0x03:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x03): Initial calibration failed due to non-steady state");
							break;
						case 0x04:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x04): Sensor UI Filter Parameter Error");
							break;
						case 0x05:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x05): RTC Trim applied");
							break;
						case 0x06:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x06): Magnetometer overflow");
							break;
						case 0x07:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x07): RTC SUB adjusted to zero");
							break;
						case 0x08:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x08): USB_Handler : timing_ok = false");
							break;
						case 0x09:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x09): Sensor Parameter error");
						case 0x0A:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x0A): Sensor ID Self Test Failed");
						case 0x0B:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x0B)");
						default:
							RCLCPP_WARN(this->get_logger(), "SYS_CODE (0x%X): Undefined", sys_code);
							break;
					}

		  		 } else if(code_indicator == 0x02) {
					RCLCPP_WARN(this->get_logger(), "FIFO_HEADER: %d", sys_code);
		  		}
        }
        return false;
    }

	void FximuNode::send_sync_request(uint32_t host_seconds)
	{
        u32_to_ui8 u;
        i32_to_ui8 i;

		if(host_seconds % 64 == 62) {						     // at 62nd second
			if(filter_offset->isWarmedUp()) {						 // notice: even we are sending non filtered phi, we wait for filter warm up
				// i.i32 = (int32_t) phi; 							 // sends last measured offset
				i.i32 = filter_offset->getAverage(); // TODO: more soften,
													 // TODO: outlier rejection
													 // TODO: works in a period and resets
				RCLCPP_WARN(this->get_logger(), "sync_request: %d %f", host_seconds, filter_offset->getAverage());
			}
		} else if(host_seconds % 64 == 63) {					 // skip sending packet at 63rd second
			return;
		} else if(host_seconds % 64 == 0) {					 	 // skip sending packet at 64-0 second
			return;
		} else {
			i.i32 = 0;
		}

        sync_packet[9] = i.ui8[0];							   // precalculated offset to gain time
        sync_packet[10] = i.ui8[1];
        sync_packet[11] = i.ui8[2];
        sync_packet[12] = i.ui8[3];

        auto t1_time = get_time();  					   	   // time marker
        t1_time += std::chrono::microseconds(0);               // add a propagation delay
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

		sync_state = 1;

		// SYNC REQUEST END
	}

    void FximuNode::init_sync()
    {

        auto pre_send_offset = std::chrono::nanoseconds(500000);

        uint32_t host_seconds;

        while (true) {
            auto current_time = get_time();
            auto duration_since_epoch = current_time.time_since_epoch();
            host_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch).count();
            auto time_into_current_second = duration_since_epoch - std::chrono::seconds(host_seconds);
            auto time_to_next_second = std::chrono::seconds(1) - time_into_current_second;
            if(time_to_next_second < pre_send_offset) {

            RCLCPP_ERROR(this->get_logger(),
                   "Breaking loop. Current time (seconds): %u, (nanoseconds into second): %ld", host_seconds, time_into_current_second);

              break; // We found our target time, exit the loop
            }
        }

        host_seconds++;

        u32_to_ui8 u;
        u.u32 = host_seconds;
        init_packet[1] = u.ui8[0];
        init_packet[2] = u.ui8[1];
        init_packet[3] = u.ui8[2];
        init_packet[4] = u.ui8[3];

        m_serial_driver->port()->send(init_packet); // always send sync mode

        // RCLCPP_INFO(this->get_logger(), "FXIMU SYNC: Packet sent approx 0.5ms before second %u", host_seconds);
    }

    /*
    void FximuNode::init_sync_exact()
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
    */

    void FximuNode::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
    {

	  // const auto cb_mark = get_time();												// get packet received time
	  const auto cb_mark = m_serial_driver->port()->get_P4();

	  if(
        (bytes_transferred == 64) &
        (buffer[0] == DIAG_PREFIX) &                                				// prefix check
        (buffer[USB_PACKET_SIZE - 1] == PACKET_POSTFIX)               				// postfix check
	  ) {

        // const auto t4_mark = get_time();											// get packet received time
        const auto since_epoch = cb_mark.time_since_epoch();						// single epoch calculation with direct uint32_t conversion

        const uint32_t t4_mark_seconds = static_cast<uint32_t>(					    // received second
            std::chrono::duration_cast<std::chrono::seconds>(since_epoch).count()
        );
        const uint32_t t4_mark_nanos = static_cast<uint32_t>(						// received nanoseconds with guaranteed range [0, 999,999,999]
            (since_epoch % std::chrono::seconds(1)).count()
        );

        // get crc from received packet
        uint8_t crc8 = buffer[USB_PACKET_SIZE - 2];                   // crc @ 62
        uint8_t c_crc8 = crc8ccitt(buffer.data(), USB_PACKET_SIZE - 3);

        if(crc8 != c_crc8) {
          RCLCPP_ERROR(this->get_logger(), "DIAG CRC8:%d not equal c_CRC8:%d", crc8, c_crc8);
        } else {

		   if(sync_state == 0) {
				RCLCPP_ERROR(this->get_logger(), "zero sync state");
		   }

		   if(sync_state == 1) {
				sync_state = 0;
		   }

		   // TODO: first thing to do is without imu packet.

           /*
           float ax_bias = R4(buffer, 1);                           // ax_bias
           float ay_bias = R4(buffer, 5);                           // ay_bias
           float az_bias = R4(buffer, 9);                           // az_bias

           float wx_bias = R4(buffer, 13);                          // wx_bias
           float wy_bias = R4(buffer, 17);                          // wy_bias
           float wz_bias = R4(buffer, 21);                          // wz_bias

           float mag_temp = R4(buffer, 45);
		   float sensor_temp = R4(buffer, 49);
           */

		   uint32_t t2_seconds = U4(buffer, 25);		// T2 seconds
		   uint32_t t2_nanos = U4(buffer, 29);			// T2 nanos

		   uint32_t t3_seconds = U4(buffer, 33);		// T3 seconds
		   uint32_t t3_nanos = U4(buffer, 37);			// T3 nanos

           int16_t applied_rtc_trim = I2(buffer, 41);

           // when timing_ok=false in device, t3 is zero
		   if(t3_seconds == 0 && t3_nanos == 0) {
				// skip
				RCLCPP_ERROR(this->get_logger(), "Skipping calculation due to zero T3");
		   } else if(t2_seconds == 0 && t2_nanos == 0) {
				RCLCPP_ERROR(this->get_logger(), "Skipping calculation due to zero T2");
		   } else {

	   			const timestamp t1 {std::chrono::seconds(t1_seconds), std::chrono::nanoseconds(t1_nanos)};
       	   		const timestamp t2 {std::chrono::seconds{t2_seconds}, std::chrono::nanoseconds{t2_nanos}};
	   			const timestamp t3 {std::chrono::seconds{t3_seconds}, std::chrono::nanoseconds{t3_nanos}};
       			const timestamp t4 {std::chrono::seconds{t4_mark_seconds}, std::chrono::nanoseconds{t4_mark_nanos}};

           		const auto t1_point = t1.first + t1.second;
           		const auto t2_point = t2.first + t2.second;
           		const auto t3_point = t3.first + t3.second;
           		const auto t4_point = t4.first + t4.second;

		   		// δ = (T4 - T1) - (T3 - T2)
           		// θ = [(T2 - T1) + (T3 - T4)] / 2
           		instant_rtt = (t4_point - t1_point).count() - (t3_point - t2_point).count();
	       		instant_offset = ((t2_point - t1_point).count() + (t3_point - t4_point).count()) / 2;

				const auto t41 = (t4_point - t1_point).count();
				const auto t32 = (t3_point - t2_point).count();

				const auto t21 = (t2_point - t1_point).count();
				const auto t34 = (t3_point - t4_point).count();

           		filter_rtt->update(instant_rtt);
           		bool offset_accepted = filter_offset->update(instant_offset);

				// TODO: we can filter t41, and not use if std dev is high. *2
				// TODO: is it when an imu packet is slipped before sync reply is made?
                // TODO: consider the scenario: packet received -> sync_request -> packet_received -> sync_reply.
				// TODO: maybe we can check in the client, that the last state was sync request.
				// TODO: make a state machine around that

				// TODO: if phi is outlier, then use prev_phi, (which might be in the filter)

				// TODO: delay corrected, is spiky. it should not be spiky. that means we filter wrong.
				// TODO: soften the filter.
				// TODO: filter should reset like other std dev ones.
				// TODO: revise
				// TODO: what happens if rejected. it will not add to filter, but next time could be wrong.

				// TODO: except rtc trim, all info printed is local, i.e after sync reply received.

				double delay_avg = filter_delay->getAverage(); 		// notice: purposefully done line this, not to trigger statistics reset
				double delay_raw = filter_delay_raw->getAverage(); 	// notice: purposefully done line this

				// SYNC REPLY REPORT START

				RCLCPP_INFO(this->get_logger(), "instant_rtt %d instant_offset %d accepted %d", instant_rtt, instant_offset, static_cast<int>(offset_accepted));
           		RCLCPP_INFO(this->get_logger(), "avg_rtt %f avg_offset %f trim %d", filter_rtt->getAverage(), filter_offset->getAverage(), applied_rtc_trim);

			    RCLCPP_INFO(this->get_logger(), "t1 %ld t2 %ld t3 %ld t4 %ld t41 %ld t32 %ld t21 %ld t34 %ld",
					t1_point.count(), t2_point.count(), t3_point.count(), t4_point.count(),
					t41, t32, t21, t34);

				// filtered average delay values and standard deviations
				// RCLCPP_INFO(this->get_logger(), "avg_delay %f avg_raw %f std_dev_delay %f std_dev_raw %f", delay_avg, delay_raw, filter_delay->getStdDev(), filter_delay_raw->getStdDev());

				// SYNC REPLY REPORT END
		   }

           /*
           RCLCPP_INFO(this->get_logger(), "ax: %.4f, ay: %.4f, az: %.4f, wx: %.4f, wy: %.4f, wz: %.4f, mt: %.1fC, st: %.1fC",
                                            ax_bias, ay_bias, az_bias, wx_bias, wy_bias, wz_bias, mag_temp, sensor_temp);
           */

	     } // TODO: fix indent
      } else if (
        (bytes_transferred == 64) &													// packet size check
        (buffer[0] == DATA_PREFIX) &                                  			    // prefix check
        (buffer[USB_PACKET_SIZE - 1] == PACKET_POSTFIX)                   			// postfix check
      ) {

        // const auto received_marker = get_time();									// get packet received time
        const auto since_epoch = cb_mark.time_since_epoch();						// single epoch calculation with direct uint32_t conversion

		sync_state = 0;

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

	      sys_code = buffer[59];
          sys_status = buffer[61];

          if(read_state == -1) {
            RCLCPP_INFO(this->get_logger(), "FXIMU init read_state = -1");
            prev_packet_seq = buffer[60];								  // record incoming packet sequence number as previous
            read_state = 0;                                               // set read state to normal
            handle_sys_status(sys_status, sys_code);                      // handle sys_status even in first packet
			return;
          } else {
            packet_seq = buffer[60];								      // incoming packet sequence number
            uint8_t expected_seq = prev_packet_seq + 1;                   // calculate expected sequence number
            if(packet_seq != expected_seq) {
              RCLCPP_ERROR(this->get_logger(), "prev_packet_seq: %d, expected_seq: %d, skip: packet_seq: %d", prev_packet_seq, expected_seq, packet_seq);
			  // we receive error, so if state is 0, we increment it, and if it already 1 we reset state
              if(read_state == 0) { read_state = 1; } else if(read_state == 1) { read_state = -1; return; }
            } else {
              read_state = 0;
            }
            prev_packet_seq = packet_seq;								  // record sequence number as previous
            handle_sys_status(sys_status, sys_code);                      // handle sys_status
          }

          // notice: we can have read_state 0 or 1 at this point, read_state should not be used for anything else

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

          const uint32_t device_rtc_seconds = U4(buffer, 53);			      // RTC seconds
          const uint16_t device_rtc_ticks = U2(buffer, 57);			          // RTC ticks
          const uint32_t device_rtc_nanos = device_rtc_ticks * 30517.578125;  // RTC nanos
          timestamp device_timestamp {std::chrono::seconds{device_rtc_seconds}, std::chrono::nanoseconds{device_rtc_nanos}};

          const auto received_point = received_timestamp.first + received_timestamp.second;    // calculate nanos diff
          const auto device_point = device_timestamp.first + device_timestamp.second;
          const int32_t nanos_diff = (received_point - device_point).count();

		  if(abs(nanos_diff) > 900000000) {

		  		RCLCPP_ERROR(this->get_logger(), "threshold nanos_diff %d rtc %d.%d host %d.%d",
					nanos_diff,
					device_rtc_seconds,
					device_rtc_ticks,
					received_marker_sec,
					received_marker_ns);

				prev_device_rtc_ticks = 16384;				// this delays the sync cycle until next time

		    	// if nanos_diff exceed threshold for 3 times in a row reset the driver
				if(threshold_count >= 3) {
					this->reset_driver();
					return;
				}
				threshold_count = threshold_count + 1;

		  } else {

			threshold_count = 0;

          	// stamp for imu packet
          	// rclcpp::Time stamp = rclcpp::Clock().now();
            //  + rclcpp::Duration(0, device_rtc_nanos);
          	//rclcpp::Time stamp1 = rclcpp::Time(device_rtc_seconds, device_rtc_nanos);


			// TODO: delay corrected is spiky on rpi5. although rtt and offset are not.
			// TODO: here is where we make correction to timestamp based on offset.

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

            // raw nanos_diff filter
			filter_delay_raw->update(nanos_diff);

			// nanos_diff is received_time - device_rtc_time
			// filter_offset is rtc offset, + for device rtc leading
            // (received_point - device_point).count();
            // so it is (a - b) already. we need to substract it from offset, so we add to nanos_diff
			filter_delay->update((int32_t) (nanos_diff + filter_offset->getAverage()));

			// TODO: TRY using average phi here or even see if phi is outlier or not.
			//       if we are not updating filter with phi
			//       then we are not updating this above
			//       - how ever it still is not the cause of spikes on delay corrected
            //       - or it could be possible that this delay corrected filter is working kind of wrong.
			//       - because it definitively is there, this is probably happening because of the reset each time
			//       - it gets.

			// TODO: slim chance that correcting with instant offset instead of average is better
			// TODO: also notice in delay corrected, we are correcting by average_offset
            // TODO: when offset is small, maybe it is better to correct it with average offset, but
            // when offset is big, one better corect it with instant offset as seen
			// also offset filter is too soft.

          	// here we send sync request packet. this triggers mid second
          	if((prev_device_rtc_ticks < 16384) && (device_rtc_ticks >= 16384)) {

				/*
				// SYNC REQUEST START

				// sync request procedure starts here
                u32_to_ui8 u;
                i32_to_ui8 i;

				if(device_rtc_seconds % 64 == 62) {						     // at 62nd second
					if(filter_offset->isWarmedUp()) {						 // notice: even we are sending non filtered phi, we wait for filter warm up
						// i.i32 = (int32_t) phi; 							 // sends last measured offset
						i.i32 = filter_offset->getAverage(); // TODO: more soften,
														     // TODO: outlier rejection
															 // TODO: works in a period and resets
					}
				} else if(device_rtc_seconds % 64 == 63) {					 // skip sending packet at 63rd second
					return;
				} else {
					i.i32 = 0;
				}

				// TODO: could add outlier detection to T41.

                sync_packet[9] = i.ui8[0];							   // precalculated offset to gain time
                sync_packet[10] = i.ui8[1];
                sync_packet[11] = i.ui8[2];
                sync_packet[12] = i.ui8[3];

                auto t1_time = get_time();  					   	   // time marker
                t1_time += std::chrono::microseconds(0);               // add a propagation delay
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

				sync_state = 1; // TODO: enum SYNC_REQUEST_SEMT

				// SYNC REQUEST END

*/

          	} // end mid second interrupt

            // prev_device_rtc_tics is used for the mid-second interrupt
            prev_device_rtc_ticks = device_rtc_ticks;

		  } // end threshold check

        }

	  // TODO: rename DIAG_PREVIX and DIAG_*
      }
	} // end receive_callback

    void FximuNode::get_serial_parameters() {

      uint32_t baud_rate{};
      auto fc = FlowControl::HARDWARE; // TODO: was none
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
    } // end get serial parameters

	// TODO: make offset filter follow instant offset

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
    } // end declare parmeters

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
    } // end send parameters
  } // namespace fximu_driver
} // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::fximu_driver::FximuNode)