#ifndef SERIAL_DRIVER__SERIAL_PORT_HPP_
#define SERIAL_DRIVER__SERIAL_PORT_HPP_

#include <array>
#include <string>
#include <vector>

#include "io_context/common.hpp"
#include "io_context/io_context.hpp"

using spb = asio::serial_port_base;
using drivers::common::IoContext;

namespace drivers
{
  namespace fximu_driver
  {

  using Functor = std::function<void (std::vector<uint8_t> &, const size_t &)>;

  enum class FlowControl {
    NONE,
    HARDWARE,
    SOFTWARE
  };

  enum class Parity {
    NONE,
    ODD,
    EVEN
  };

  enum class StopBits {
    ONE,
    ONE_POINT_FIVE,
    TWO
  };

  class SerialPortConfig
  {
    public:
      SerialPortConfig(
        uint32_t baud_rate,
        FlowControl flow_control,
        Parity parity,
        StopBits stop_bits)
      : m_baud_rate{baud_rate},
        m_flow_control{flow_control},
        m_parity{parity},
        m_stop_bits{stop_bits}
      {
      }

      uint32_t get_baud_rate() const
      {
        return m_baud_rate;
      }

      spb::baud_rate get_baud_rate_asio() const
      {
        return spb::baud_rate{m_baud_rate};
      }

      FlowControl get_flow_control() const
      {
        return m_flow_control;
      }

      spb::flow_control::type get_flow_control_asio() const
      {
        switch (m_flow_control) {
          case FlowControl::HARDWARE:
            return spb::flow_control::hardware;
            break;
          case FlowControl::SOFTWARE:
            return spb::flow_control::software;
            break;
          case FlowControl::NONE:
          default:
            return spb::flow_control::none;
        }
      }

      Parity get_parity() const {
        return m_parity;
      }

      spb::parity::type get_parity_asio() const {
        switch (m_parity) {
          case Parity::ODD:
            return spb::parity::odd;
            break;
          case Parity::EVEN:
            return spb::parity::even;
            break;
          case Parity::NONE:
          default:
            return spb::parity::none;
        }
      }

      StopBits get_stop_bits() const {
        return m_stop_bits;
      }

      spb::stop_bits::type get_stop_bits_asio() const {
        switch (m_stop_bits) {
          case StopBits::ONE_POINT_FIVE:
            return spb::stop_bits::onepointfive;
            break;
          case StopBits::TWO:
            return spb::stop_bits::two;
            break;
          case StopBits::ONE:
          default:
            return spb::stop_bits::one;
        }
      }

    private:
      uint32_t m_baud_rate;
      FlowControl m_flow_control;
      Parity m_parity;
      StopBits m_stop_bits;
  };

  class SerialPort {

    public:

      SerialPort(
        const IoContext & ctx,
        const std::string & device_name,
        const SerialPortConfig serial_port_config);
      ~SerialPort();

      SerialPort(const SerialPort &) = delete;
      SerialPort & operator=(const SerialPort &) = delete;

      std::string device_name() const;

      SerialPortConfig serial_port_config() const;

      void open();
      void close();
      bool is_open() const;

      size_t send(const std::vector<uint8_t> & buff);
      size_t receive(std::vector<uint8_t> & buff);
      void async_send(const std::vector<uint8_t> & buff);
      void async_receive(Functor func);
      bool send_break();

    private:

      void async_send_handler(const asio::error_code & error, size_t bytes_transferred);

      void async_receive_handler(const asio::error_code & error, size_t bytes_transferred);

      const IoContext & m_ctx;
      std::string m_device_name;
      asio::serial_port m_serial_port;
      SerialPortConfig m_port_config;
      Functor m_func;
      
      static constexpr size_t m_recv_buffer_size{2048};
      std::vector<uint8_t> m_recv_buffer;

  };

  }
}

#endif