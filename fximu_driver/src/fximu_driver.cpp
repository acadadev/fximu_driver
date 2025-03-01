#include "fximu_driver/fximu_driver.hpp"

#include <memory>
#include <string>

// TODO: MASTER LIST
// TODO: test_serial_driver.cpp, test_serial_port.cpp
// TODO: windows compatibility
// TODO: github workflow
// TODO: turn all to cpp

namespace drivers
{
  namespace fximu_driver
  {

    FximuDriver::FximuDriver(const IoContext & ctx) : m_ctx(ctx) {}

    void FximuDriver::init_port(const std::string & device_name, const SerialPortConfig & config) {
      m_port.reset(new SerialPort(m_ctx, device_name, config));
    }

    std::shared_ptr<SerialPort> FximuDriver::port() const {
      return m_port;
    }

  }
}