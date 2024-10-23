#ifndef FXIMU_DRIVER__FXIMU_DRIVER_HPP_
#define FXIMU_DRIVER__FXIMU_DRIVER_HPP_

#include <memory>
#include <string>

#include "io_context/io_context.hpp"
#include "fximu_driver/serial_port.hpp"

namespace drivers
{
  namespace fximu_driver
  {

    class FximuDriver
    {
      public:
        explicit FximuDriver(const IoContext & ctx);
        void init_port(const std::string & device_name, const SerialPortConfig & config);
        std::shared_ptr<SerialPort> port() const;

      private:
        const IoContext & m_ctx;
        std::shared_ptr<SerialPort> m_port;
    };

  }
}

#endif