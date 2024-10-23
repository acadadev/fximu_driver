#include <gtest/gtest.h>

#include <string>

#include "fximu_driver/fximu_driver.hpp"

using drivers::fximu_driver::FlowControl;
using drivers::fximu_driver::Parity;
using drivers::fximu_driver::FximuDriver;
using drivers::fximu_driver::SerialPortConfig;
using drivers::fximu_driver::StopBits;

static constexpr const char * dev_name = "/dev/ttyACM0";
static constexpr uint32_t baud = 115200;
static constexpr FlowControl fc = FlowControl::NONE;
static constexpr Parity pt = Parity::NONE;
static constexpr StopBits sb = StopBits::ONE;

TEST(SerialDriverTest, PropertiesTest)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  FximuDriver driver(ctx);

  EXPECT_EQ(driver.port().get(), nullptr);

  driver.init_port(dev_name, config);

  EXPECT_EQ(driver.port()->device_name(), dev_name);

  ctx.waitForExit();
}
