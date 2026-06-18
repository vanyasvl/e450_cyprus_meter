#pragma once

#include <string>

namespace esphome {
namespace text_sensor {

class TextSensor {
 public:
  virtual ~TextSensor() = default;
  virtual void publish_state(const std::string &state) { (void) state; }
};

}  // namespace text_sensor
}  // namespace esphome
