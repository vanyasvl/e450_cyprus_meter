#pragma once

namespace esphome {
namespace sensor {

class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual void publish_state(float state) { (void) state; }
};

}  // namespace sensor
}  // namespace esphome
