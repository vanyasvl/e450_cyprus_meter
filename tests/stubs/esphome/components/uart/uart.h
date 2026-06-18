#pragma once

#include <cstdint>
#include <vector>

namespace esphome {
namespace uart {

class UARTDevice {
 public:
  virtual ~UARTDevice() = default;

  int available() const { return static_cast<int>(this->rx_.size() - this->pos_); }

  bool read_byte(uint8_t *byte) {
    if (this->pos_ >= this->rx_.size())
      return false;
    *byte = this->rx_[this->pos_++];
    return true;
  }

  void push_bytes(const std::vector<uint8_t> &bytes) { this->rx_.insert(this->rx_.end(), bytes.begin(), bytes.end()); }

  void check_uart_settings(uint32_t baud_rate) const { (void) baud_rate; }

 protected:
  std::vector<uint8_t> rx_;
  size_t pos_{0};
};

}  // namespace uart
}  // namespace esphome
