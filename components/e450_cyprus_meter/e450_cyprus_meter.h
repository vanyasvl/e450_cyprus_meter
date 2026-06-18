#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace e450_cyprus_meter {

class E450CyprusMeterComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_frame_timeout(uint32_t timeout_ms) { this->frame_timeout_ms_ = timeout_ms; }
  void set_validate_crc(bool validate_crc) { this->validate_crc_ = validate_crc; }
  uint32_t get_hdlc_frames_consumed() const { return this->hdlc_frames_consumed_; }
  uint32_t get_valid_pushes() const { return this->valid_pushes_; }
  uint32_t get_skipped_groups() const { return this->skipped_groups_; }

  void set_active_power_import_l1_sensor(sensor::Sensor *sensor) { this->active_power_import_l1_sensor_ = sensor; }
  void set_active_power_import_l2_sensor(sensor::Sensor *sensor) { this->active_power_import_l2_sensor_ = sensor; }
  void set_active_power_import_l3_sensor(sensor::Sensor *sensor) { this->active_power_import_l3_sensor_ = sensor; }
  void set_active_power_export_l1_sensor(sensor::Sensor *sensor) { this->active_power_export_l1_sensor_ = sensor; }
  void set_active_power_export_l2_sensor(sensor::Sensor *sensor) { this->active_power_export_l2_sensor_ = sensor; }
  void set_active_power_export_l3_sensor(sensor::Sensor *sensor) { this->active_power_export_l3_sensor_ = sensor; }
  void set_active_power_import_total_sensor(sensor::Sensor *sensor) { this->active_power_import_total_sensor_ = sensor; }
  void set_active_power_export_total_sensor(sensor::Sensor *sensor) { this->active_power_export_total_sensor_ = sensor; }
  void set_reactive_power_import_sensor(sensor::Sensor *sensor) { this->reactive_power_import_sensor_ = sensor; }
  void set_reactive_power_export_sensor(sensor::Sensor *sensor) { this->reactive_power_export_sensor_ = sensor; }
  void set_power_factor_sensor(sensor::Sensor *sensor) { this->power_factor_sensor_ = sensor; }
  void set_current_l1_sensor(sensor::Sensor *sensor) { this->current_l1_sensor_ = sensor; }
  void set_current_l2_sensor(sensor::Sensor *sensor) { this->current_l2_sensor_ = sensor; }
  void set_current_l3_sensor(sensor::Sensor *sensor) { this->current_l3_sensor_ = sensor; }
  void set_voltage_l1_sensor(sensor::Sensor *sensor) { this->voltage_l1_sensor_ = sensor; }
  void set_voltage_l2_sensor(sensor::Sensor *sensor) { this->voltage_l2_sensor_ = sensor; }
  void set_voltage_l3_sensor(sensor::Sensor *sensor) { this->voltage_l3_sensor_ = sensor; }
  void set_frequency_sensor(sensor::Sensor *sensor) { this->frequency_sensor_ = sensor; }
  void set_disconnect_control_state_sensor(sensor::Sensor *sensor) { this->disconnect_control_state_sensor_ = sensor; }
  void set_active_energy_import_t1_sensor(sensor::Sensor *sensor) { this->active_energy_import_t1_sensor_ = sensor; }
  void set_active_energy_import_t2_sensor(sensor::Sensor *sensor) { this->active_energy_import_t2_sensor_ = sensor; }
  void set_active_energy_export_t1_sensor(sensor::Sensor *sensor) { this->active_energy_export_t1_sensor_ = sensor; }
  void set_active_energy_export_t2_sensor(sensor::Sensor *sensor) { this->active_energy_export_t2_sensor_ = sensor; }
  void set_active_energy_import_total_sensor(sensor::Sensor *sensor) { this->active_energy_import_total_sensor_ = sensor; }
  void set_active_energy_export_total_sensor(sensor::Sensor *sensor) { this->active_energy_export_total_sensor_ = sensor; }
  void set_reactive_energy_import_sensor(sensor::Sensor *sensor) { this->reactive_energy_import_sensor_ = sensor; }
  void set_reactive_energy_export_sensor(sensor::Sensor *sensor) { this->reactive_energy_export_sensor_ = sensor; }
  void set_disconnect_limiter_state_sensor(sensor::Sensor *sensor) { this->disconnect_limiter_state_sensor_ = sensor; }

  void set_push_timestamp_text_sensor(text_sensor::TextSensor *sensor) { this->push_timestamp_text_sensor_ = sensor; }
  void set_meter_clock_text_sensor(text_sensor::TextSensor *sensor) { this->meter_clock_text_sensor_ = sensor; }
  void set_energy_push_timestamp_text_sensor(text_sensor::TextSensor *sensor) { this->energy_push_timestamp_text_sensor_ = sensor; }
  void set_meter_logical_device_name_text_sensor(text_sensor::TextSensor *sensor) {
    this->meter_logical_device_name_text_sensor_ = sensor;
  }
  void set_meter_serial_number_text_sensor(text_sensor::TextSensor *sensor) { this->meter_serial_number_text_sensor_ = sensor; }
  void set_active_tariff_text_sensor(text_sensor::TextSensor *sensor) { this->active_tariff_text_sensor_ = sensor; }

 protected:
  struct HdlcInfo {
    uint8_t control{0};
    std::vector<uint8_t> info;
  };

  struct GbtBlock {
    uint16_t number{0};
    bool last{false};
    std::vector<uint8_t> content;
  };

  struct AxdrNode {
    enum Type { NIL, LIST, INT, BYTES } type{NIL};
    std::vector<AxdrNode> children;
    std::vector<uint8_t> bytes;
    int64_t integer{0};
  };

  struct DecodedValue {
    enum Type { NUMBER, TEXT } type{NUMBER};
    double number{0};
    std::string text;
  };

  class Reader {
   public:
    explicit Reader(const std::vector<uint8_t> &data) : data_(data) {}
    bool u8(uint8_t *value);
    bool take(size_t count, std::vector<uint8_t> *out);
    bool length(size_t *value);
    size_t position() const { return this->pos_; }
    size_t remaining() const { return this->pos_ <= this->data_.size() ? this->data_.size() - this->pos_ : 0; }

   protected:
    const std::vector<uint8_t> &data_;
    size_t pos_{0};
  };

  void process_uart_();
  bool extract_frame_(std::vector<uint8_t> *frame);
  void process_frame_(const std::vector<uint8_t> &frame);
  bool parse_hdlc_(const std::vector<uint8_t> &frame, HdlcInfo *parsed);
  bool parse_gbt_(const std::vector<uint8_t> &info, GbtBlock *block);
  void handle_gbt_block_(const GbtBlock &block);
  void finalize_gbt_(const char *reason);
  void clear_gbt_();
  bool parse_notification_(const std::vector<uint8_t> &apdu, std::vector<std::string> *obis_list,
                           std::vector<DecodedValue> *values, std::string *timestamp);
  bool read_axdr_value_(Reader *reader, AxdrNode *node);
  bool walk_axdr_(const AxdrNode &node, std::vector<std::string> *obis_list, std::vector<DecodedValue> *values);
  void publish_decoded_(const std::vector<std::string> &obis_list, const std::vector<DecodedValue> &values,
                        const std::string &timestamp);
  void publish_numeric_(const std::string &obis, double value);
  void publish_text_(const std::string &obis, const std::string &value, bool leading_timestamp);
  double scale_value_(const std::string &obis, double value) const;

  static uint16_t crc16_hdlc_(const uint8_t *data, size_t len);
  static std::string obis_to_string_(const std::vector<uint8_t> &bytes);
  static std::string bytes_to_ascii_(const std::vector<uint8_t> &bytes);
  static std::string format_datetime_(const std::vector<uint8_t> &bytes);
  static uint16_t read_be16_(const std::vector<uint8_t> &data, size_t offset);
  static uint32_t read_be32_(const std::vector<uint8_t> &data, size_t offset);

  uint32_t frame_timeout_ms_{1500};
  bool validate_crc_{true};
  std::vector<uint8_t> rx_buffer_;
  std::map<uint16_t, std::vector<uint8_t>> gbt_blocks_;
  bool gbt_saw_last_{false};
  uint32_t last_gbt_ms_{0};
  uint32_t hdlc_frames_consumed_{0};
  uint32_t valid_pushes_{0};
  uint32_t skipped_groups_{0};

  sensor::Sensor *active_power_import_l1_sensor_{nullptr};
  sensor::Sensor *active_power_import_l2_sensor_{nullptr};
  sensor::Sensor *active_power_import_l3_sensor_{nullptr};
  sensor::Sensor *active_power_export_l1_sensor_{nullptr};
  sensor::Sensor *active_power_export_l2_sensor_{nullptr};
  sensor::Sensor *active_power_export_l3_sensor_{nullptr};
  sensor::Sensor *active_power_import_total_sensor_{nullptr};
  sensor::Sensor *active_power_export_total_sensor_{nullptr};
  sensor::Sensor *reactive_power_import_sensor_{nullptr};
  sensor::Sensor *reactive_power_export_sensor_{nullptr};
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *current_l1_sensor_{nullptr};
  sensor::Sensor *current_l2_sensor_{nullptr};
  sensor::Sensor *current_l3_sensor_{nullptr};
  sensor::Sensor *voltage_l1_sensor_{nullptr};
  sensor::Sensor *voltage_l2_sensor_{nullptr};
  sensor::Sensor *voltage_l3_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *disconnect_control_state_sensor_{nullptr};
  sensor::Sensor *active_energy_import_t1_sensor_{nullptr};
  sensor::Sensor *active_energy_import_t2_sensor_{nullptr};
  sensor::Sensor *active_energy_export_t1_sensor_{nullptr};
  sensor::Sensor *active_energy_export_t2_sensor_{nullptr};
  sensor::Sensor *active_energy_import_total_sensor_{nullptr};
  sensor::Sensor *active_energy_export_total_sensor_{nullptr};
  sensor::Sensor *reactive_energy_import_sensor_{nullptr};
  sensor::Sensor *reactive_energy_export_sensor_{nullptr};
  sensor::Sensor *disconnect_limiter_state_sensor_{nullptr};

  text_sensor::TextSensor *push_timestamp_text_sensor_{nullptr};
  text_sensor::TextSensor *meter_clock_text_sensor_{nullptr};
  text_sensor::TextSensor *energy_push_timestamp_text_sensor_{nullptr};
  text_sensor::TextSensor *meter_logical_device_name_text_sensor_{nullptr};
  text_sensor::TextSensor *meter_serial_number_text_sensor_{nullptr};
  text_sensor::TextSensor *active_tariff_text_sensor_{nullptr};
};

}  // namespace e450_cyprus_meter
}  // namespace esphome
