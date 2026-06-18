#include "e450_cyprus_meter.h"

#include <algorithm>
#include <cstdio>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace e450_cyprus_meter {

static const char *const TAG = "e450_cyprus_meter";

static const uint8_t T_NULL = 0x00;
static const uint8_t T_ARRAY = 0x01;
static const uint8_t T_STRUCT = 0x02;
static const uint8_t T_INT32 = 0x05;
static const uint8_t T_UINT32 = 0x06;
static const uint8_t T_OCTSTR = 0x09;
static const uint8_t T_VISSTR = 0x0A;
static const uint8_t T_INT8 = 0x0F;
static const uint8_t T_INT16 = 0x10;
static const uint8_t T_UINT8 = 0x11;
static const uint8_t T_UINT16 = 0x12;
static const uint8_t T_ENUM = 0x16;
static const uint8_t T_DATETIME = 0x19;

void E450CyprusMeterComponent::setup() {
  this->rx_buffer_.reserve(2048);
}

void E450CyprusMeterComponent::loop() {
  this->process_uart_();
  if (!this->gbt_blocks_.empty() && millis() - this->last_gbt_ms_ > this->frame_timeout_ms_) {
    this->finalize_gbt_("timeout");
  }
}

void E450CyprusMeterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Landis+Gyr E450 M-Bus CII");
  ESP_LOGCONFIG(TAG, "  Frame timeout: %u ms", this->frame_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  CRC validation: %s", TRUEFALSE(this->validate_crc_));
  this->check_uart_settings(2400);
}

bool E450CyprusMeterComponent::Reader::u8(uint8_t *value) {
  if (this->pos_ >= this->data_.size())
    return false;
  *value = this->data_[this->pos_++];
  return true;
}

bool E450CyprusMeterComponent::Reader::take(size_t count, std::vector<uint8_t> *out) {
  if (count > this->remaining())
    return false;
  out->assign(this->data_.begin() + this->pos_, this->data_.begin() + this->pos_ + count);
  this->pos_ += count;
  return true;
}

bool E450CyprusMeterComponent::Reader::length(size_t *value) {
  uint8_t first;
  if (!this->u8(&first))
    return false;
  if ((first & 0x80) == 0) {
    *value = first;
    return true;
  }
  uint8_t count = first & 0x7F;
  if (count == 0 || count > sizeof(size_t) || count > this->remaining())
    return false;
  size_t length = 0;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t part;
    if (!this->u8(&part))
      return false;
    length = (length << 8) | part;
  }
  *value = length;
  return true;
}

void E450CyprusMeterComponent::process_uart_() {
  uint8_t byte;
  std::vector<uint8_t> frame;
  while (this->available()) {
    if (!this->read_byte(&byte))
      break;
    this->rx_buffer_.push_back(byte);
    while (this->extract_frame_(&frame))
      this->process_frame_(frame);
    if (this->rx_buffer_.size() > 4096) {
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.end() - 2048);
      ESP_LOGW(TAG, "UART buffer overflow; dropped old bytes");
    }
  }

  while (this->extract_frame_(&frame))
    this->process_frame_(frame);
}

bool E450CyprusMeterComponent::extract_frame_(std::vector<uint8_t> *frame) {
  while (true) {
    auto start = std::find(this->rx_buffer_.begin(), this->rx_buffer_.end(), 0x7E);
    if (start == this->rx_buffer_.end()) {
      this->rx_buffer_.clear();
      return false;
    }
    if (start != this->rx_buffer_.begin())
      this->rx_buffer_.erase(this->rx_buffer_.begin(), start);
    if (this->rx_buffer_.size() >= 2 && this->rx_buffer_[1] == 0x7E) {
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }
    break;
  }

  if (this->rx_buffer_.size() < 3)
    return false;

  size_t declared = ((this->rx_buffer_[1] & 0x07) << 8) | this->rx_buffer_[2];
  size_t total = declared + 2;
  if (declared < 8) {
    this->rx_buffer_.erase(this->rx_buffer_.begin());
    return false;
  }
  if (this->rx_buffer_.size() < total)
    return false;
  if (this->rx_buffer_[total - 1] != 0x7E) {
    this->rx_buffer_.erase(this->rx_buffer_.begin());
    return false;
  }

  frame->assign(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total);
  this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total);
  return true;
}

void E450CyprusMeterComponent::process_frame_(const std::vector<uint8_t> &frame) {
  HdlcInfo parsed;
  if (!this->parse_hdlc_(frame, &parsed)) {
    ESP_LOGW(TAG, "Skipped invalid HDLC frame");
    return;
  }
  this->hdlc_frames_consumed_++;

  GbtBlock block;
  if (!this->parse_gbt_(parsed.info, &block)) {
    this->finalize_gbt_("non-GBT frame");
    return;
  }
  this->handle_gbt_block_(block);
}

bool E450CyprusMeterComponent::parse_hdlc_(const std::vector<uint8_t> &frame, HdlcInfo *parsed) {
  if (frame.size() < 10 || frame.front() != 0x7E || frame.back() != 0x7E)
    return false;

  const uint8_t *body = frame.data() + 1;
  size_t body_len = frame.size() - 2;
  if (this->validate_crc_ && this->crc16_hdlc_(body, body_len) != 0xF0B8)
    return false;

  size_t i = 2;
  while (i < body_len && (body[i] & 1) == 0)
    i++;
  if (i >= body_len)
    return false;
  i++;
  while (i < body_len && (body[i] & 1) == 0)
    i++;
  if (i >= body_len)
    return false;
  i++;
  if (i >= body_len)
    return false;
  parsed->control = body[i++];
  if (body_len - i < 4)
    return false;

  size_t info_start = i + 2;  // skip HCS
  size_t info_end = body_len - 2;
  if (info_start > info_end)
    return false;
  parsed->info.assign(body + info_start, body + info_end);
  return true;
}

bool E450CyprusMeterComponent::parse_gbt_(const std::vector<uint8_t> &info, GbtBlock *block) {
  size_t p = 0;
  if (info.size() >= 3 && info[0] == 0xE6 && info[1] == 0xE7 && info[2] == 0x00)
    p = 3;
  if (p >= info.size() || info[p] != 0xE0)
    return false;
  p++;
  if (info.size() - p < 6)
    return false;

  uint8_t control = info[p++];
  block->last = (control & 0x80) != 0;
  block->number = (uint16_t(info[p]) << 8) | info[p + 1];
  p += 2;
  p += 2;  // ack number

  std::vector<uint8_t> length_data(info.begin() + p, info.end());
  Reader reader(length_data);
  size_t length = 0;
  if (!reader.length(&length))
    return false;
  p += reader.position();
  if (info.size() - p < length)
    return false;
  block->content.assign(info.begin() + p, info.begin() + p + length);
  return true;
}

void E450CyprusMeterComponent::handle_gbt_block_(const GbtBlock &block) {
  if (this->gbt_blocks_.empty() && block.number != 1) {
    ESP_LOGW(TAG, "Ignored orphan GBT block %u", block.number);
    return;
  }

  if (!this->gbt_blocks_.empty()) {
    auto existing = this->gbt_blocks_.find(block.number);
    if (existing != this->gbt_blocks_.end() && existing->second == block.content) {
      ESP_LOGW(TAG, "Ignored duplicate GBT block %u", block.number);
      this->last_gbt_ms_ = millis();
      return;
    }
  }

  if (block.number == 1 && !this->gbt_blocks_.empty()) {
    this->finalize_gbt_("new block 1");
  } else if (!this->gbt_blocks_.empty()) {
    uint16_t expected = this->gbt_blocks_.rbegin()->first + 1;
    if (block.number != expected)
      this->finalize_gbt_("sequence break");
  }

  if (this->gbt_blocks_.empty() && block.number != 1) {
    ESP_LOGW(TAG, "Ignored orphan GBT block %u", block.number);
    return;
  }

  this->gbt_blocks_[block.number] = block.content;
  this->gbt_saw_last_ = this->gbt_saw_last_ || block.last;
  this->last_gbt_ms_ = millis();
  if (block.last)
    this->finalize_gbt_("last block");
}

void E450CyprusMeterComponent::finalize_gbt_(const char *reason) {
  (void) reason;
  if (this->gbt_blocks_.empty())
    return;

  uint16_t expected = 1;
  for (const auto &item : this->gbt_blocks_) {
    if (item.first != expected) {
      this->skipped_groups_++;
      ESP_LOGW(TAG, "Skipped incomplete GBT group at %s", reason);
      this->clear_gbt_();
      return;
    }
    expected++;
  }
  if (!this->gbt_saw_last_) {
    this->skipped_groups_++;
    ESP_LOGW(TAG, "Skipped GBT group without last block at %s", reason);
    this->clear_gbt_();
    return;
  }

  std::vector<uint8_t> apdu;
  for (const auto &item : this->gbt_blocks_)
    apdu.insert(apdu.end(), item.second.begin(), item.second.end());

  std::vector<std::string> obis_list;
  std::vector<DecodedValue> values;
  std::string timestamp;
  if (!this->parse_notification_(apdu, &obis_list, &values, &timestamp)) {
    this->skipped_groups_++;
    ESP_LOGW(TAG, "Skipped unparsable DLMS notification");
    this->clear_gbt_();
    return;
  }

  this->valid_pushes_++;
  this->publish_decoded_(obis_list, values, timestamp);
  this->clear_gbt_();
}

void E450CyprusMeterComponent::clear_gbt_() {
  this->gbt_blocks_.clear();
  this->gbt_saw_last_ = false;
}

bool E450CyprusMeterComponent::parse_notification_(const std::vector<uint8_t> &apdu, std::vector<std::string> *obis_list,
                                            std::vector<DecodedValue> *values, std::string *timestamp) {
  if (apdu.empty() || apdu[0] != 0x0F)
    return false;

  Reader reader(apdu);
  uint8_t tag;
  if (!reader.u8(&tag))
    return false;
  std::vector<uint8_t> ignored;
  if (!reader.take(4, &ignored))
    return false;
  uint8_t dt_len;
  if (!reader.u8(&dt_len))
    return false;
  std::vector<uint8_t> dt;
  if (dt_len != 0) {
    if (!reader.take(dt_len, &dt))
      return false;
    *timestamp = this->format_datetime_(dt);
  }

  std::vector<uint8_t> body;
  if (!reader.take(reader.remaining(), &body))
    return false;
  Reader body_reader(body);
  AxdrNode root;
  if (!this->read_axdr_value_(&body_reader, &root))
    return false;
  if (!this->walk_axdr_(root, obis_list, values))
    return false;
  return !obis_list->empty() && values->size() <= obis_list->size();
}

bool E450CyprusMeterComponent::read_axdr_value_(Reader *reader, AxdrNode *node) {
  uint8_t tag;
  if (!reader->u8(&tag))
    return false;

  if (tag == T_NULL) {
    node->type = AxdrNode::NIL;
    return true;
  }
  if (tag == T_ARRAY || tag == T_STRUCT) {
    size_t count = 0;
    if (!reader->length(&count))
      return false;
    node->type = AxdrNode::LIST;
    node->children.reserve(count);
    for (size_t i = 0; i < count; i++) {
      AxdrNode child;
      if (!this->read_axdr_value_(reader, &child))
        return false;
      node->children.push_back(child);
    }
    return true;
  }
  if (tag == T_OCTSTR || tag == T_VISSTR) {
    size_t length = 0;
    if (!reader->length(&length))
      return false;
    node->type = AxdrNode::BYTES;
    return reader->take(length, &node->bytes);
  }
  if (tag == T_DATETIME) {
    node->type = AxdrNode::BYTES;
    return reader->take(12, &node->bytes);
  }

  std::vector<uint8_t> bytes;
  node->type = AxdrNode::INT;
  switch (tag) {
    case T_UINT8: {
      uint8_t v;
      if (!reader->u8(&v))
        return false;
      node->integer = v;
      return true;
    }
    case T_INT8: {
      uint8_t v;
      if (!reader->u8(&v))
        return false;
      node->integer = v > 127 ? int(v) - 256 : v;
      return true;
    }
    case T_UINT16:
      if (!reader->take(2, &bytes))
        return false;
      node->integer = this->read_be16_(bytes, 0);
      return true;
    case T_INT16:
      if (!reader->take(2, &bytes))
        return false;
      node->integer = int16_t(this->read_be16_(bytes, 0));
      return true;
    case T_UINT32:
      if (!reader->take(4, &bytes))
        return false;
      node->integer = this->read_be32_(bytes, 0);
      return true;
    case T_INT32:
      if (!reader->take(4, &bytes))
        return false;
      node->integer = int32_t(this->read_be32_(bytes, 0));
      return true;
    case T_ENUM: {
      uint8_t v;
      if (!reader->u8(&v))
        return false;
      node->integer = v;
      return true;
    }
    default:
      ESP_LOGW(TAG, "Unsupported A-XDR tag 0x%02X at offset %u", tag, unsigned(reader->position()));
      return false;
  }
}

bool E450CyprusMeterComponent::walk_axdr_(const AxdrNode &node, std::vector<std::string> *obis_list,
                                   std::vector<DecodedValue> *values) {
  if (node.type == AxdrNode::LIST) {
    if (node.children.size() == 4 && node.children[0].type == AxdrNode::INT &&
        node.children[1].type == AxdrNode::BYTES && node.children[1].bytes.size() == 6) {
      obis_list->push_back(this->obis_to_string_(node.children[1].bytes));
      return true;
    }
    for (const auto &child : node.children) {
      if (!this->walk_axdr_(child, obis_list, values))
        return false;
    }
    return true;
  }
  if (node.type == AxdrNode::INT) {
    DecodedValue value;
    value.type = DecodedValue::NUMBER;
    value.number = node.integer;
    values->push_back(value);
  } else if (node.type == AxdrNode::BYTES) {
    DecodedValue value;
    value.type = DecodedValue::TEXT;
    if (node.bytes.size() == 12)
      value.text = this->format_datetime_(node.bytes);
    else
      value.text = this->bytes_to_ascii_(node.bytes);
    values->push_back(value);
  }
  return true;
}

void E450CyprusMeterComponent::publish_decoded_(const std::vector<std::string> &obis_list,
                                         const std::vector<DecodedValue> &values,
                                         const std::string &timestamp) {
  size_t offset = obis_list.size() - values.size();
  for (size_t i = 0; i < offset; i++)
    this->publish_text_(obis_list[i], timestamp, true);

  for (size_t i = 0; i < values.size(); i++) {
    const std::string &obis = obis_list[offset + i];
    const DecodedValue &value = values[i];
    if (value.type == DecodedValue::NUMBER)
      this->publish_numeric_(obis, this->scale_value_(obis, value.number));
    else
      this->publish_text_(obis, value.text, false);
  }
}

void E450CyprusMeterComponent::publish_numeric_(const std::string &obis, double value) {
  sensor::Sensor *target = nullptr;
  if (obis == "1.0.21.7.0.255")
    target = this->active_power_import_l1_sensor_;
  else if (obis == "1.0.41.7.0.255")
    target = this->active_power_import_l2_sensor_;
  else if (obis == "1.0.61.7.0.255")
    target = this->active_power_import_l3_sensor_;
  else if (obis == "1.0.22.7.0.255")
    target = this->active_power_export_l1_sensor_;
  else if (obis == "1.0.42.7.0.255")
    target = this->active_power_export_l2_sensor_;
  else if (obis == "1.0.62.7.0.255")
    target = this->active_power_export_l3_sensor_;
  else if (obis == "1.0.1.7.0.255")
    target = this->active_power_import_total_sensor_;
  else if (obis == "1.0.2.7.0.255")
    target = this->active_power_export_total_sensor_;
  else if (obis == "1.0.3.7.0.255")
    target = this->reactive_power_import_sensor_;
  else if (obis == "1.0.4.7.0.255")
    target = this->reactive_power_export_sensor_;
  else if (obis == "1.0.13.7.0.255")
    target = this->power_factor_sensor_;
  else if (obis == "1.0.31.7.0.255")
    target = this->current_l1_sensor_;
  else if (obis == "1.0.51.7.0.255")
    target = this->current_l2_sensor_;
  else if (obis == "1.0.71.7.0.255")
    target = this->current_l3_sensor_;
  else if (obis == "1.0.32.7.0.255")
    target = this->voltage_l1_sensor_;
  else if (obis == "1.0.52.7.0.255")
    target = this->voltage_l2_sensor_;
  else if (obis == "1.0.72.7.0.255")
    target = this->voltage_l3_sensor_;
  else if (obis == "1.0.14.7.0.255")
    target = this->frequency_sensor_;
  else if (obis == "0.0.96.3.0.255")
    target = this->disconnect_control_state_sensor_;
  else if (obis == "1.0.1.8.1.255")
    target = this->active_energy_import_t1_sensor_;
  else if (obis == "1.0.1.8.2.255")
    target = this->active_energy_import_t2_sensor_;
  else if (obis == "1.0.2.8.1.255")
    target = this->active_energy_export_t1_sensor_;
  else if (obis == "1.0.2.8.2.255")
    target = this->active_energy_export_t2_sensor_;
  else if (obis == "1.0.1.8.0.255")
    target = this->active_energy_import_total_sensor_;
  else if (obis == "1.0.2.8.0.255")
    target = this->active_energy_export_total_sensor_;
  else if (obis == "1.0.3.8.0.255")
    target = this->reactive_energy_import_sensor_;
  else if (obis == "1.0.4.8.0.255")
    target = this->reactive_energy_export_sensor_;
  else if (obis == "0.0.96.3.10.255")
    target = this->disconnect_limiter_state_sensor_;

  if (target != nullptr)
    target->publish_state(value);
}

void E450CyprusMeterComponent::publish_text_(const std::string &obis, const std::string &value, bool leading_timestamp) {
  (void) leading_timestamp;
  text_sensor::TextSensor *target = nullptr;
  if (obis == "0.8.25.9.0.255")
    target = this->push_timestamp_text_sensor_;
  else if (obis == "0.9.25.9.0.255")
    target = this->energy_push_timestamp_text_sensor_;
  else if (obis == "0.0.1.0.0.255")
    target = this->meter_clock_text_sensor_;
  else if (obis == "0.0.42.0.0.255")
    target = this->meter_logical_device_name_text_sensor_;
  else if (obis == "0.0.96.1.1.255")
    target = this->meter_serial_number_text_sensor_;
  else if (obis == "0.0.96.14.0.255")
    target = this->active_tariff_text_sensor_;

  if (target != nullptr)
    target->publish_state(value);
}

double E450CyprusMeterComponent::scale_value_(const std::string &obis, double value) const {
  if (obis == "1.0.31.7.0.255" || obis == "1.0.51.7.0.255" || obis == "1.0.71.7.0.255")
    return value * 0.01;
  if (obis == "1.0.13.7.0.255")
    return value * 0.001;
  return value;
}

uint16_t E450CyprusMeterComponent::crc16_hdlc_(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return crc;
}

std::string E450CyprusMeterComponent::obis_to_string_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() != 6)
    return "";
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%u.%u.%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
                bytes[5]);
  return buffer;
}

std::string E450CyprusMeterComponent::bytes_to_ascii_(const std::vector<uint8_t> &bytes) {
  size_t end = bytes.size();
  while (end > 0 && bytes[end - 1] == 0)
    end--;
  std::string out;
  out.reserve(end);
  for (size_t i = 0; i < end; i++) {
    uint8_t c = bytes[i];
    out.push_back(c >= 32 && c <= 126 ? char(c) : '?');
  }
  return out;
}

std::string E450CyprusMeterComponent::format_datetime_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() != 12)
    return bytes_to_ascii_(bytes);
  uint16_t year = (uint16_t(bytes[0]) << 8) | bytes[1];
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u:%02u", year, bytes[2], bytes[3], bytes[5],
                bytes[6], bytes[7]);
  return buffer;
}

uint16_t E450CyprusMeterComponent::read_be16_(const std::vector<uint8_t> &data, size_t offset) {
  return (uint16_t(data[offset]) << 8) | data[offset + 1];
}

uint32_t E450CyprusMeterComponent::read_be32_(const std::vector<uint8_t> &data, size_t offset) {
  return (uint32_t(data[offset]) << 24) | (uint32_t(data[offset + 1]) << 16) | (uint32_t(data[offset + 2]) << 8) |
         data[offset + 3];
}

}  // namespace e450_cyprus_meter
}  // namespace esphome
