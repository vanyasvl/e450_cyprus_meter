#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "components/e450_cyprus_meter/e450_cyprus_meter.h"

namespace esphome {

static uint32_t fake_millis = 0;

uint32_t millis() { return fake_millis; }

}  // namespace esphome

namespace {

struct Event {
  std::string name;
  std::string value;
};

std::string format_number(float value, int decimals) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(decimals) << value;
  return out.str();
}

class CaptureSensor : public esphome::sensor::Sensor {
 public:
  CaptureSensor(std::vector<Event> *events, std::string name, int decimals)
      : events_(events), name_(std::move(name)), decimals_(decimals) {}

  void publish_state(float state) override { this->events_->push_back({this->name_, format_number(state, this->decimals_)}); }

 private:
  std::vector<Event> *events_;
  std::string name_;
  int decimals_;
};

class CaptureTextSensor : public esphome::text_sensor::TextSensor {
 public:
  CaptureTextSensor(std::vector<Event> *events, std::string name) : events_(events), name_(std::move(name)) {}

  void publish_state(const std::string &state) override { this->events_->push_back({this->name_, state}); }

 private:
  std::vector<Event> *events_;
  std::string name_;
};

std::string redacted_value(const Event &event) {
  if (event.name == "meter_serial_number" || event.name == "meter_logical_device_name")
    return "<REDACTED>";
  return event.value;
}

std::string dump_events(const std::vector<Event> &events) {
  std::ostringstream out;
  for (const auto &event : events)
    out << event.name << ": " << redacted_value(event) << '\n';
  return out.str();
}

bool read_binary_file(const std::string &path, std::vector<uint8_t> *bytes, std::string *error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    *error = "failed to open " + path;
    return false;
  }
  bytes->assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  return true;
}

bool read_text_file(const std::string &path, std::string *text, std::string *error) {
  std::ifstream input(path);
  if (!input) {
    *error = "failed to open " + path;
    return false;
  }
  text->assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  return true;
}

bool write_text_file(const std::string &path, const std::string &text, std::string *error) {
  std::ofstream output(path);
  if (!output) {
    *error = "failed to open " + path + " for writing";
    return false;
  }
  output << text;
  return true;
}

bool expect(bool condition, const std::string &message, std::vector<std::string> *failures) {
  if (!condition)
    failures->push_back(message);
  return condition;
}

std::string first_push_value(const std::vector<Event> &events, const std::string &name) {
  bool in_first_push = false;
  for (const auto &event : events) {
    if (event.name == "push_timestamp") {
      if (in_first_push)
        break;
      in_first_push = true;
    }
    if (in_first_push && event.name == name)
      return event.value;
  }
  return "";
}

int count_events_named(const std::vector<Event> &events, const std::string &name) {
  int count = 0;
  for (const auto &event : events) {
    if (event.name == name)
      count++;
  }
  return count;
}

void register_all_sensors(esphome::e450_cyprus_meter::E450CyprusMeterComponent *meter,
                          std::vector<Event> *events, std::vector<CaptureSensor> *numeric,
                          std::vector<CaptureTextSensor> *text) {
  auto add_numeric = [&](const std::string &name, int decimals) -> CaptureSensor * {
    numeric->emplace_back(events, name, decimals);
    return &numeric->back();
  };
  auto add_text = [&](const std::string &name) -> CaptureTextSensor * {
    text->emplace_back(events, name);
    return &text->back();
  };

  numeric->reserve(28);
  text->reserve(6);

  meter->set_push_timestamp_text_sensor(add_text("push_timestamp"));
  meter->set_meter_clock_text_sensor(add_text("meter_clock"));
  meter->set_energy_push_timestamp_text_sensor(add_text("energy_push_timestamp"));
  meter->set_meter_logical_device_name_text_sensor(add_text("meter_logical_device_name"));
  meter->set_meter_serial_number_text_sensor(add_text("meter_serial_number"));
  meter->set_active_tariff_text_sensor(add_text("active_tariff"));

  meter->set_active_power_import_l1_sensor(add_numeric("active_power_import_l1", 0));
  meter->set_active_power_import_l2_sensor(add_numeric("active_power_import_l2", 0));
  meter->set_active_power_import_l3_sensor(add_numeric("active_power_import_l3", 0));
  meter->set_active_power_export_l1_sensor(add_numeric("active_power_export_l1", 0));
  meter->set_active_power_export_l2_sensor(add_numeric("active_power_export_l2", 0));
  meter->set_active_power_export_l3_sensor(add_numeric("active_power_export_l3", 0));
  meter->set_active_power_import_total_sensor(add_numeric("active_power_import_total", 0));
  meter->set_active_power_export_total_sensor(add_numeric("active_power_export_total", 0));
  meter->set_reactive_power_import_sensor(add_numeric("reactive_power_import", 0));
  meter->set_reactive_power_export_sensor(add_numeric("reactive_power_export", 0));
  meter->set_power_factor_sensor(add_numeric("power_factor", 3));
  meter->set_current_l1_sensor(add_numeric("current_l1", 2));
  meter->set_current_l2_sensor(add_numeric("current_l2", 2));
  meter->set_current_l3_sensor(add_numeric("current_l3", 2));
  meter->set_voltage_l1_sensor(add_numeric("voltage_l1", 0));
  meter->set_voltage_l2_sensor(add_numeric("voltage_l2", 0));
  meter->set_voltage_l3_sensor(add_numeric("voltage_l3", 0));
  meter->set_frequency_sensor(add_numeric("frequency", 0));
  meter->set_disconnect_control_state_sensor(add_numeric("disconnect_control_state", 0));
  meter->set_active_energy_import_t1_sensor(add_numeric("active_energy_import_t1", 0));
  meter->set_active_energy_import_t2_sensor(add_numeric("active_energy_import_t2", 0));
  meter->set_active_energy_export_t1_sensor(add_numeric("active_energy_export_t1", 0));
  meter->set_active_energy_export_t2_sensor(add_numeric("active_energy_export_t2", 0));
  meter->set_active_energy_import_total_sensor(add_numeric("active_energy_import_total", 0));
  meter->set_active_energy_export_total_sensor(add_numeric("active_energy_export_total", 0));
  meter->set_reactive_energy_import_sensor(add_numeric("reactive_energy_import", 0));
  meter->set_reactive_energy_export_sensor(add_numeric("reactive_energy_export", 0));
  meter->set_disconnect_limiter_state_sensor(add_numeric("disconnect_limiter_state", 0));
}

bool redaction_unit_test(std::vector<std::string> *failures) {
  const std::vector<Event> events = {
      {"meter_serial_number", "123456789"},
      {"meter_logical_device_name", "LGZ450"},
      {"push_timestamp", "2026-06-08 18:46:02"},
  };
  const std::string expected =
      "meter_serial_number: <REDACTED>\n"
      "meter_logical_device_name: <REDACTED>\n"
      "push_timestamp: 2026-06-08 18:46:02\n";
  return expect(dump_events(events) == expected, "redaction unit test failed", failures);
}

}  // namespace

int main() {
  const std::string capture_path = "tests/fixtures/mbus_frames.log";
  const std::string golden_path = "tests/fixtures/mbus_frames.redacted.dump";
  std::vector<std::string> failures;

  redaction_unit_test(&failures);

  std::vector<uint8_t> capture;
  std::string error;
  if (!read_binary_file(capture_path, &capture, &error)) {
    std::cerr << error << '\n';
    return 1;
  }

  esphome::e450_cyprus_meter::E450CyprusMeterComponent meter;
  std::vector<Event> events;
  std::vector<CaptureSensor> numeric_sensors;
  std::vector<CaptureTextSensor> text_sensors;
  register_all_sensors(&meter, &events, &numeric_sensors, &text_sensors);

  meter.setup();
  meter.push_bytes(capture);
  meter.loop();

  expect(meter.get_hdlc_frames_consumed() == 117,
         "expected 117 HDLC frames consumed, got " + std::to_string(meter.get_hdlc_frames_consumed()), &failures);
  expect(meter.get_valid_pushes() == 23,
         "expected 23 complete push groups, got " + std::to_string(meter.get_valid_pushes()), &failures);
  expect(meter.get_skipped_groups() == 0,
         "expected 0 skipped groups, got " + std::to_string(meter.get_skipped_groups()), &failures);
  expect(count_events_named(events, "push_timestamp") == 21,
         "expected 21 push_timestamp events, got " + std::to_string(count_events_named(events, "push_timestamp")),
         &failures);

  expect(first_push_value(events, "push_timestamp") == "2026-06-08 18:46:02",
         "first push_timestamp mismatch: " + first_push_value(events, "push_timestamp"), &failures);
  expect(first_push_value(events, "meter_clock") == "2026-06-08 18:46:04",
         "first meter_clock mismatch: " + first_push_value(events, "meter_clock"), &failures);
  expect(first_push_value(events, "active_power_import_total") == "187",
         "first active_power_import_total mismatch: " + first_push_value(events, "active_power_import_total"),
         &failures);
  expect(first_push_value(events, "current_l1") == "1.30",
         "first current_l1 mismatch: " + first_push_value(events, "current_l1"), &failures);
  expect(first_push_value(events, "power_factor") == "0.286",
         "first power_factor mismatch: " + first_push_value(events, "power_factor"), &failures);
  expect(first_push_value(events, "frequency") == "50",
         "first frequency mismatch: " + first_push_value(events, "frequency"), &failures);

  const std::string actual_dump = dump_events(events);
  if (std::getenv("UPDATE_GOLDEN") != nullptr) {
    std::filesystem::create_directories("tests/fixtures");
    if (!write_text_file(golden_path, actual_dump, &error)) {
      std::cerr << error << '\n';
      return 1;
    }
  } else {
    std::string expected_dump;
    if (!read_text_file(golden_path, &expected_dump, &error)) {
      std::cerr << error << '\n';
      return 1;
    }
    expect(actual_dump == expected_dump, "decoded dump did not match " + golden_path, &failures);
  }

  if (!failures.empty()) {
    for (const auto &failure : failures)
      std::cerr << failure << '\n';
    return 1;
  }

  std::cout << "C++ mbus_frames test passed: " << meter.get_hdlc_frames_consumed() << " HDLC frames, "
            << meter.get_valid_pushes() << " push groups, " << events.size() << " published events\n";
  return 0;
}
