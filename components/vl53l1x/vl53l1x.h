#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l1x {

enum DistanceMode {
  SHORT = 0,
  LONG,
};

// to store ranging results which are read from registers
// RESULT__RANGE_STATUS (0x0089) to
// RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LOW (0x0099)
struct RangingResults {
  uint8_t  range_status;
  uint8_t  report_status;                   // not used
  uint8_t  stream_count;
  uint16_t dss_actual_effective_spads_sd0;
  uint16_t peak_signal_count_rate_mcps_sd0; // not used
  uint16_t ambient_count_rate_mcps_sd0;
  uint16_t sigma_sd0;                       // not used
  uint16_t phase_sd0;                       // not used
  uint16_t final_crosstalk_corrected_range_mm_sd0;
  uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
};

class VL53L1XComponent : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_range_status_sensor(sensor::Sensor *range_status_sensor) { range_status_sensor_ = range_status_sensor; }
  void config_distance_mode(DistanceMode distance_mode ) { distance_mode_ = distance_mode; }

  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override;

  std::string range_status_to_string();

 protected:
  DistanceMode distance_mode_;

  uint16_t distance_{0};

  enum RangeStatus {
    RANGE_VALID = 0,
    RANGE_VALID_NOWRAP_CHECK_FAIL,
    RANGE_VALID_MIN_RANGE_CLIPPED,
    HARDWARE_FAIL,
    SIGNAL_FAIL,
    OUT_OF_BOUNDS_FAIL,
    SIGMA_FAIL,
    WRAP_TARGET_FAIL,
    MIN_RANGE_FAIL,
    UNDEFINED,
  } range_status_{UNDEFINED};

  enum ErrorCode {
    NONE = 0,
    WRONG_CHIP_ID,
    SOFT_RESET_FAILED,
    BOOT_STATE_FAILED,
    BOOT_STATE_TIMEOUT,
    CONFIG_FAILED,
    SET_MODE_FAILED,
    START_RANGING_FAILED,
    SENSOR_READ_FAILED,
  } error_code_{NONE};

  bool get_sensor_id(bool *valid_sensor);
  bool boot_state(uint8_t *state);

  bool set_timing_budget(uint16_t timing_budget_ms);
  bool get_timing_budget(uint16_t *timing_budget_ms);

  bool set_distance_mode(DistanceMode distance_mode);
  bool get_distance_mode(DistanceMode *mode);

  bool start_continuous(uint32_t period_ms);
  bool stop_continuous();

  bool start_oneshot();

  bool check_for_dataready(bool *is_dataready);

  bool perform_sensor_read();
  bool read_ranging_results();
  bool setup_manual_calibration();
  bool update_dss();

  uint32_t decode_timeout(uint16_t reg_val);
  uint16_t encode_timeout(uint32_t timeout_mclks);
  uint32_t timeout_mclks_to_microseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
  uint32_t timeout_microseconds_to_mclks(uint32_t timeout_us, uint32_t macro_period_us);
  uint32_t calculate_macro_period(uint8_t vcsel_period);

  bool vl53l1x_write_bytes(uint16_t a_register, const uint8_t *data, uint8_t len);
  bool vl53l1x_write_byte(uint16_t a_register, uint8_t data);

  bool vl53l1x_write_bytes_16(uint8_t a_register, const uint16_t *data, uint8_t len);
  bool vl53l1x_write_byte_16(uint16_t a_register, uint16_t data);


  bool vl53l1x_read_bytes(uint16_t a_register, uint8_t *data, uint8_t len);
  bool vl53l1x_read_byte(uint16_t a_register, uint8_t *data);

  bool vl53l1x_read_bytes_16(uint16_t a_register, uint16_t *data, uint8_t len);
  bool vl53l1x_read_byte_16(uint16_t a_register, uint16_t *data);


  // pololu globals
  bool calibrated_{false};
  uint8_t saved_vhv_init_{0};
  uint8_t saved_vhv_timeout_{0};

  uint16_t fast_osc_frequency_;
  uint16_t osc_calibrate_val_;

  RangingResults results_;

  // internal
  bool distance_mode_overriden_{false};
  bool ranging_active_{false};
  uint16_t sensor_id_{0};
  uint32_t last_loop_time_{0};

  // sensors
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *range_status_sensor_{nullptr};
};

}  // namespace vl53l1x
}  // namespace esphome
