/*
This ESPHome component was based on the Polulo VL53L1X Arduino Library
which in turn was based on the VL53L1X API by STMicroelectronics.
Therefore the licence terms are the same as those presented in the
Polulo VL53L1X Arduino Library as specified below.
Most comments are those provided in thePolulo VL53L1X Arduino Library.

Polulo VL53L1X Arduino Library states that most of the functionality of their
library is based on the VL53L1X API provided provided by ST (STSW-IMG007),
and some of the explanatory comments are quoted or paraphrased
from the API source code, API user manual (UM2356), and VL53L1X datasheet.
Therefore, the license terms for the API source code (BSD 3-clause
"New" or "Revised" License) also apply to this derivative work, as specified below.

Copyright (c) 2017, STMicroelectronics
Copyright (c) 2018-2022, Pololu Corporation
All Rights Reserved

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "vl53l1x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vl53l1x {
static const char *const TAG = "vl53l1x.sensor";

// TimingGuard value used in measurement timing budget calculations
// assumes PresetMode is LOWPOWER_AUTONOMOUS
// vhv         = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
//               (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
//             = 245 + 3 * 245 = 980
// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING + LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
//             = 1448 + 2100 + 980 = 4528
static const uint32_t TIMING_GUARD = 4528;

// value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS calculations
static const uint16_t TARGET_RATE  = 0x0A00;

// VL53L1x registers
static const uint16_t SOFT_RESET                                                          = 0x0000;
static const uint16_t OSC_MEASURED__FAST_OSC__FREQUENCY                                   = 0x0006;
static const uint16_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                               = 0x0008;
static const uint16_t VHV_CONFIG__INIT                                                    = 0x000B;
static const uint16_t ALGO__PART_TO_PART_RANGE_OFFSET_MM                                  = 0x001E;
static const uint16_t MM_CONFIG__OUTER_OFFSET_MM                                          = 0x0022;
static const uint16_t DSS_CONFIG__TARGET_TOTAL_RATE_MCPS                                  = 0x0024;
static const uint16_t PAD_I2C_HV__EXTSUP_CONFIG                                           = 0x002E;
static const uint16_t GPIO__TIO_HV_STATUS                                                 = 0x0031;
static const uint16_t SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS                           = 0x0036;
static const uint16_t SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS                         = 0x0037;
static const uint16_t ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM                        = 0x0039;
static const uint16_t ALGO__RANGE_IGNORE_VALID_HEIGHT_MM                                  = 0x003E;
static const uint16_t ALGO__RANGE_MIN_CLIP                                                = 0x003F;
static const uint16_t ALGO__CONSISTENCY_CHECK__TOLERANCE                                  = 0x0040;
static const uint16_t CAL_CONFIG__VCSEL_START                                             = 0x0047;
static const uint16_t PHASECAL_CONFIG__TIMEOUT_MACROP                                     = 0x004B;
static const uint16_t PHASECAL_CONFIG__OVERRIDE                                           = 0x004D;
static const uint16_t DSS_CONFIG__ROI_MODE_CONTROL                                        = 0x004F;
static const uint16_t SYSTEM__THRESH_RATE_HIGH                                            = 0x0050;
static const uint16_t SYSTEM__THRESH_RATE_LOW                                             = 0x0052;
static const uint16_t DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT                           = 0x0054;
static const uint16_t DSS_CONFIG__APERTURE_ATTENUATION                                    = 0x0057;
static const uint16_t MM_CONFIG__TIMEOUT_MACROP_A                                         = 0x005A; // added by Pololu for 16-bit accesses
static const uint16_t MM_CONFIG__TIMEOUT_MACROP_B                                         = 0x005C; // added by Pololu for 16-bit accesses
static const uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A                                      = 0x005E; // added by Pololu for 16-bit accesses
static const uint16_t RANGE_CONFIG__VCSEL_PERIOD_A                                        = 0x0060;
static const uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B                                      = 0x0061; // added by Pololu for 16-bit accesses
static const uint16_t RANGE_CONFIG__VCSEL_PERIOD_B                                        = 0x0063;
static const uint16_t RANGE_CONFIG__SIGMA_THRESH                                          = 0x0064;
static const uint16_t RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                         = 0x0066;
static const uint16_t RANGE_CONFIG__VALID_PHASE_HIGH                                      = 0x0069;
static const uint16_t SYSTEM__INTERMEASUREMENT_PERIOD                                     = 0x006C;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD_0                                    = 0x0071;
static const uint16_t SYSTEM__SEED_CONFIG                                                 = 0x0077;
static const uint16_t SD_CONFIG__WOI_SD0                                                  = 0x0078;
static const uint16_t SD_CONFIG__WOI_SD1                                                  = 0x0079;
static const uint16_t SD_CONFIG__INITIAL_PHASE_SD0                                        = 0x007A;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD_1                                    = 0x007C;
static const uint16_t SD_CONFIG__QUANTIFIER                                               = 0x007E;
static const uint16_t SYSTEM__SEQUENCE_CONFIG                                             = 0x0081;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD                                      = 0x0082;
static const uint16_t SYSTEM__INTERRUPT_CLEAR                                             = 0x0086;
static const uint16_t SYSTEM__MODE_START                                                  = 0x0087;
static const uint16_t RESULT__RANGE_STATUS                                                = 0x0089;
static const uint16_t RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                      = 0x0096;
static const uint16_t PHASECAL_RESULT__VCSEL_START                                        = 0x00D8;
static const uint16_t RESULT__OSC_CALIBRATE_VAL                                           = 0x00DE;
static const uint16_t FIRMWARE__SYSTEM_STATUS                                             = 0x00E5;
static const uint16_t IDENTIFICATION__MODEL_ID                                            = 0x010F;

static const uint16_t BOOT_TIMEOUT     = 120;
static const uint16_t TIMING_BUDGET    = 500;                          // timing budget is maximum allowable = 500 ms
static const uint16_t RANGING_FINISHED = (TIMING_BUDGET * 115) / 100;  // add 15% extra to timing budget to ensure ranging is finished

// Sensor Initialisation
void VL53L1XComponent::setup() {
  // try checking sensor id before reset
  bool valid_sensor = false;
  if (this->get_sensor_id(&valid_sensor)) {
    if (!valid_sensor) {
      this->error_code_ = WRONG_CHIP_ID;
      this->mark_failed();
      return;
    }
  }

  // reset sensor
  if (!this->vl53l1x_write_byte(SOFT_RESET, 0x00)) {
    ESP_LOGE(TAG, "Error writing soft reset 0");
    this->error_code_ = SOFT_RESET_FAILED;
    this->mark_failed();
    return;
  }

  delayMicroseconds(100);

  if (!this->vl53l1x_write_byte(SOFT_RESET, 0x01)) {
    ESP_LOGE(TAG, "Error writing soft reset 1");
    this->error_code_ = SOFT_RESET_FAILED;
    this->mark_failed();
    return;
  }

  // give sensor time to boot
  delayMicroseconds(1200);

  // now wait for sensor to boot successfully
  uint8_t state = 0;
  uint32_t start_time = millis();
  while ((millis() - start_time) < BOOT_TIMEOUT ) {
    if (!this->boot_state(&state)) {
      this->error_code_ = BOOT_STATE_FAILED;
      this->mark_failed();
      return;
    }
    if (state) break;
  }

  if (!state) {
    this->error_code_ = BOOT_STATE_TIMEOUT;
    this->mark_failed();
    return;
  }

  // if getting sensor id failed prior to reset then try again
  if (!valid_sensor) {
    this->get_sensor_id(&valid_sensor);
    if (!valid_sensor) {
      this->error_code_ = WRONG_CHIP_ID;
      this->mark_failed();
      return;
    }
  }

  bool ok = true;
  // sensor uses 1V8 mode for I/O by default
  // code examples by default switch to 2V8 mode
  uint8_t config;
  if (ok) ok = this->vl53l1x_read_byte(PAD_I2C_HV__EXTSUP_CONFIG, &config);
  if (ok) ok = this->vl53l1x_write_byte(PAD_I2C_HV__EXTSUP_CONFIG, config | 0x01);

  // store oscillator info for later use
  if (ok) ok = this->vl53l1x_read_byte_16(OSC_MEASURED__FAST_OSC__FREQUENCY, &this->fast_osc_frequency_);
  if (ok) ok = this->vl53l1x_read_byte_16(RESULT__OSC_CALIBRATE_VAL, &this->osc_calibrate_val_);

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // API uses these in VL53L1_init_tuning_parm_storage_struct()

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do that?
  // asit seems like it would disable 2V8 mode
  if (ok) ok = this->vl53l1x_write_byte_16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE);  // should already be this value after reset
  if (ok) ok = this->vl53l1x_write_byte(GPIO__TIO_HV_STATUS, 0x02);
  if (ok) ok = this->vl53l1x_write_byte(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);        // tuning parm default
  if (ok) ok = this->vl53l1x_write_byte(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);     // tuning parm default
  if (ok) ok = this->vl53l1x_write_byte(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  if (ok) ok = this->vl53l1x_write_byte(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  if (ok) ok = this->vl53l1x_write_byte(ALGO__RANGE_MIN_CLIP, 0);                             // tuning parm default
  if (ok) ok = this->vl53l1x_write_byte(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);               // tuning parm default


  // general config
  if (ok) ok = this->vl53l1x_write_byte_16(SYSTEM__THRESH_RATE_HIGH, 0x0000);
  if (ok) ok = this->vl53l1x_write_byte_16(SYSTEM__THRESH_RATE_LOW, 0x0000);
  if (ok) ok = this->vl53l1x_write_byte(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  if (ok) ok = this->vl53l1x_write_byte_16(RANGE_CONFIG__SIGMA_THRESH, 360);                  // tuning parm default
  if (ok) ok = this->vl53l1x_write_byte_16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config
  if (ok) ok = this->vl53l1x_write_byte(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  if (ok) ok = this->vl53l1x_write_byte(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  if (ok) ok = this->vl53l1x_write_byte(SD_CONFIG__QUANTIFIER, 2);                            // tuning parm default

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1
  // and things don't seem to work if we don't set GPH back to 0 (which the API does here)
  if (ok) ok = this->vl53l1x_write_byte(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  if (ok) ok = this->vl53l1x_write_byte(SYSTEM__SEED_CONFIG, 1);                              // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  if (ok) ok = this->vl53l1x_write_byte(SYSTEM__SEQUENCE_CONFIG, 0x8B);                       // VHV, PHASECAL, DSS1, RANGE
  if (ok) ok = this->vl53l1x_write_byte_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  if (ok) ok = this->vl53l1x_write_byte(DSS_CONFIG__ROI_MODE_CONTROL, 2);                     // REQUESTED_EFFFECTIVE_SPADS

  if (!ok) {
    this->error_code_ = CONFIG_FAILED;
    this->mark_failed();
    return;
  }

  // 0xEBAA = VL53L4CD must run with SHORT distance mode
  if ((this->sensor_id_ == 0xEBAA) && (this->distance_mode_ == LONG)) {
    this->distance_mode_ = SHORT;
    this->distance_mode_overriden_ = true;
  }

  if (!this->set_distance_mode(this->distance_mode_)) {
    this->error_code_ = SET_MODE_FAILED;
    this->mark_failed();
    return;
  }

  if (!this->set_timing_budget(TIMING_BUDGET)) {
    this->error_code_ = SET_MODE_FAILED;
    this->mark_failed();
    return;
  }

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  uint16_t offset;
  if (ok) ok = this->vl53l1x_read_byte_16(MM_CONFIG__OUTER_OFFSET_MM, &offset);
  if (ok) ok = this->vl53l1x_write_byte_16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, offset * 4);
  if (!ok) {
    this->error_code_ = CONFIG_FAILED;
    this->mark_failed();
    return;
  }
}

void VL53L1XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");

  switch (this->error_code_) {
    case WRONG_CHIP_ID:
      ESP_LOGE(TAG, " Sensor id does not match VL53L1X or VL53L4CD OR communication failure reading sensor id");
      break;

    case SOFT_RESET_FAILED:
      ESP_LOGE(TAG, "  Soft reset communication failure");
      break;

    case BOOT_STATE_FAILED:
      ESP_LOGE(TAG, "  Boot state communication failure");
      break;

    case BOOT_STATE_TIMEOUT:
      ESP_LOGE(TAG, "  Timeout waiting for sensor to boot");
      break;

    case CONFIG_FAILED:
      ESP_LOGE(TAG, "  Communication failure when configuring sensor");
      break;

    case SET_MODE_FAILED:
      ESP_LOGE(TAG, "  Communication failure when setting distance or timing budget");
      break;

    case START_RANGING_FAILED:
      ESP_LOGE(TAG, "  Start Ranging failed");
      break;
    case SENSOR_READ_FAILED:
      ESP_LOGE(TAG, " Sensor read process failed");
      break;
    case NONE:
      ESP_LOGD(TAG, "  Setup successful");

      // no errors so sensor must be VL53L1X or VL53L4CD
      if (this->sensor_id_ == 0xEACC) {
        ESP_LOGI(TAG, "  Found sensor: VL53L1X");
      }
      if (this->sensor_id_ == 0xEBAA) {
        ESP_LOGI(TAG,"  Found sensor: VL53L4CD");
      }

      if (this->distance_mode_overriden_) {
        ESP_LOGW(TAG, "  VL53L4CD Distance Mode overriden: must be SHORT");
      }
      else {
        if (this->distance_mode_ == SHORT) {
          ESP_LOGCONFIG(TAG, "  Distance Mode: SHORT");
        }
        else {
          ESP_LOGCONFIG(TAG, "  Distance Mode: LONG");
        }
      }
      ESP_LOGD(TAG, "  Timing Budget: %ims",TIMING_BUDGET);
      LOG_I2C_DEVICE(this);
      LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Distance Sensor:", this->distance_sensor_);
      LOG_SENSOR("  ", "Range Status Sensor:", this->range_status_sensor_);

      break;
   }
}

void VL53L1XComponent::loop() {
  bool is_dataready;
  // only run loop if not updating and every LOOP_TIME
  if ((!this->ranging_active_) || ((millis() - this->last_loop_time_) < RANGING_FINISHED) || this->is_failed() )
    return;

  if (!this->check_for_dataready(&is_dataready)) {
    ESP_LOGD(TAG, "  Checking for data ready failed");
    this->ranging_active_ = false;
    return;
  }

  if (!is_dataready) {
    ESP_LOGD(TAG, "  Data ready not ready when it should be!");
    this->ranging_active_ = false;
    return;
  }

  // data ready now, so read and publish
  if (!this->perform_sensor_read()) {
    this->error_code_ = SENSOR_READ_FAILED;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Publishing Distance: %imm with Ranging status: %i",this->distance_,this->range_status_);
  if (this->distance_sensor_ != nullptr)
     this->distance_sensor_->publish_state(this->distance_);
  if (this->range_status_sensor_ != nullptr)
     this->range_status_sensor_->publish_state(this->range_status_);

  this->ranging_active_ = false;
}

void VL53L1XComponent::update() {
  if (this->ranging_active_) {
    ESP_LOGD(TAG, " Update triggered while ranging active"); // should never happen
    return;
  }

  if (!this->start_oneshot()) {
    ESP_LOGE(TAG, " Start ranging failed in update");
    this->error_code_ = START_RANGING_FAILED;
    this->mark_failed();
    return;
  }
  this->ranging_active_ = true;
  this->last_loop_time_ = millis();
}

float VL53L1XComponent::get_setup_priority() const { return setup_priority::DATA; }

bool VL53L1XComponent::boot_state(uint8_t* state) {
  // 0 = not booted, 1 = booted
  *state = 0;
  return this->vl53l1x_read_byte(FIRMWARE__SYSTEM_STATUS, state);
}

bool VL53L1XComponent::get_sensor_id(bool* valid_sensor) {
  if (!this->vl53l1x_read_byte_16(IDENTIFICATION__MODEL_ID, &this->sensor_id_)) {
    *valid_sensor = false;
    return false;
  }
  // 0xEACC = VL53L1X, 0xEBAA = VL53L4CD
  *valid_sensor = ((this->sensor_id_ == 0xEACC) || (this->sensor_id_ == 0xEBAA));
  return true;
}

bool VL53L1XComponent::set_distance_mode(DistanceMode distance_mode) {
  uint16_t timing_budget;
  if (!this->get_timing_budget(&timing_budget)) {
    ESP_LOGE(TAG, "  Reading timimg budget failed when setting distance mode");
    return false;
  }

  bool ok = true;
  switch (distance_mode) {
    case SHORT:
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      if (ok) ok = this->vl53l1x_write_byte_16(SD_CONFIG__WOI_SD0, 0x0705);
      if (ok) ok = this->vl53l1x_write_byte_16(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
      break;
    case LONG:
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      if (ok) ok = this->vl53l1x_write_byte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
      if (ok) ok = this->vl53l1x_write_byte_16(SD_CONFIG__WOI_SD0, 0x0F0D);
      if (ok) ok = this->vl53l1x_write_byte_16(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
      break;
    default:
      // should never happen
      ESP_LOGE(TAG,"  Attempt to set invalid distance mode"); // should never happen
      return false;
  }

  if (!ok ) {
    ESP_LOGE(TAG, "  Writing set distance mode configuration values failed");
    return false;
  }

  if (!this->set_timing_budget(timing_budget)) {
    ESP_LOGE(TAG, "  Re-writing timing budget failed when setting distance mode");
    return false;
  }

  return true;
}

// set the measurement timing budget, which is the time allowed for one measurement
// longer timing budget allows for more accurate measurements
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool VL53L1XComponent::set_timing_budget(uint16_t timing_budget_ms) {
  uint32_t budget_us;
  budget_us = (uint32_t)(timing_budget_ms * 1000);

  // assumes PresetMode is LOWPOWER_AUTONOMOUS
  if (budget_us <= TIMING_GUARD) return false;

  uint32_t range_config_timeout_us = budget_us -= TIMING_GUARD;
  if (range_config_timeout_us > 1100000) return false; // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // based on VL53L1_calc_timeout_register_values()

  uint32_t macro_period_us;

  // update macro period for Range A VCSEL Period
  uint8_t temp;
  if (!this->vl53l1x_read_byte(RANGE_CONFIG__VCSEL_PERIOD_A, &temp)) return false;
  macro_period_us = calculate_macro_period(temp);

  // update phase timeout - uses Timing A
  // timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg()
  uint32_t phasecal_timeout_mclks = timeout_microseconds_to_mclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) phasecal_timeout_mclks = 0xFF;
  if (!this->vl53l1x_write_byte(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks)) return false;

  // update MM Timing A timeout
  // timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg()
  // with the API, the register actually ends up with a slightly different value
  // because it gets assigned, retrieved, recalculated with a different macro period, and reassigned,
  // but it probably do not matter because it seems like the MM (mode mitigation ?)
  // sequence steps are disabled in low power auto mode anyway

  if (!this->vl53l1x_write_byte_16(MM_CONFIG__TIMEOUT_MACROP_A,
                                   encode_timeout(timeout_microseconds_to_mclks(1, macro_period_us)))) return false;

  // update range Timing A timeout
  if (!this->vl53l1x_write_byte_16(RANGE_CONFIG__TIMEOUT_MACROP_A,
                                   encode_timeout(timeout_microseconds_to_mclks(range_config_timeout_us, macro_period_us)))) return false;

  // update macro period for Range B VCSEL Period
  if (!this->vl53l1x_read_byte(RANGE_CONFIG__VCSEL_PERIOD_B, &temp)) return false;
  macro_period_us = calculate_macro_period(temp);

  // update MM Timing B timeout
  // see above comment about MM Timing A timeout
  if (!this->vl53l1x_write_byte_16(MM_CONFIG__TIMEOUT_MACROP_B,
                                   encode_timeout(timeout_microseconds_to_mclks(1, macro_period_us)))) return false;

  // update Range Timing B timeout
  if (!this->vl53l1x_write_byte_16(RANGE_CONFIG__TIMEOUT_MACROP_B,
                                   encode_timeout(timeout_microseconds_to_mclks(range_config_timeout_us, macro_period_us)))) return false;
  return true;
}

// get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool VL53L1XComponent::get_timing_budget(uint16_t *timing_budget_ms) {
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // update macro period for Range A VCSEL Period
  uint8_t temp_macro;
  if (!this->vl53l1x_read_byte(RANGE_CONFIG__VCSEL_PERIOD_A, &temp_macro)) return false;
  uint32_t macro_period_us = calculate_macro_period(temp_macro);

  // get Range Timing A timeout
  uint16_t temp_timeout;
  if (!this->vl53l1x_read_byte_16(RANGE_CONFIG__TIMEOUT_MACROP_A, &temp_timeout)) return false;
  uint32_t range_config_timeout_us = timeout_mclks_to_microseconds(decode_timeout(temp_timeout), macro_period_us);

  // VL53L1_get_timeouts_us() end

  uint32_t timing_budget_us = (2 * range_config_timeout_us) + TIMING_GUARD;
  *timing_budget_ms = (uint16_t)(timing_budget_us / 1000);
  return true;
}

bool VL53L1XComponent::get_distance_mode(DistanceMode *mode) {
  uint8_t raw_distance_mode;

  if (!this->vl53l1x_read_byte(PHASECAL_CONFIG__TIMEOUT_MACROP, &raw_distance_mode)) {
    ESP_LOGE(TAG, "  Error reading distance mode");
    return false;
  }

  if (raw_distance_mode == 0x14) {
    *mode = SHORT;
    return true;
  }

  if (raw_distance_mode == 0x0A) {
    *mode = LONG;
    return true;
  }

  // should never get here
  ESP_LOGE(TAG, "  Invalid value when reading distance mode");
  return false;
}

// start continuous ranging measurements, with the given intermeasurement
// period in milliseconds determining how often the sensor takes a measurement
// based on VL53L1_set_inter_measurement_period_ms()
bool VL53L1XComponent::start_continuous(uint32_t period_ms) {
  uint32_t intermeasurement_period = static_cast<uint32_t>(period_ms * this->osc_calibrate_val_);

  if (!this->vl53l1x_write_bytes_16(SYSTEM__INTERMEASUREMENT_PERIOD,
                                    reinterpret_cast<const uint16_t *>(&intermeasurement_period), 2)) {
    ESP_LOGE(TAG, "Error writing intermeasurement period");
    return false;
  }

  // sys_interrupt_clear_range
  if (!this->vl53l1x_write_byte(SYSTEM__INTERRUPT_CLEAR, 0x01))  {
    ESP_LOGE(TAG, "Error writing clear interrupt");
    return false;
  }

  // mode_range__timed
  if (!this->vl53l1x_write_byte(SYSTEM__MODE_START, 0x40)) {
    ESP_LOGE(TAG, "Error writing start continuous ranging");
    return false;
  }

  return true;
}

// based on VL53L1_stop_range()
bool VL53L1XComponent::stop_continuous() {
  // mode_range__abort
  if (!this->vl53l1x_write_byte(SYSTEM__MODE_START, 0x80)) {
    ESP_LOGE(TAG, "  Error writing stop ranging");
    return false;
  }

  // based on VL53L1_low_power_auto_data_stop_range()
  this->calibrated_ = false;

  bool ok = true;

  // restore vhv configs
  if (this->saved_vhv_init_ != 0) {
    if (ok) ok = this->vl53l1x_write_byte(VHV_CONFIG__INIT, this->saved_vhv_init_);
  }

  if (this->saved_vhv_timeout_ != 0) {
    if (ok) ok = this->vl53l1x_write_byte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, this->saved_vhv_timeout_);
  }

  // remove phasecal override
  if (ok) ok = this->vl53l1x_write_byte(PHASECAL_CONFIG__OVERRIDE, 0x00);

  if (!ok) {
    ESP_LOGE(TAG, "  Error writing configuration for stop ranging");
    return false;
  }
  return true;
}


bool VL53L1XComponent::start_oneshot() {
  // clear interrupt trigger
  if (!this->vl53l1x_write_byte(SYSTEM__INTERRUPT_CLEAR, 0x01))  {
    ESP_LOGE(TAG, "  Error writing clear interrupt when starting one-shot ranging");
    return false;
  }

  // enable one-shot ranging
  if (!this->vl53l1x_write_byte(SYSTEM__MODE_START, 0x10)) {
    ESP_LOGE(TAG, "  Error writing start one-shot ranging");
    return false;
  }

  return true;
}


bool VL53L1XComponent::check_for_dataready(bool *is_dataready) {
  uint8_t temp;
  if (!this->vl53l1x_read_byte(GPIO__TIO_HV_STATUS, &temp)) {
    ESP_LOGE(TAG, "  Error reading data ready");
    *is_dataready = false;
    return false;
  }
  // assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
  *is_dataready = ((temp & 0x01) == 0);
  return true;
}



// perform sensor read process
bool VL53L1XComponent::perform_sensor_read() {
  if (!read_ranging_results()) {
    ESP_LOGE(TAG, "  Error reading ranging results");
    return false;
  }

  if (!this->calibrated_) {
    if (!setup_manual_calibration()) {
      ESP_LOGE(TAG, "  Error setting up manual calibration");
      return false;
    }
    this->calibrated_ = true;
  }

  if (!update_dss()) {
    ESP_LOGE(TAG, "  Error updating dynamic SPAD selection");
    return false;
  }

  // sys_interrupt_clear_range
  if (!this->vl53l1x_write_byte(SYSTEM__INTERRUPT_CLEAR, 0x01))  {
    ESP_LOGE(TAG, "  Error writing clear interrupt after reading sensor");
    return false;
  }

  return true;
}

bool VL53L1XComponent::read_ranging_results() {
  // if (!this->vl53l1x_read_bytes(RESULT__RANGE_STATUS, reinterpret_cast<uint8_t *>(&this->results_.range_status), 17)) {
  //   ESP_LOGW(TAG, "Error reading Range Status");
  //   this->status_set_warning();
  //   return false;
  // }
  uint8_t results_buffer[17];
  uint8_t status;

  if (!this->vl53l1x_read_bytes(RESULT__RANGE_STATUS, results_buffer, 17)) {
    ESP_LOGE(TAG, "  Error reading ranging results");
    return false;
  }

  this->results_.range_status = results_buffer[0];
  this->results_.stream_count = results_buffer[2];

  this->results_.dss_actual_effective_spads_sd0  = (uint16_t)results_buffer[3] << 8 | results_buffer[4];
  this->results_.ambient_count_rate_mcps_sd0  = (uint16_t)results_buffer[7] << 8 | results_buffer[8];

  this->results_.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)results_buffer[13] << 8 | results_buffer[14];
  this->results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)results_buffer[15] << 8 | results_buffer[16];


  switch(this->results_.range_status) {
    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (this->results_.stream_count != 0) {
        this->range_status_ = RANGE_VALID;
      }
      else {
        this->range_status_ = RANGE_VALID_NOWRAP_CHECK_FAIL; // range valid but wraparound check has not been done
      }
      break;

    case 8: // MINCLIP
      this->range_status_ = RANGE_VALID_MIN_RANGE_CLIPPED; // target is below minimum detection threshold
      break;

    case 1: // VCSELCONTINUITYTESTFAILURE
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 3: // NOVHVVALUEFOUND
    case 17: // MULTCLIPFAIL
      // from SetSimpleData()
      this->range_status_ = HARDWARE_FAIL; // hardware or VCSEL failure
      break;

    case 4: // MSRCNOTARGET
      this->range_status_ = SIGNAL_FAIL; // signal value below internal defined threshold
      break;

    case 5: // RANGEPHASECHECK
      this->range_status_ =  OUT_OF_BOUNDS_FAIL; // nothing detected in range (try a longer rang mode if applicable)
      break;

    case 6: // SIGMATHRESHOLDCHECK
      this->range_status_ = SIGMA_FAIL; // sigma (standard deviation) estimator check is above internally defined threshold
      break;

    case 7: // PHASECONSISTENCY
      this->range_status_ = WRAP_TARGET_FAIL; // wrapped target not matching phases, no matching phase in other VCSEL period timing
      break;

    case 13: // USERROICLIP - target is below minimum detection threshold
      // from SetSimpleData()
      this->range_status_ = MIN_RANGE_FAIL;
      break;

    default:
      this->range_status_ = UNDEFINED;
  }

  uint32_t range = static_cast<uint32_t>(this->results_.final_crosstalk_corrected_range_mm_sd0);

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // basically, this appears to scale the result by 2011 / 2048(0x0800) or about 98%
  // with the 1024(0x0400) added for proper rounding
  this->distance_ = static_cast<uint16_t>(((range * 2011) + 0x0400) / 0x0800);

  return true;
}

// setup ranges after the first one in low power auto mode by turning off
// calibration steps and programming static values
// based on VL53L1_low_power_auto_setup_manual_calibration()
bool VL53L1XComponent::setup_manual_calibration() {
  // "save original vhv configs"
  if (!this->vl53l1x_read_byte(VHV_CONFIG__INIT, &this->saved_vhv_init_)) return false;
  if (!this->vl53l1x_read_byte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &this->saved_vhv_timeout_)) return false;

  // "disable VHV init"
  if (!this->vl53l1x_write_byte(VHV_CONFIG__INIT, this->saved_vhv_init_ & 0x7F)) return false;

  // set loop bound to tuning param
  // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
  if (!this->vl53l1x_write_byte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (this->saved_vhv_timeout_ & 0x03) + (3 << 2))) return false;

  // override phasecal
  if (!this->vl53l1x_write_byte(PHASECAL_CONFIG__OVERRIDE, 0x01)) return false;
  uint8_t temp;
  if (!this->vl53l1x_read_byte(PHASECAL_RESULT__VCSEL_START, &temp)) return false;
  if (!this->vl53l1x_write_byte(CAL_CONFIG__VCSEL_START, temp)) return false;
  return true;
}

// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
bool VL53L1XComponent::update_dss() {
  uint16_t spadCount = this->results_.dss_actual_effective_spads_sd0;

  if (spadCount != 0) {
    // calc total rate per spad
    uint32_t totalRatePerSpad =
      (uint32_t)this->results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      this->results_.ambient_count_rate_mcps_sd0;

    // clip to 16 bits
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // shift up to take advantage of 32 bits
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0) {
      // get the target rate and shift up by 16
      uint32_t requiredSpads = ((uint32_t)TARGET_RATE << 16) / totalRatePerSpad;

      // clip to 16 bit
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // override DSS config
      if (!this->vl53l1x_write_byte_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads)) return false;

      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS
      return true;
    }
  }

  // if we reached this point, it means something above would have resulted in a divide by zero
  // gracefully set a spad target, not just exit with an error

  // set target to mid point
  if (!this->vl53l1x_write_byte_16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000)) return false;
  return true;
}

// decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t VL53L1XComponent::decode_timeout(uint16_t reg_val) {
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t VL53L1XComponent::encode_timeout(uint32_t timeout_mclks) {
  // encoded format: (LSByte * 2^MSByte) + 1

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  } else {
    return 0;
  }
}

// convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t VL53L1XComponent::timeout_mclks_to_microseconds(uint32_t timeout_mclks, uint32_t macro_period_us) {
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t VL53L1XComponent::timeout_microseconds_to_mclks(uint32_t timeout_us, uint32_t macro_period_us) {
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t VL53L1XComponent::calculate_macro_period(uint8_t vcsel_period) {
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / this->fast_osc_frequency_;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

std::string VL53L1XComponent::range_status_to_string() {
  switch (this->range_status_) {
    case RANGE_VALID:
      return "Range valid";

    case RANGE_VALID_NOWRAP_CHECK_FAIL:
      return "Range valid, no wrap check fail";

    case RANGE_VALID_MIN_RANGE_CLIPPED:
      return "Range valid, minimum range clipped";

    case HARDWARE_FAIL:
      return "Hardware fail";

    case SIGNAL_FAIL:
      return "Signal fail";

    case OUT_OF_BOUNDS_FAIL:
      return "Out of bounds fail";

    case SIGMA_FAIL:
      return "Sigma fail";

    case WRAP_TARGET_FAIL:
      return "Wrap target fail";

    case MIN_RANGE_FAIL:
      return "Minimum range fail";

    default:
      return "Undefined ranging fail";
  }
}

bool VL53L1XComponent::vl53l1x_write_bytes(uint16_t a_register, const uint8_t *data, uint8_t len) {
    return this->write_register16(a_register, data, len, true) == i2c::ERROR_OK;
}

bool VL53L1XComponent::vl53l1x_write_byte(uint16_t a_register, uint8_t data) {
    return this->vl53l1x_write_bytes(a_register, &data, 1);
}

bool VL53L1XComponent::vl53l1x_write_bytes_16(uint8_t a_register, const uint16_t *data, uint8_t len) {
  // we have to copy in order to be able to change byte order
  std::unique_ptr<uint16_t[]> temp{new uint16_t[len]};
  for (size_t i = 0; i < len; i++)
    temp[i] = i2c::htoi2cs(data[i]);
  return (this->write_register16(a_register, reinterpret_cast<const uint8_t *>(temp.get()), len * 2, true) == i2c::ERROR_OK);
}

bool VL53L1XComponent::vl53l1x_write_byte_16(uint16_t a_register, uint16_t data) {
  return this->vl53l1x_write_bytes_16(a_register, &data, 1);
}

bool VL53L1XComponent::vl53l1x_read_bytes(uint16_t a_register, uint8_t *data, uint8_t len) {
    return this->read_register16(a_register, data, len, true) == i2c::ERROR_OK;
}

bool VL53L1XComponent::vl53l1x_read_byte(uint16_t a_register, uint8_t *data) {
    return this->read_register16(a_register, data, 1, true) == i2c::ERROR_OK;
}

bool VL53L1XComponent::vl53l1x_read_bytes_16(uint16_t a_register, uint16_t *data, uint8_t len) {
  if (this->read_register16(a_register, reinterpret_cast<uint8_t *>(data), len * 2, true) != i2c::ERROR_OK)
    return false;
  for (size_t i = 0; i < len; i++)
    data[i] = i2c::i2ctohs(data[i]);
  return true;
}

bool VL53L1XComponent::vl53l1x_read_byte_16(uint16_t a_register, uint16_t *data) {
  return this->vl53l1x_read_bytes_16(a_register, data, 1);
}

} // namespace VL53L1X
} // namespace esphome
