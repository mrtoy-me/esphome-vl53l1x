
## ESPHome component for VL53L1X and VL53L4CD
This ESPHome external component is based on the Polulo VL53L1X Arduino Library
which in turn was based on the VL53L1X API by STMicroelectronics.
Therefore the licence terms are the same as those presented in the
Polulo VL53L1X Arduino Library.

Polulo VL53L1X Arduino Library states that most of the functionality of their
library is based on the VL53L1X API provided provided by ST (STSW-IMG007),
and some of the explanatory comments are quoted or paraphrased
from the API source code, API user manual (UM2356), and VL53L1X datasheet.
Therefore, the license terms for the API source code (BSD 3-clause
"New" or "Revised" License) also apply to this derivative work, as specified below.

Copyright (c) 2017, STMicroelectronics
Copyright (c) 2018-2022, Pololu Corporation
All Rights Reserved

**All Copyright licences are shown in vl53l1x.cpp and licence.md files**


# Usage: vl53l1x component on Github
This component requires Esphome version 2023.7.1 or later.

The following yaml can be used so ESPHome accesses the component files:
```
external_components:
  - source: github://mrtoy-me/esphome-components-test@main
    components: [ vl53l1x ]
    refresh: 0s
```

## Overview

This component supports VL53L1X (up to 4000mm range) and VL53L4CD (up to 1300mm range)
with default i2c address of 0x29. Timing budget (measurement period) is
set internally at 500ms. Ranging occurs continuously every 500ms, but measurements
are published at the specified update interval. Consequently, the update interval
must be greater or equal to 1 second.


# YAML configuration

Example typical configuration:
```
external_components:
  - source: github://mrtoy-me/esphome-vl53l1x@main
    components: [ vl53l1x ]
    refresh: 0s

i2c:
    sda: 21
    scl: 22
    frequency: 400khz
    scan: true

sensor:
  - platform: vl53l1x
    distance_mode: long
    distance:
      name: My Raw Distance
      id: my_raw_distance
    range_status:
      name: My Range Status
      id: my_range_status
    update_interval: 1s

  - platform: template
    name: My Distance
    device_class: "distance"
    state_class: "measurement"
    unit_of_measurement: "mm"
    accuracy_decimals: 0
    lambda: |-
       if (id(my_range_status).state > 0) {
         return NAN;
       } else {
         return id(my_raw_distance).state;
       }
    update_interval: 1s
```

VL53L1X sensor platform configuration variables:

- **distance_mode** (*Optional*): valid values **short** or **long**. Default to **long**.
  Note: the VL53L4CD sensor can have value **short**, if VL53L4CD is detected
  then distance mode is forced to **short**.

- **update_interval** (*Optional*): defaults to 60s. Update interval must be greater or equal to 1 second.


VL53L1X Sensor configuration headers:

Two sensors can be configured ***distance:*** and ***range_status:***
Distance has units mm while range status gives the status code of the distance measurement.
- **distance** (*Optional*): **name:** and **id:** can be defined
- **range_status** (*Optional*): **name:** and **id:** can be defined


# Range Status sensor values
The Range Status sensor values defined by this component differ from those used by the Polulo Arduino Library.
This component's Range Status sensor values are as follows:

- 0 = Range Valid

- 1 = Range valid, no wraparound check fail

- 2 = Range valid, below minimum range threshold

- 3 = Hardware or VCSEL fail

- 4 = Signal fail
  (signal below threshold, too weak for valid measurement)

- 5 = Out of bounds fail
  (nothing detected in range typically when target is at or more than maximum range)

- 6 = Sigma fail
  (standard deviation is above threshold indicating poor measurement repeatability)

- 7 = Wrap target fail
  (target is very reflective and is more than than maximum range)

- 8 = Minimum range fail
  (target is below minimum detection threshold)

- 9 = Undefined
