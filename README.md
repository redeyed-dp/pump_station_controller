# PUMP STATION CONTROLLER

Functions:
- Air temperature measure
- Motor temperature measure
- Water leak detection
- Turning off pump station while leak or motor overheat detected
- MODBUS over LoRa interface

# LED indicators
1st RED
- blinks - sensor not connected
- lights up - motor overheat
- does not lights - OK

2nd RED
- blinks - sensor not connected
- lights up - low air temperature, water freezing in pipes is possible
- does not lights - OK

YELLOW
- blinks - water leak detected
- lights up - OK

BLUE
- LoRa sends response

## Home Assistant configuration


```
modbus:
  - name: modbus_to_lora
    type: rtuovertcp
    host: 127.0.0.1
    port: 502
    delay: 0
    message_wait_milliseconds: 300
    timeout: 30
    binary_sensors:
      - name: pump_water_leak
        slave: 1
        address: 0
        input_type: discrete_input
        device_class: moisture
    sensors:
      - name: pump_RSSI
        slave: 1
        address: 0
        input_type: input
        unit_of_measurement: dBm
      - name: pump_SNR
        slave: 1
        address: 1
        input_type: input
        unit_of_measurement: dBm
      - name: pump_motor_temp
        slave: 1
        address: 2
        input_type: input
        unit_of_measurement: °C
      - name: pump_air_temp
        slave: 1
        address: 3
        input_type: input
        unit_of_measurement: °C
```
