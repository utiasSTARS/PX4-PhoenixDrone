uint8 NUM_ACTUATOR_CONTROLS = 4
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 2
uint8 INDEX_RPM_LEFT = 0
uint8 INDEX_RPM_RIGHT = 1
uint8 INDEX_DEGREE_LEFT = 2
uint8 INDEX_DEGREE_RIGHT = 3
uint8 GROUP_INDEX_TS_ATTITUDE = 0
uint8 GROUP_INDEX_TS_ATTITUDE_ALTERNATE = 1
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[4] control

# TOPICS ts_actuator_controls ts_actuator_controls_0 ts_actuator_controls_1
# TOPICS ts_actuator_controls_virtual_fw ts_actuator_controls_virtual_mc

