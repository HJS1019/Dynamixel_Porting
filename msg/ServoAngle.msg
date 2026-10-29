#servo angle message

uint64 timestamp # time since system start (microseconds)

uint8 valid_mask # bit i is set when servo_angle[i] was received and validated

uint8 active_mask # bit i is set when servo i is configured in the driver (-n)
uint8 ready_mask  # bit i is set when servo i responds, is in position mode and its torque is confirmed ON

float32[4] servo_angle
