#servo angle message

uint64 timestamp # time since system start (microseconds)

uint8 valid_mask # bit i is set when servo_angle[i] was received and validated

float32[4] servo_angle
